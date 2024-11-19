import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import scipy.interpolate as si
import scipy.spatial.transform as st
import numpy as np
from xarm import XArmAPI

from umi.shared_memory.shared_memory_queue import SharedMemoryQueue, Empty
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from umi.common.pose_util import pose_to_mat, mat_to_pose, transform_pose
from diffusion_policy.common.precise_sleep import precise_wait

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

class XArmInterface:
    def __init__(self, ip='192.168.1.211'):
        # Initialize transformation matrix only once
        self.tx_ur5_xarm = np.identity(4)

        self.tx_ur5_xarm[:3,:3] = st.Rotation.from_euler('xyz', [0, 0, 0]).as_matrix()
        # Cache the inverse transformation
        self.tx_xarm_ur5 = np.linalg.inv(self.tx_ur5_xarm)
        
        self.arm = XArmAPI(ip)
        self.last_pose = None
        self.target_pose = None
        self.verbose = True
        self.initial_pose = None
        self.initialize_robot()
    
    def initialize_robot(self):
        """Initialize the robot to be ready for motion commands"""
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        
        # Get initial position
        e_code, pose = self.arm.get_position(is_radian=True)
        if e_code == 0:
            self.initial_pose = np.array(pose)
            self.last_pose = self.initial_pose.copy()
            self.target_pose = self.initial_pose.copy()
            if self.verbose:
                print(f"Initial pose (XArm frame): {self.initial_pose}")
        
        # Switch to servo mode
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(0.1)
        
    def get_ee_pose(self):
        """Get end-effector pose and convert to UR5 convention"""
        e_code, xarm_pose = self.arm.get_position(is_radian=True)
        if self.verbose:
            print(f"XArm raw pose: {xarm_pose}")
            
        if e_code == 0:
            xarm_pose = np.array(xarm_pose)
            if self.initial_pose is None:
                self.initial_pose = xarm_pose
                return xarm_pose
                
            return xarm_pose
        else:
            raise RuntimeError(f"Failed to get position: {e_code}")
    
    def get_joint_positions(self):
        angles = self.arm.get_servo_angle()
        if angles[0] == 0:  # Success
            return np.array(angles[1][:6])
        else:
            raise RuntimeError(f"Failed to get joint angles: {angles[0]}")
    
    def get_state(self):
        """Get the current state of the robot"""
        return {
            'ActualTCPPose': self.get_ee_pose(),
            'TargetTCPPose': self.target_pose,  # Add target pose to state
            'ActualQ': self.get_joint_positions(),
            'ActualQd': self.get_joint_velocities()
        }
    
    def get_joint_velocities(self):
        # XArm doesn't provide direct velocity reading, estimate from positions
        curr_pos = self.get_joint_positions()
        time.sleep(0.001)  # Small delay for position change
        new_pos = self.get_joint_positions()
        return (new_pos - curr_pos) / 0.001

    def update_desired_ee_pose(self, pose: np.ndarray, speed=None, accel=None):
        """Update the desired end-effector pose"""
        pose = np.array(pose)
        
        if self.initial_pose is None:
            self.initial_pose = pose
            
        # Track the target pose
        self.target_pose = pose
        xarm_pose = pose
        
        if self.verbose:
            print(f"Target pose: {pose}")
        
        # Keep step size limiting for safety
        if self.last_pose is not None:
            step_size = np.linalg.norm(xarm_pose[:3] - self.last_pose[:3])
            if step_size > 0.005:  # 5mm maximum step
                fraction = 0.005 / step_size
                interpolated_pose = self.last_pose + fraction * (xarm_pose - self.last_pose)
                xarm_pose = interpolated_pose
        
        # Keep mode checking for safety
        if self.arm.mode != 1 or self.arm.state != 0:
            self.arm.set_mode(1)
            self.arm.set_state(0)
            time.sleep(0.01)
            
        # Execute command
        ret = self.arm.set_servo_cartesian(xarm_pose.tolist(), 
                                        speed=speed,
                                        mvacc=accel,
                                        is_radian=True,
                                        wait=False)
        
        # Track last pose for step size limiting
        if ret == 0:
            self.last_pose = xarm_pose
        else:
            error_msg = f"Failed to update pose: {ret}\n"
            error_msg += f"XArm pose: {xarm_pose}\n"
            error_msg += f"Speed: {speed}, Accel: {accel}"
            raise RuntimeError(error_msg)
                
        return ret

    def terminate_current_policy(self):
        self.arm.set_state(4)  # stop state
        self.arm.motion_enable(False)

    def close(self):
        self.arm.disconnect()

class XArmInterpolationController(mp.Process):
    def __init__(self,
            shm_manager: SharedMemoryManager, 
            robot_ip,
            frequency=100,
            launch_timeout=3,
            max_pos_speed=100,  # mm/s
            max_rot_speed=0.5,  # rad/s (~100 deg/s)
            tcp_offset_pose=None,
            payload_mass=None,
            payload_cog=None,
            joints_init=None,
            joints_init_speed=None,
            soft_real_time=False,
            verbose=False,
            get_max_k=None,
            receive_latency=0.0
            ):

        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (6,)

        super().__init__(name="XArmPositionalController")
        self.robot_ip = robot_ip
        self.frequency = min(max(frequency, 30), 250)
        self.launch_timeout = launch_timeout
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_cog = payload_cog
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        # Input queue setup
        example = {
            'cmd': Command.SERVOL.value,
            'target_pose': np.zeros((6,), dtype=np.float64),
            'duration': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=256
        )

        # Ring buffer setup
        receive_keys = [
            ('ActualTCPPose', 'get_ee_pose'),
            ('TargetTCPPose', 'target_pose'),
            ('ActualQ', 'get_joint_positions'),
            ('ActualQd', 'get_joint_velocities')
        ]
        
        example = dict()
        for key, func_name in receive_keys:
            if 'joint' in func_name:
                example[key] = np.zeros(6)
            else:
                example[key] = np.zeros(6)

        example['robot_receive_timestamp'] = time.time()
        example['robot_timestamp'] = time.time()
        
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = ring_buffer
        self.receive_keys = receive_keys

    def run(self):
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))
            
        robot = XArmInterface(self.robot_ip)

        try:
            if self.verbose:
                print(f"[XArmPositionalController] Connect to robot: {self.robot_ip}")
            
            if self.tcp_offset_pose is not None:
                robot.arm.set_tcp_offset(self.tcp_offset_pose)
            
            if self.payload_mass is not None and self.payload_cog is not None:
                robot.arm.set_tcp_load(self.payload_mass, self.payload_cog)
                
            if self.max_pos_speed is not None:
                robot.arm.set_tcp_maxacc(self.max_pos_speed)
            
            if self.joints_init is not None:
                robot.arm.set_servo_angle(angle=self.joints_init.tolist(), 
                                       speed=self.joints_init_speed,
                                       wait=True)

            dt = 1. / self.frequency
            curr_pose = robot.get_ee_pose()

            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[curr_pose]
            )

            t_start = time.monotonic()
            iter_idx = 0
            keep_running = True
            while keep_running:
                t_now = time.monotonic()
                target_pose = pose_interp(t_now)
                print(f"New target pose: {target_pose}")
                
                robot.update_desired_ee_pose(
                    target_pose,
                    speed=self.max_pos_speed,
                    accel=1000
                )

                # Update robot state
                # state = dict()
                # for key, func_name in self.receive_keys:
                #     state[key] = getattr(robot, func_name)()
                state = robot.get_state()     
                               
                t_recv = time.time()
                state['robot_receive_timestamp'] = t_recv
                state['robot_timestamp'] = t_recv - self.receive_latency
                self.ring_buffer.put(state)

                # Process commands
                try:
                    commands = self.input_queue.get_k(1)
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']

                    if cmd == Command.STOP.value:
                        keep_running = False
                        break
                    elif cmd == Command.SERVOL.value:
                        target_pose = command['target_pose']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        pose_interp = pose_interp.drive_to_waypoint(
                            pose=target_pose,
                            time=t_insert,
                            curr_time=curr_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print("[XArmPositionalController] New pose target:{} duration:{}s".format(
                                target_pose, duration))
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']
                        target_time = float(command['target_time'])
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[XArmPositionalController] Actual frequency {1/(time.monotonic() - t_now)}")

        finally:
            robot.terminate_current_policy()
            robot.close()
            self.ready_event.set()

            if self.verbose:
                print(f"[XArmPositionalController] Disconnected from robot: {self.robot_ip}")

    # Launch methods
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[XArmPositionalController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.STOP.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()
    
    def stop_wait(self):
        self.join()
    
    @property
    def is_ready(self):
        return self.ready_event.is_set()
    
    # Command methods
    def servoL(self, pose, duration=0.1):
        assert self.is_alive()
        assert(duration >= (1/self.frequency))
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SERVOL.value,
            'target_pose': pose,
            'duration': duration
        }
        self.input_queue.put(message)
    
    def schedule_waypoint(self, pose, target_time):
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': pose,
            'target_time': target_time
        }
        self.input_queue.put(message)
    
    # State methods
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.ring_buffer.get_all()