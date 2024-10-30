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
from diffusion_policy.common.precise_sleep import precise_wait
from umi.common.pose_util import pose_to_mat, mat_to_pose

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

# XArm tool center point (TCP) transformation matrix
# Adjust these values based on your end-effector setup
tx_flange_tip = np.identity(4)
tx_flange_tip[:3, 3] = np.array([0, 0, 0.1])  # Adjust these offsets for your tool
tx_tip_flange = np.linalg.inv(tx_flange_tip)

class XArmInterface:
    def __init__(self, ip='192.168.1.211'):  # Replace with your XArm's IP
        self.arm = XArmAPI(ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)  # Set to position control mode
        self.arm.set_state(state=0)  # Set to ready state
        # Set maximum acceleration and velocity
        self.arm.set_tcp_maxacc(10000)
        self.arm.set_tcp_maxacc(10000)
        # Set tool offset
        self.arm.set_tcp_offset(tx_flange_tip[:3, 3].tolist() + [0, 0, 0])

    def get_ee_pose(self):
        # XArm returns [x, y, z, roll, pitch, yaw]
        pose = self.arm.get_position()
        if pose[0] == 0:  # Check if successful
            return np.array(pose[1])  # XArm SDK returns [code, [x,y,z,r,p,y]]
        else:
            raise RuntimeError(f"Failed to get position: {pose[0]}")
    
    def get_joint_positions(self):
        angles = self.arm.get_servo_angle()
        if angles[0] == 0:
            return np.array(angles[1])
        else:
            raise RuntimeError(f"Failed to get joint angles: {angles[0]}")
    
    def get_joint_velocities(self):
        # XArm doesn't provide direct velocity reading, estimate from positions
        curr_pos = self.get_joint_positions()
        time.sleep(0.001)  # Small delay for position change
        new_pos = self.get_joint_positions()
        return (new_pos - curr_pos) / 0.001

    def move_to_joint_positions(self, positions: np.ndarray, time_to_go: float):
        # Convert time_to_go to appropriate speed
        max_diff = np.max(np.abs(positions - self.get_joint_positions()))
        speed = max_diff / time_to_go * 60  # Convert to degrees/minute
        speed = min(speed, 1000)  # Limit speed to safe value
        
        ret = self.arm.set_servo_angle(angle=positions.tolist(), 
                                     speed=speed,
                                     wait=False,
                                     radius=0)
        if ret != 0:
            raise RuntimeError(f"Failed to move to joint positions: {ret}")

    def update_desired_ee_pose(self, pose: np.ndarray):
        # Convert pose to XArm format if necessary
        ret = self.arm.set_servo_cartesian(pose.tolist(), wait=False)
        if ret != 0:
            raise RuntimeError(f"Failed to update pose: {ret}")

    def terminate_current_policy(self):
        self.arm.stop_move()
        self.arm.motion_enable(False)

    def close(self):
        self.arm.disconnect()

class XArmInterpolationController(mp.Process):
    def __init__(self,
        shm_manager: SharedMemoryManager, 
        robot_ip,
        frequency=100,  # XArm typically operates at lower frequency than Franka
        launch_timeout=3,
        joints_init=None,
        joints_init_duration=None,
        soft_real_time=False,
        verbose=False,
        get_max_k=None,
        receive_latency=0.0
        ):

        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (6,)  # XArm6 has 6 joints

        super().__init__(name="XArmPositionalController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.launch_timeout = launch_timeout
        self.joints_init = joints_init
        self.joints_init_duration = joints_init_duration
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        # Input queue setup remains the same
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

        # Ring buffer setup adjusted for XArm
        receive_keys = [
            ('ActualTCPPose', 'get_ee_pose'),
            ('ActualQ', 'get_joint_positions'),
            ('ActualQd','get_joint_velocities'),
        ]
        example = dict()
        for key, func_name in receive_keys:
            if 'joint' in func_name:
                example[key] = np.zeros(6)  # XArm6 has 6 joints
            elif 'ee_pose' in func_name:
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
            
    # The launch methods remain the same
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
    
    # Context manager methods remain the same
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # Command methods remain the same
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
    
    # State methods remain the same
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.ring_buffer.get_all()
    
    def run(self):
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))
            
        robot = XArmInterface(self.robot_ip)

        try:
            if self.verbose:
                print(f"[XArmPositionalController] Connect to robot: {self.robot_ip}")
            
            if self.joints_init is not None:
                robot.move_to_joint_positions(
                    positions=np.asarray(self.joints_init),
                    time_to_go=self.joints_init_duration
                )

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
                tip_pose = pose_interp(t_now)
                flange_pose = mat_to_pose(pose_to_mat(tip_pose) @ tx_tip_flange)

                robot.update_desired_ee_pose(flange_pose)

                state = dict()
                for key, func_name in self.receive_keys:
                    state[key] = getattr(robot, func_name)()
                    
                t_recv = time.time()
                state['robot_receive_timestamp'] = t_recv
                state['robot_timestamp'] = t_recv - self.receive_latency
                self.ring_buffer.put(state)

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
                            last_waypoint_time=last_waypoint_time
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