import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import numpy as np
from xarm.wrapper import XArmAPI

from umi.shared_memory.shared_memory_queue import SharedMemoryQueue, Empty
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from diffusion_policy.common.precise_sleep import precise_wait

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2

class XArmSDKInterpolationController(mp.Process):
    """
    To ensure sending command to the robot with predictable latency
    this controller needs its separate process (due to python GIL)
    """
    def __init__(self,
            shm_manager: SharedMemoryManager, 
            robot_ip, 
            frequency=125,  # XArm typically uses lower frequency
            max_pos_speed=250,  # mm/s
            max_rot_speed=1.0,  # rad/s
            launch_timeout=3,
            tcp_offset_pose=None,
            payload_mass=None,
            payload_cog=None,
            joints_init=None,
            joints_init_speed=20,  # degrees/s
            soft_real_time=False,
            verbose=False,
            receive_keys=None,
            get_max_k=None,
            receive_latency=0.0
            ):
        """
        frequency: control frequency in Hz
        max_pos_speed: maximum position speed in mm/s
        max_rot_speed: maximum rotation speed in rad/s
        tcp_offset_pose: 6d pose for TCP offset
        payload_mass: payload mass in kg
        payload_cog: center of gravity [x, y, z] in mm
        """
        if tcp_offset_pose is not None:
            tcp_offset_pose = np.array(tcp_offset_pose)
            assert tcp_offset_pose.shape == (6,)
        if payload_mass is not None:
            assert payload_mass >= 0
        if payload_cog is not None:
            payload_cog = np.array(payload_cog)
            assert payload_cog.shape == (3,)
            assert payload_mass is not None
        if joints_init is not None:
            joints_init = np.array(joints_init)
            assert joints_init.shape == (6,)  # XArm6

        super().__init__(name="XArmPositionalController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.launch_timeout = launch_timeout
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

        # build input queue
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

        # build ring buffer
        if receive_keys is None:
            receive_keys = [
                ('ActualTCPPose', 'get_position'),
                ('ActualTCPSpeed', 'get_position'),  # XArm doesn't provide TCP speed
                ('ActualQ', 'get_servo_angle'),
                ('ActualQd', 'get_joint_speeds')
            ]

        example = dict()
        for key, _ in receive_keys:
            if 'TCP' in key:
                example[key] = np.zeros(6)
            elif 'Q' in key:
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
        
    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[RTDEPositionalController] Controller process spawned at {self.pid}")

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

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        
    # ========= command methods ============
    def servoL(self, pose, duration=0.1):
        """
        duration: desired time to reach pose
        """
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

    # ========= receive APIs =============
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

        # Initialize XArm
        arm = XArmAPI(self.robot_ip)
        try:
            if self.verbose:
                print(f"[XArmPositionalController] Connect to robot: {self.robot_ip}")

            # Initialize robot
            arm.motion_enable(enable=True)
            arm.set_mode(0)  # Position control mode
            arm.set_state(state=0)
            
            # Set parameters
            if self.tcp_offset_pose is not None:
                arm.set_tcp_offset(self.tcp_offset_pose)
            if self.payload_mass is not None:
                if self.payload_cog is not None:
                    arm.set_tcp_load(self.payload_mass, self.payload_cog)
                else:
                    arm.set_tcp_load(self.payload_mass, [0, 0, 0])
            
            # Move to initial joint positions if specified
            if self.joints_init is not None:
                arm.set_servo_angle(angle=self.joints_init.tolist(), 
                                  speed=self.joints_init_speed, 
                                  wait=True)

            # Main control loop
            dt = 1. / self.frequency
            curr_pose = arm.get_position()[1]  # [code, [x,y,z,roll,pitch,yaw]]
            
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
                pose_command = pose_interp(t_now)
                
                # Send command to robot using servo_cartesian mode
                arm.set_servo_cartesian(pose_command.tolist())
                
                # Update robot state
                state = dict()
                for key, func_name in self.receive_keys:
                    result = getattr(arm, func_name)()
                    # Handle return format [code, values]
                    if isinstance(result, tuple):
                        state[key] = np.array(result[1])
                    else:
                        state[key] = np.array(result)
                
                t_recv = time.time()
                state['robot_receive_timestamp'] = t_recv
                state['robot_timestamp'] = t_recv - self.receive_latency
                self.ring_buffer.put(state)

                # Handle commands
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
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                # Maintain frequency
                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[XArmPositionalController] Actual frequency {1/(time.monotonic() - t_now)}")

        finally:
            if 'arm' in locals():
                arm.motion_enable(False)
                arm.disconnect()
            self.ready_event.set()

            if self.verbose:
                print(f"[XArmPositionalController] Disconnected from robot: {self.robot_ip}")