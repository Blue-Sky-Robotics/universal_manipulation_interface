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
from umi.common.precise_sleep import precise_wait

class Command(enum.Enum):
    SHUTDOWN = 0
    SCHEDULE_WAYPOINT = 1
    RESTART_PUT = 2

class XArmGripperController(mp.Process):
    def __init__(self,
            shm_manager: SharedMemoryManager,
            robot_ip,
            frequency=30,
            speed=5000,  # XArm gripper speed
            get_max_k=None,
            command_queue_size=1024,
            launch_timeout=3,
            receive_latency=0.0,
            use_meters=False,
            verbose=False
            ):
        super().__init__(name="XArmGripperController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.speed = speed
        self.launch_timeout = launch_timeout
        self.receive_latency = receive_latency
        self.scale = 1000.0 if use_meters else 1.0
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 10)
        
        # build input queue
        example = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=command_queue_size
        )
        
        # build ring buffer
        example = {
            'gripper_state': 0,
            'gripper_position': 0.0,
            'gripper_velocity': 0.0,
            'gripper_force': 0.0,
            'gripper_measure_timestamp': time.time(),
            'gripper_receive_timestamp': time.time(),
            'gripper_timestamp': time.time()
        }
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
        self._last_position = None

    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[XArmGripperController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.SHUTDOWN.value
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
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        
    def schedule_waypoint(self, pos: float, target_time: float):
        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': pos,
            'target_time': target_time
        }
        self.input_queue.put(message)

    def restart_put(self, start_time):
        self.input_queue.put({
            'cmd': Command.RESTART_PUT.value,
            'target_time': start_time
        })
    
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.ring_buffer.get_all()
    
    def run(self):
        try:
            # Initialize XArm
            arm = XArmAPI(self.robot_ip)
            arm.motion_enable(enable=True)
            arm.clean_error()
            arm.set_mode(0)  # Position control mode
            arm.set_state(state=0)
            
            # Enable gripper
            arm.set_gripper_enable(True)
            arm.set_gripper_mode(0)  # Position mode
            arm.set_gripper_speed(self.speed)
            
            # Initialize gripper position
            code, curr_pos = arm.get_gripper_position()
            if code != 0:
                raise RuntimeError(f"Failed to get gripper position: {code}")
            self._last_position = curr_pos
            
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[[curr_pos,0,0,0,0,0]]
            )
            
            keep_running = True
            t_start = time.monotonic()
            iter_idx = 0
            
            while keep_running:
                t_now = time.monotonic()
                dt = 1 / self.frequency
                t_target = t_now
                target_pos = pose_interp(t_target)[0]
                
                # Move gripper to target position
                if abs(target_pos - self._last_position) > 0.1:  # Small threshold to avoid unnecessary commands
                    code = arm.set_gripper_position(target_pos, wait=False)
                    if code != 0:
                        print(f"Warning: Failed to set gripper position: {code}")
                    self._last_position = target_pos

                # Get gripper state
                code, position = arm.get_gripper_position()
                if code != 0:
                    position = self._last_position
                
                code, err_code = arm.get_gripper_err_code()
                gripper_state = 0 if err_code == 0 else 1
                
                # Update state
                state = {
                    'gripper_state': gripper_state,
                    'gripper_position': position,
                    # 'gripper_velocity': 0.0,  # XArm SDK doesn't provide velocity
                    # 'gripper_force': 0.0,     # XArm SDK doesn't provide force
                    'gripper_measure_timestamp': time.time(),
                    'gripper_receive_timestamp': time.time(),
                    'gripper_timestamp': time.time() - self.receive_latency
                }
                self.ring_buffer.put(state)

                # Handle commands
                try:
                    commands = self.input_queue.get_all()
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0
                
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']
                    
                    if cmd == Command.SHUTDOWN.value:
                        keep_running = False
                        break
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pos = command['target_pos'] * self.scale
                        target_time = command['target_time']
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=[target_pos, 0, 0, 0, 0, 0],
                            time=target_time,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    elif cmd == Command.RESTART_PUT.value:
                        t_start = command['target_time'] - time.time() + time.monotonic()
                        iter_idx = 1

                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1
                
                # Maintain frequency
                t_end = t_start + dt * iter_idx
                precise_wait(t_end=t_end, time_func=time.monotonic)
                
        finally:
            if 'arm' in locals():
                arm.set_gripper_enable(False)
                arm.disconnect()
            self.ready_event.set()
            if self.verbose:
                print(f"[XArmGripperController] Disconnected from robot: {self.robot_ip}")