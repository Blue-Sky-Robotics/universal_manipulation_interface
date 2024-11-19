import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import numpy as np
from xarm import XArmAPI

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
            speed=5000,  # XArm gripper speed (1-5000)
            get_max_k=None,
            command_queue_size=1024,
            launch_timeout=3,
            receive_latency=0.0,
            verbose=True  # Set to True for debugging
            ):
        super().__init__(name="XArmGripperController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.speed = min(max(speed, 1), 5000)  # Clamp speed to valid range
        self.launch_timeout = launch_timeout
        self.receive_latency = receive_latency
        self.verbose = verbose

        if get_max_k is None:
            get_max_k = int(frequency * 10)
        
        # build input queue
        example = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pos': 0.0,  # Normalized 0-1 position
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
            'gripper_position': 0.0,  # Normalized 0-1 position
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

    def _normalize_position(self, pos, reverse=False):
        """Convert between normalized (0-1) and actual gripper position (0-850)"""
        if reverse:
            return float(pos) / 850.0  # Actual to normalized 
        else:
            return int(pos * 850.0)  # Normalized to actual

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
            arm = XArmAPI(self.robot_ip)
            
            # Initialize gripper
            arm.set_gripper_enable(True)
            arm.clean_gripper_error()  # Clean any previous errors
            arm.set_gripper_mode(0)  # Position mode
            arm.set_gripper_speed(self.speed)
            time.sleep(1.0)  # Wait for initialization
            
            # Get initial position
            code, curr_pos = arm.get_gripper_position()
            if code != 0:
                print(f"Warning: Failed to get initial gripper position: {code}")
                curr_pos = 0
            
            self._last_position = self._normalize_position(curr_pos, reverse=True)
            if self.verbose:
                print(f"Initial gripper position (normalized): {self._last_position}")
            
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[[self._last_position]]
            )
            
            keep_running = True
            t_start = time.monotonic()
            iter_idx = 0
            
            while keep_running:
                t_now = time.monotonic()
                dt = 1 / self.frequency
                
                # Get target position
                target_norm_pos = pose_interp(t_now)[0]
                target_actual_pos = self._normalize_position(target_norm_pos)
                
                # Move gripper if position changed significantly
                if abs(target_norm_pos - self._last_position) > 0.01:
                    if self.verbose:
                        print(f"Moving gripper to {target_actual_pos} (normalized: {target_norm_pos})")
                    code = arm.set_gripper_position(target_actual_pos, wait=False)
                    if code != 0:
                        print(f"Warning: Failed to set gripper position: {code}")
                    self._last_position = target_norm_pos

                # Get current state
                code, position = arm.get_gripper_position()
                if code != 0:
                    if self.verbose:
                        print(f"Warning: Failed to get gripper position: {code}")
                    position = self._normalize_position(self._last_position)
                
                code, err_code = arm.get_gripper_err_code()
                if code != 0 and self.verbose:
                    print(f"Warning: Failed to get gripper error code: {code}")
                    err_code = 0
                
                # Update state
                t_recv = time.time()
                state = {
                    'gripper_state': 0 if err_code == 0 else 1,
                    'gripper_position': self._normalize_position(position, reverse=True),
                    'gripper_measure_timestamp': t_recv,
                    'gripper_receive_timestamp': t_recv,
                    'gripper_timestamp': t_recv - self.receive_latency
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
                        target_pos = command['target_pos']  # Already normalized 0-1
                        target_time = command['target_time']
                        # Translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=[target_pos],
                            time=target_time,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                        if self.verbose:
                            print(f"Scheduled gripper waypoint: {target_pos} at time {target_time}")
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