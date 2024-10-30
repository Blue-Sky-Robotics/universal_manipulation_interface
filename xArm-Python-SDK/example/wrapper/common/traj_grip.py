import os
import sys
import time

from xarm.wrapper import XArmAPI

ip = "192.168.1.213"

arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

code, values = arm.get_tgpio_digital()
if code == 0:
    print(f"All GPIO pin values: {values}")
else:
    print(f"Error getting GPIO pin values, code: {code}")
    
#load and playback the trajectory file
#arm.load_trajectory('test5.traj')
#arm.playback_trajectory()
#time.sleep(5)

#arm.set_linear_track_back_origin(wait=True)
#arm.set_linear_track_enable(True)
#arm.set_linear_track_speed(200)

#arm.set_linear_track_pos(300, wait=True)
#arm.set_linear_track_pos(700, wait=True)
#arm.set_linear_track_pos(0, wait=True)
#print(arm.get_linear_track_status())

#arm.set_gripper_enable(True)
#arm.set_gripper_speed(5000)  # Set speed of gripper movement
#arm.set_gripper_position(500, wait=True)  # Open gripper (max position)
#time.sleep(2)  # Allow time for gripper to open

#arm.set_position(x=0, y=0, z=0, roll=0, pitch=0, yaw=0, speed=100, wait=True)

#closing the gripper
#arm.set_gripper_position(0, wait=True)
#time.sleep(2)

#arm.reset(wait=True)