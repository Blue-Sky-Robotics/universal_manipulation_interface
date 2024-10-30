import os
import sys
import time

from xarm.wrapper import XArmAPI

ip = "192.168.1.240"

arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

#load and playback the trajectory file
arm.load_trajectory('coffee-start-position.traj')
arm.playback_trajectory()
time.sleep(5)

#initialize the gripper to closed postion
arm.set_gripper_mode(0)  # Set gripper to position mode
arm.set_gripper_enable(True)
arm.set_gripper_speed(5000)  # Set speed of gripper movement
arm.set_gripper_position(0, wait=True)

arm.load_trajectory('coffee-make-pos-01.traj')
arm.playback_trajectory()
time.sleep(5)

#opening the gripper
arm.set_gripper_position(700, wait=True)
#time.sleep(2)  # Allow time for gripper to open

arm.load_trajectory('coffee-make-pickup.traj')
arm.playback_trajectory()

#closing the gripper
arm.set_gripper_position(620, wait=True)
#time.sleep(2)

arm.load_trajectory('coffee-make-pass2.traj')
arm.playback_trajectory()
time.sleep(10)

#move arm back to starting position
arm.load_trajectory('coffee-start-position.traj')
arm.playback_trajectory()
time.sleep(2)

arm.reset(wait=True)