from xarm.wrapper import XArmAPI
import scipy.spatial.transform as st
import numpy as np

class XArmInterface:
    def __init__(self, ip='192.168.1.211'):
        # Initialize the xArm API with the robot's IP address
        self.robot = XArmAPI(port=ip)
        self.robot.connect()

    def get_ee_pose(self):
        # Get the end-effector pose in Cartesian coordinates
        code, pos = self.robot.get_position(is_radian=True)
        if code == 0:
            position = pos[:3]  # x, y, z
            rpy = pos[3:]       # roll, pitch, yaw
            rot_vec = st.Rotation.from_euler('xyz', rpy).as_rotvec()
            return np.concatenate([position, rot_vec]).tolist()
        else:
            raise Exception(f"Error getting EE pose, code: {code}")
    
    def get_joint_positions(self):
        # Get joint positions
        code, joint_states = self.robot.get_joint_states(is_radian=True)
        if code == 0:
            return joint_states[0].tolist()  # positions are the first element in the tuple
        else:
            raise Exception(f"Error getting joint positions, code: {code}")
    
    def get_joint_velocities(self):
        # Get joint velocities
        code, joint_states = self.robot.get_joint_states(is_radian=True)
        if code == 0:
            return joint_states[1].tolist()  # velocities are the second element in the tuple
        else:
            raise Exception(f"Error getting joint velocities, code: {code}")
    
    def move_to_joint_positions(self, positions, time_to_go):
        # Move to specified joint positions
        positions = [p * 180 / np.pi for p in positions]  # Convert radians to degrees
        self.robot.set_servo_angle(angle=positions, speed=20, mvacc=0.35, wait=True)
    
    def start_cartesian_impedance(self, Kx, Kxd):
        # xArm does not support impedance control directly, we can set force control if available
        self.robot.config_force_control(coord=0, c_axis=[0, 0, 0, 0, 0, 0], f_ref=[0, 0, 0, 0, 0, 0], limits=[0, 0, 0, 0, 0, 0])

    def update_desired_ee_pose(self, pose):
        position = pose[:3]
        rot_vec = pose[3:]
        quat = st.Rotation.from_rotvec(rot_vec).as_quat()
        # Update end-effector position and orientation
        self.robot.set_position(
            x=position[0], y=position[1], z=position[2],
            roll=quat[0], pitch=quat[1], yaw=quat[2],
            is_radian=False, speed=50, mvacc=0.2, wait=False
        )

    def terminate_current_policy(self):
        # Stop any motion or policies
        self.robot.motion_enable(enable=False)

