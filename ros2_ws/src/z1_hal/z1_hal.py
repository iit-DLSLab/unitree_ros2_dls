import rclpy 
from rclpy.node import Node 
from dls2_interface.msg import ArmState, ArmTrajectoryGenerator, ArmControlSignal

import numpy as np
import sys
import os
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path + "/../../../unitree_z1/z1_sdk/lib")
sys.path.append(dir_path + "/../../../unitree_z1/z1_sdk/examples_py")
import unitree_arm_interface


class Z1HALNode(Node):
    def __init__(self):
        super().__init__('Z1_HAL_Node')

        arm_state_freq = 300  # Hz 
        self.timer = self.create_timer(1/arm_state_freq, self.compute_z1_hal_callback)
        self.publisher_arm_blind_state = self.create_publisher(ArmState,"/arm_state", 1)
        self.subscriber_trajectory_generator_arm = self.create_subscription(ArmTrajectoryGenerator,"/arm_trajectory_generator", self.get_arm_trajectory_generator_callback, 1)
        self.subscriber_arm_control_signal = self.create_subscription(ArmControlSignal,"/arm_control_signal", self.get_arm_control_signal_callback, 1)

        # some init
        self.desired_arm_joints_position = np.zeros(6)
        self.desired_arm_joints_velocity = np.zeros(6)
        self.desired_arm_joints_torque = np.zeros(6)
        self.desired_gripper_position = 0.0
        self.desired_gripper_velocity = 0.0
        self.desired_gripper_torque = 0.0
        self.Kp_arm = 0
        self.Kd_arm = 0
        self.Kp_gripper = 0.0
        self.Kd_gripper = 0.0

        np.set_printoptions(precision=3, suppress=True)
        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.armModel = self.arm._ctrlComp.armModel
        self.arm.setFsmLowcmd()


    def get_arm_trajectory_generator_callback(self, msg):
        
        self.arm.setArmCmd(np.array(msg.desired_arm_joints_position), np.array(msg.desired_arm_joints_velocity), self.desired_arm_joints_torque)
        self.arm.setGripperCmd(msg.desired_arm_gripper_position, msg.desired_arm_gripper_velocity, self.desired_gripper_torque)
        self.arm._ctrlComp.lowcmd.setControlGain(np.array(msg.arm_kp), np.array(msg.arm_kd))


    def get_arm_control_signal_callback(self, msg):

        self.desired_arm_joints_torque = np.array(msg.desired_arm_joints_torque)
        self.desired_gripper_torque = msg.desired_arm_gripper_torque


    def compute_z1_hal_callback(self):
        self.arm.sendRecv()# udp connection

        current_q = self.arm.lowstate.getQ()
        current_qd = self.arm.lowstate.getQd()
        current_gripper_q = self.arm.lowstate.getGripperQ()

        arm_state_msg = ArmState()
        arm_state_msg.joints_position = current_q.tolist()
        arm_state_msg.joints_velocity = current_qd.tolist()
        arm_state_msg.gripper_position = current_gripper_q
        self.publisher_arm_blind_state.publish(arm_state_msg)


#---------------------------
if __name__ == '__main__':
    
    print('Hello from the z1 hal node.')
    
    rclpy.init()
    z1_hal_node = Z1HALNode()
    rclpy.spin(z1_hal_node)

    z1_hal_node.destroy_node()
    rclpy.shutdown()

    print("z1 hal node is stopped")
    exit(0)