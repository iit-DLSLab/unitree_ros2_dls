import rclpy
from rclpy.node import Node
from dls2_interface.msg import BlindState, ControlSignal
from sensor_msgs.msg import JointState

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
        self.publisher_arm_blind_state = self.create_publisher(BlindState,"/blind_state_arm", 1)
        self.publisher_joint_state = self.create_publisher(JointState, "/joint_states_arm", 1)
        self.subscriber_arm_control_signal = self.create_subscription(ControlSignal,"/control_signal_arm", self.get_arm_control_signal_callback, 1)

        # some init
        self.joint_names = [f"joint{i}" for i in range(1, 7)] + ["gripper"]

        np.set_printoptions(precision=3, suppress=True)
        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.armModel = self.arm._ctrlComp.armModel
        self.arm.setFsmLowcmd()
        self.arm._ctrlComp.lowcmd.setControlGain(np.array([0.0]*6), np.array([0.0]*6))
        self.arm._ctrlComp.lowcmd.setGripperGain(0.0, 0.0)


    def get_arm_control_signal_callback(self, msg):

        self.arm.setArmCmd(np.array(msg.joints_position[:6]), np.array(msg.joints_velocity[:6]), np.array(msg.joints_torques[:6]))
        self.arm.setGripperCmd(msg.joints_position[6], msg.joints_velocity[6], msg.joints_torques[6])

        # control gain - these values are divided for two constants since in the low-level
        # since in the unitree low-level control these constants are multiplied..
        # see https://support.unitree.com/home/en/Z1_developer/sdk_intro sec. 2.2.3
        self.arm._ctrlComp.lowcmd.setControlGain(np.array(msg.kp[:6])/25.6, np.array(msg.kd[:6])/0.0128)
        self.arm._ctrlComp.lowcmd.setGripperGain(msg.kp[6]/25.6, msg.kd[6]/0.0128)


    def compute_z1_hal_callback(self):
        self.arm.sendRecv()# udp connection

        current_q = self.arm.lowstate.getQ()
        current_qd = self.arm.lowstate.getQd()
        current_tau = self.arm.lowstate.getTau()
        current_gripper_q = self.arm.lowstate.getGripperQ()
        current_gripper_qd = self.arm.lowstate.getGripperQd()

        blind_state_msg = BlindState()
        blind_state_msg.joints_name = self.joint_names
        blind_state_msg.joints_position = current_q.tolist() + [current_gripper_q]
        blind_state_msg.joints_velocity = current_qd.tolist() + [current_gripper_qd]
        self.publisher_arm_blind_state.publish(blind_state_msg)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names[:6]
        joint_state_msg.position = current_q.tolist()
        joint_state_msg.velocity = current_qd.tolist()
        joint_state_msg.effort = current_tau.tolist()
        self.publisher_joint_state.publish(joint_state_msg)


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
