import rclpy 
from rclpy.node import Node 
from dls2_interfaces.msg import ArmState, ArmTrajectoryGenerator

import sys
sys.path.append("../../../unitree_z1/z1_controller/lib")
import unitree_arm_interface


class Z1HALNode(Node):
    def __init__(self):
        super().__init__('Z1_HAL_Node')

        arm_state_freq = 300  # Hz 
        self.timer = self.create_timer(1/arm_state_freq, self.compute_z1_hal_callback)
        self.publisher_arm_blind_state = self.create_publisher(ArmState,"/arm_blind_state", 1)
        self.subscriber_trajectory_generator_arm = self.create_subscription(ArmTrajectoryGenerator,"/arm_trajectory_generator", self.get_arm_trajectory_generator_callback, 1)

        # some init
        self.desired_arm_joints_position = np.zeros(6)
        self.desired_arm_joints_velocity = np.zeros(6)
        self.desired_gripper_position = 0.0
        self.Kp_arm = 0
        self.Kd_arm = 0
        self.Kp_gripper = 0.0
        self.Kd_gripper = 0.0

        np.set_printoptions(precision=3, suppress=True)
        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.armModel = self.arm._ctrlComp.armModel
        self.arm.setFsmLowcmd()


    def get_arm_trajectory_generator_callback(self, msg):

        joints_position = np.array(msg.desired_arm_joints_position)
        self.desired_arm_joints_position = joints_position
        self.Kp_arm = np.array(msg.arm_kp)[0]
        self.Kd_arm = np.array(msg.arm_kd)[0]


    def compute_simulator_step_callback(self):

        current_q = self.arm.lowstate.getQ()
        current_qd = self.arm.lowstate.getQd()
        current_gripper_q = self.arm.lowstate.getGripperQ()

        arm_state_msg = ArmState()
        arm_state_msg.joints_position = current_q.tolist()
        arm_state_msg.joints_velocity = current_qd.tolist()
        arm_state_msg.gripper_position = current_gripper_q.tolist()
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