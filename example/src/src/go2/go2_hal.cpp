#include <chrono>
#include <cmath>
#include <cstring>
#include <string>

#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

// DLS2 related message
#include "dls2_msgs/msg/imu_msg.hpp"
#include "dls2_msgs/msg/base_state_msg.hpp"
#include "dls2_msgs/msg/blind_state_msg.hpp"
#include "dls2_msgs/msg/trajectory_generator_msg.hpp"

// Not sure is these defines are necessary
#define TOPIC_LOWCMD "/lowcmd"
#define TOPIC_LOWSTATE "/lowstate"

class LowLevelCmdNode : public rclcpp::Node {
  public:
    explicit LowLevelCmdNode() : Node("low_level_cmd_node") {
        Init();
  }
    void Init();

  private:
    void InitLowCmd();
    void LowStateMessageHandler(unitree_go::msg::LowState::SharedPtr msg);
    void TrajectoryGeneratorMessageHandler(dls2_msgs::msg::TrajectoryGeneratorMsg::SharedPtr msg);
    void LowCmdWrite();


    // Don't know what this function does
    std::string queryServiceName(std::string form, std::string name);


    unitree_go::msg::LowCmd low_cmd_;      // default init
    unitree_go::msg::LowState low_state_;  // default init
    unitree_go::msg::MotorState motor_[12];  // Unitree motor state message

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

    // DLS2 related publisher and subscriber
    dls2_msgs::msg::ImuMsg imu_;              // default init
    dls2_msgs::msg::BlindStateMsg blind_state_; // default init
    dls2_msgs::msg::TrajectoryGeneratorMsg trajectory_generator_; // default init

    rclcpp::Publisher<dls2_msgs::msg::ImuMsg>::SharedPtr imu_pub_;
    rclcpp::Publisher<dls2_msgs::msg::BlindStateMsg>::SharedPtr blind_state_pub_;
    rclcpp::Subscription<dls2_msgs::msg::TrajectoryGeneratorMsg>::SharedPtr trajectory_generator_sub_;

};


void LowLevelCmdNode::Init() {
  InitLowCmd();

  low_cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
  low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
        LowStateMessageHandler(msg);
      });
  
  // This are the DLS2 related publisher and subscriber
  imu_pub_ = this->create_publisher<dls2_msgs::msg::ImuMsg>("/dls2/imu", 10);
  blind_state_pub_ = this->create_publisher<dls2_msgs::msg::BlindStateMsg>("/dls2/blind_state", 10);
  trajectory_generator_sub_ = this->create_subscription<dls2_msgs::msg::TrajectoryGeneratorMsg>(
      "/dls2/trajectory_generator", 10, [this](const dls2_msgs::msg::TrajectoryGeneratorMsg::SharedPtr msg) {
        TrajectoryGeneratorMessageHandler(msg);
      });

}


void LowLevelCmdNode::InitLowCmd() {
  low_cmd_.head[0] = 0xFE;
  low_cmd_.head[1] = 0xEF;
  low_cmd_.level_flag = 0xFF;
  low_cmd_.gpio = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd[i].mode = (0x01);  // motor switch to servo (PMSM) mode
    low_cmd_.motor_cmd[i].q = (PosStopF);
    low_cmd_.motor_cmd[i].kp = (0);
    low_cmd_.motor_cmd[i].dq = (VelStopF);
    low_cmd_.motor_cmd[i].kd = (0);
    low_cmd_.motor_cmd[i].tau = (0);
  }
}


void LowLevelCmdNode::LowStateMessageHandler(
    const unitree_go::msg::LowState::SharedPtr msg) {
  low_state_ = *msg;


  // Publish DLS2 IMU message
  for (int i = 0; i < 4; i++) {
    imu_.orientation[i] = low_state_.imu_state.quaternion[i];
  }
  for (int i = 0; i < 3; i++) {
    imu_.angular_velocity[i] = low_state_.imu_state.gyroscope[i];
    imu_.linear_acceleration[i] = low_state_.imu_state.accelerometer[i];
  }
  imu_pub_->publish(imu_);


  // Publish DLS2 blind state message
  for (int i = 0; i < 12; i++) {
    motor_[i] = low_state_.motor_state[i];
    blind_state_.joints_position[i] = motor_[i].q;
    blind_state_.joints_velocity[i] = motor_[i].dq;
    blind_state_.joints_acceleration[i] = motor_[i].ddq;
    blind_state_.joints_effort[i] = motor_[i].tau_est;
  }
  blind_state_pub_->publish(blind_state_);
  
}


void LowLevelCmdNode::TrajectoryGeneratorMessageHandler(
    const dls2_msgs::msg::TrajectoryGeneratorMsg::SharedPtr msg) {

  trajectory_generator_ = *msg;

  for (int j = 0; j < 12; j++) {
      low_cmd_.motor_cmd[j].q = trajectory_generator_.joints_position[j];
      low_cmd_.motor_cmd[j].dq = trajectory_generator_.joints_velocity[j];
      low_cmd_.motor_cmd[j].kp = trajectory_generator_.kp[j];
      low_cmd_.motor_cmd[j].kd = trajectory_generator_.kd[j];
      low_cmd_.motor_cmd[j].tau = 0;
  }
  get_crc(low_cmd_);  // Check motor cmd crc
  low_cmd_pub_->publish(low_cmd_);

}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LowLevelCmdNode>();
  node->Init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
