#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "unitree_legged_sdk/unitree_legged_sdk.h"

// DLS2 related message
#include "dls2_interface/msg/imu.hpp"
#include "dls2_interface/msg/base_state.hpp"
#include "dls2_interface/msg/blind_state.hpp"
#include "dls2_interface/msg/trajectory_generator.hpp"
#include "dls2_interface/msg/control_signal.hpp"

using namespace UNITREE_LEGGED_SDK;

// Same low-level Aliengo communication settings used by example_torque_aliengo.
constexpr uint16_t TARGET_PORT = 8007;
constexpr uint16_t LOCAL_PORT = 8082;
constexpr char TARGET_IP[] = "192.168.123.10";
constexpr int LOW_CMD_LENGTH = 610;
constexpr int LOW_STATE_LENGTH = 771;
constexpr float CONTROL_DT = 0.002f;

class LowLevelCmdNode : public rclcpp::Node {
  public:
    explicit LowLevelCmdNode()
        : Node("low_level_cmd_node"),
          safe_(LeggedType::Aliengo),
          udp_(LOCAL_PORT, TARGET_IP, TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH) {
      Init();
      Start();
      std::cout << "HAL started correctly." << std::endl;
    }

    void Init();
    void Start();

  private:
    void InitLowCmd();
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void PublishLowState();
    void TrajectoryGeneratorMessageHandler(dls2_interface::msg::TrajectoryGenerator::SharedPtr msg);
    void ControlSignalMessageHandler(dls2_interface::msg::ControlSignal::SharedPtr msg);

    Safety safe_;
    UDP udp_;
    LowCmd low_cmd_{};
    LowState low_state_{};
    MotorState motor_[12]{};
    std::mutex low_cmd_mutex_;

    std::unique_ptr<LoopFunc> loop_control_;
    std::unique_ptr<LoopFunc> loop_udp_send_;
    std::unique_ptr<LoopFunc> loop_udp_recv_;

    // DLS2 related publisher and subscriber
    dls2_interface::msg::Imu imu_;              // default init
    dls2_interface::msg::BlindState blind_state_; // default init
    dls2_interface::msg::TrajectoryGenerator trajectory_generator_; // default init
    dls2_interface::msg::ControlSignal control_signal_; // default init

    rclcpp::Publisher<dls2_interface::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<dls2_interface::msg::BlindState>::SharedPtr blind_state_pub_;
    rclcpp::Subscription<dls2_interface::msg::TrajectoryGenerator>::SharedPtr trajectory_generator_sub_;
    rclcpp::Subscription<dls2_interface::msg::ControlSignal>::SharedPtr control_signal_sub_;
};


void LowLevelCmdNode::Init() {
  InitEnvironment();
  InitLowCmd();

  // Create publishers and subscribers to talk with the controller/DLS2
  imu_pub_ = this->create_publisher<dls2_interface::msg::Imu>("/imu", 1);
  blind_state_pub_ = this->create_publisher<dls2_interface::msg::BlindState>("/blind_state", 1);
  trajectory_generator_sub_ = this->create_subscription<dls2_interface::msg::TrajectoryGenerator>(
      "/trajectory_generator", 1, [this](const dls2_interface::msg::TrajectoryGenerator::SharedPtr msg) {
        TrajectoryGeneratorMessageHandler(msg);
      });
  control_signal_sub_ = this->create_subscription<dls2_interface::msg::ControlSignal>(
      "/control_signal", 1, [this](const dls2_interface::msg::ControlSignal::SharedPtr msg) {
        ControlSignalMessageHandler(msg);
      });
}


void LowLevelCmdNode::InitLowCmd() {
  udp_.InitCmdData(low_cmd_);
  low_cmd_.levelFlag = LOWLEVEL;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motorCmd[i].mode = 0x01;  // motor switch to servo (PMSM) mode
    low_cmd_.motorCmd[i].q = PosStopF;
    low_cmd_.motorCmd[i].Kp = 0;
    low_cmd_.motorCmd[i].dq = VelStopF;
    low_cmd_.motorCmd[i].Kd = 0;
    low_cmd_.motorCmd[i].tau = 0;
  }

  udp_.SetSend(low_cmd_);
}


void LowLevelCmdNode::Start() {
  loop_udp_send_ = std::make_unique<LoopFunc>(
      "udp_send", CONTROL_DT, 3, boost::bind(&LowLevelCmdNode::UDPSend, this));
  loop_udp_recv_ = std::make_unique<LoopFunc>(
      "udp_recv", CONTROL_DT, 3, boost::bind(&LowLevelCmdNode::UDPRecv, this));
  loop_control_ = std::make_unique<LoopFunc>(
      "control_loop", CONTROL_DT, boost::bind(&LowLevelCmdNode::RobotControl, this));

  loop_udp_send_->start();
  loop_udp_recv_->start();
  loop_control_->start();
}


void LowLevelCmdNode::UDPRecv() {
  udp_.Recv();
}


void LowLevelCmdNode::UDPSend() {
  udp_.Send();
}


void LowLevelCmdNode::RobotControl() {
  udp_.GetRecv(low_state_);
  PublishLowState();

  std::lock_guard<std::mutex> lock(low_cmd_mutex_);
  udp_.SetSend(low_cmd_);
}


// Publish states to DLS2 topics
void LowLevelCmdNode::PublishLowState() {
  // We use xywz convention
  imu_.orientation[0] = low_state_.imu.quaternion[1];
  imu_.orientation[1] = low_state_.imu.quaternion[2];
  imu_.orientation[2] = low_state_.imu.quaternion[3];
  imu_.orientation[3] = low_state_.imu.quaternion[0];
  for (int i = 0; i < 3; i++) {
    imu_.angular_velocity[i] = low_state_.imu.gyroscope[i];
    imu_.linear_acceleration[i] = low_state_.imu.accelerometer[i];
  }
  imu_pub_->publish(imu_);


  // Publish DLS2 blind state message
  blind_state_.joints_position.resize(12);
  blind_state_.joints_velocity.resize(12);
  blind_state_.joints_acceleration.resize(12);
  blind_state_.joints_effort.resize(12);
  // First motors state is FR
  for (int i = 0; i < 3; i++) {
    motor_[i] = low_state_.motorState[i];
    blind_state_.joints_position[i+3] = motor_[i].q;
    blind_state_.joints_velocity[i+3] = motor_[i].dq;
    blind_state_.joints_acceleration[i+3] = motor_[i].ddq;
    blind_state_.joints_effort[i+3] = motor_[i].tauEst;
  }

  // Second motors state is FL
  for (int i = 3; i < 6; i++) {
    motor_[i] = low_state_.motorState[i];
    blind_state_.joints_position[i-3] = motor_[i].q;
    blind_state_.joints_velocity[i-3] = motor_[i].dq;
    blind_state_.joints_acceleration[i-3] = motor_[i].ddq;
    blind_state_.joints_effort[i-3] = motor_[i].tauEst;
  }

  // Third motors state is RR
  for (int i = 6; i < 9; i++) {
    motor_[i] = low_state_.motorState[i];
    blind_state_.joints_position[i+3] = motor_[i].q;
    blind_state_.joints_velocity[i+3] = motor_[i].dq;
    blind_state_.joints_acceleration[i+3] = motor_[i].ddq;
    blind_state_.joints_effort[i+3] = motor_[i].tauEst;
  }

  // Fourth motors state is RL
  for (int i = 9; i < 12; i++) {
    motor_[i] = low_state_.motorState[i];
    blind_state_.joints_position[i-3] = motor_[i].q;
    blind_state_.joints_velocity[i-3] = motor_[i].dq;
    blind_state_.joints_acceleration[i-3] = motor_[i].ddq;
    blind_state_.joints_effort[i-3] = motor_[i].tauEst;
  }

  blind_state_pub_->publish(blind_state_);
}


// Subscribe to DLS2 trajectory generator message and publish low level command to Unitree
void LowLevelCmdNode::TrajectoryGeneratorMessageHandler(
    const dls2_interface::msg::TrajectoryGenerator::SharedPtr msg) {

  trajectory_generator_ = *msg;
  std::lock_guard<std::mutex> lock(low_cmd_mutex_);

  // First motors state is FR
  for (int i = 0; i < 3; i++) {
    low_cmd_.motorCmd[i+3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motorCmd[i+3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motorCmd[i+3].Kp = trajectory_generator_.kp[i];
    low_cmd_.motorCmd[i+3].Kd = trajectory_generator_.kd[i];
  }

  // Second motors state is FL
  for (int i = 3; i < 6; i++) {
    low_cmd_.motorCmd[i-3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motorCmd[i-3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motorCmd[i-3].Kp = trajectory_generator_.kp[i];
    low_cmd_.motorCmd[i-3].Kd = trajectory_generator_.kd[i];
  }

  // Third motors state is RR
  for (int i = 6; i < 9; i++) {
    low_cmd_.motorCmd[i+3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motorCmd[i+3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motorCmd[i+3].Kp = trajectory_generator_.kp[i];
    low_cmd_.motorCmd[i+3].Kd = trajectory_generator_.kd[i];
  }

  // Fourth motors state is RL
  for (int i = 9; i < 12; i++) {
    low_cmd_.motorCmd[i-3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motorCmd[i-3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motorCmd[i-3].Kp = trajectory_generator_.kp[i];
    low_cmd_.motorCmd[i-3].Kd = trajectory_generator_.kd[i];
  }
}


void LowLevelCmdNode::ControlSignalMessageHandler(
    const dls2_interface::msg::ControlSignal::SharedPtr msg) {

  control_signal_ = *msg;
  std::lock_guard<std::mutex> lock(low_cmd_mutex_);

  // First motors state is FR
  for (int i = 0; i < 3; i++) {
    low_cmd_.motorCmd[i+3].tau = control_signal_.torques[i];
  }

  // Second motors state is FL
  for (int i = 3; i < 6; i++) {
    low_cmd_.motorCmd[i-3].tau = control_signal_.torques[i];
  }

  // Third motors state is RR
  for (int i = 6; i < 9; i++) {
    low_cmd_.motorCmd[i+3].tau = control_signal_.torques[i];
  }

  // Fourth motors state is RL
  for (int i = 9; i < 12; i++) {
    low_cmd_.motorCmd[i-3].tau = control_signal_.torques[i];
  }
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LowLevelCmdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
