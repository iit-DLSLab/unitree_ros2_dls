#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include "g1/g1_motion_switch_client.hpp"

// DLS2 related message
#include "dls2_interface/msg/imu.hpp"
#include "dls2_interface/msg/base_state.hpp"
#include "dls2_interface/msg/blind_state.hpp"
#include "dls2_interface/msg/trajectory_generator.hpp"

// Not sure is these defines are necessary
#define TOPIC_LOWCMD "/lowcmd"
#define TOPIC_LOWSTATE "/lowstate"

class LowLevelCmdNode : public rclcpp::Node {
  public:
    explicit LowLevelCmdNode() : Node("low_level_cmd_node") {
      Init();
      Start();
      std::cout << "HAL started correctly." << std::endl;
  }

  void Init();
  void Start();

  private:
    void InitLowCmd();
    void LowStateMessageHandler(unitree_go::msg::LowState::SharedPtr msg);
    void TrajectoryGeneratorMessageHandler(dls2_interface::msg::TrajectoryGenerator::SharedPtr msg);
    void LowCmdWrite();


    // Don't know what this function does
    std::string queryServiceName(std::string form, std::string name);
    int queryMotionStatus();
    std::shared_ptr<unitree::robot::g1::MotionSwitchClient> client_;

    unitree_go::msg::LowCmd low_cmd_;      // default init
    unitree_go::msg::LowState low_state_;  // default init
    unitree_go::msg::MotorState motor_[12];  // Unitree motor state message

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // DLS2 related publisher and subscriber
    dls2_interface::msg::Imu imu_;              // default init
    dls2_interface::msg::BlindState blind_state_; // default init
    dls2_interface::msg::TrajectoryGenerator trajectory_generator_; // default init

    rclcpp::Publisher<dls2_interface::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<dls2_interface::msg::BlindState>::SharedPtr blind_state_pub_;
    rclcpp::Subscription<dls2_interface::msg::TrajectoryGenerator>::SharedPtr trajectory_generator_sub_;

};


void LowLevelCmdNode::Init() {
  
  // Deactivate any control mode on the robot
  client_ = std::make_shared<unitree::robot::g1::MotionSwitchClient>(this);

  // HACK
  int32_t ret = client_->ReleaseMode();
  std::this_thread::sleep_for(std::chrono::seconds(1)); 
  ret = client_->ReleaseMode();
  std::this_thread::sleep_for(std::chrono::seconds(1)); 
  ret = client_->ReleaseMode();
  std::this_thread::sleep_for(std::chrono::seconds(1)); 

  /*while (true) {
    std::cout << "Try to deactivate the motion control-related service."
              << std::endl;
    int32_t ret = client_->ReleaseMode();
    
    if (ret == 0) {
      std::cout << "ReleaseMode succeeded." << std::endl;
      break;
    } else {
      std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    }

    // Sleep for 1 second to ensure proper initialization
    std::this_thread::sleep_for(std::chrono::seconds(1));    
  }*/

  // Create publishers and subscribers to talk with Unitree
  InitLowCmd(); 
  low_cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 1);
  low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
        LowStateMessageHandler(msg);
      });
  
  // Create publishers and subscribers to talk with the controller/DLS2
  imu_pub_ = this->create_publisher<dls2_interface::msg::Imu>("/imu", 1);
  blind_state_pub_ = this->create_publisher<dls2_interface::msg::BlindState>("/blind_state", 1);
  trajectory_generator_sub_ = this->create_subscription<dls2_interface::msg::TrajectoryGenerator>(
      "/trajectory_generator", 1, [this](const dls2_interface::msg::TrajectoryGenerator::SharedPtr msg) {
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

void LowLevelCmdNode::Start() {
  /*loop publishing thread*/
  timer_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] {
    LowCmdWrite();
    // timer_->cancel();
  });
}


// Publish states to DLS2 topics
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
  blind_state_.joints_position.resize(12);
  blind_state_.joints_velocity.resize(12);
  blind_state_.joints_acceleration.resize(12);
  blind_state_.joints_effort.resize(12);
  // First motors state is FR
  for (int i = 0; i < 3; i++) {
    motor_[i] = low_state_.motor_state[i];
    blind_state_.joints_position[i+3] = motor_[i].q;
    blind_state_.joints_velocity[i+3] = motor_[i].dq;
    blind_state_.joints_acceleration[i+3] = motor_[i].ddq;
    blind_state_.joints_effort[i+3] = motor_[i].tau_est;
  }

  // Second motors state is FL
  for (int i = 3; i < 6; i++) {
    motor_[i] = low_state_.motor_state[i];
    blind_state_.joints_position[i-3] = motor_[i].q;
    blind_state_.joints_velocity[i-3] = motor_[i].dq;
    blind_state_.joints_acceleration[i-3] = motor_[i].ddq;
    blind_state_.joints_effort[i-3] = motor_[i].tau_est;
  }

  // Third motors state is RR
  for (int i = 6; i < 9; i++) {
    motor_[i] = low_state_.motor_state[i];
    blind_state_.joints_position[i+3] = motor_[i].q;
    blind_state_.joints_velocity[i+3] = motor_[i].dq;
    blind_state_.joints_acceleration[i+3] = motor_[i].ddq;
    blind_state_.joints_effort[i+3] = motor_[i].tau_est;
  }

  // Fourth motors state is RL
  for (int i = 9; i < 12; i++) {
    motor_[i] = low_state_.motor_state[i];
    blind_state_.joints_position[i-3] = motor_[i].q;
    blind_state_.joints_velocity[i-3] = motor_[i].dq;
    blind_state_.joints_acceleration[i-3] = motor_[i].ddq;
    blind_state_.joints_effort[i-3] = motor_[i].tau_est;
  }

  blind_state_pub_->publish(blind_state_);
  
}


// Subscribe to DLS2 trajectory generator message and publish low level command to Unitree
void LowLevelCmdNode::TrajectoryGeneratorMessageHandler(
    const dls2_interface::msg::TrajectoryGenerator::SharedPtr msg) {

  trajectory_generator_ = *msg;
  
  // First motors state is FR
  for (int i = 0; i < 3; i++) {  
    low_cmd_.motor_cmd[i+3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motor_cmd[i+3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motor_cmd[i+3].kp = trajectory_generator_.kp[i];
    low_cmd_.motor_cmd[i+3].kd = trajectory_generator_.kd[i];
    low_cmd_.motor_cmd[i+3].tau = 0;
  }
  
  // Second motors state is FL
  for (int i = 3; i < 6; i++) {  
    low_cmd_.motor_cmd[i-3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motor_cmd[i-3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motor_cmd[i-3].kp = trajectory_generator_.kp[i];
    low_cmd_.motor_cmd[i-3].kd = trajectory_generator_.kd[i];
    low_cmd_.motor_cmd[i-3].tau = 0;
  }

  // Third motors state is RR
  for (int i = 6; i < 9; i++) {  
    low_cmd_.motor_cmd[i+3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motor_cmd[i+3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motor_cmd[i+3].kp = trajectory_generator_.kp[i];
    low_cmd_.motor_cmd[i+3].kd = trajectory_generator_.kd[i];
    low_cmd_.motor_cmd[i+3].tau = 0;
  }

  // Fourth motors state is RL
  for (int i = 9; i < 12; i++) {  
    low_cmd_.motor_cmd[i-3].q = trajectory_generator_.joints_position[i];
    low_cmd_.motor_cmd[i-3].dq = trajectory_generator_.joints_velocity[i];
    low_cmd_.motor_cmd[i-3].kp = trajectory_generator_.kp[i];
    low_cmd_.motor_cmd[i-3].kd = trajectory_generator_.kd[i];
    low_cmd_.motor_cmd[i-3].tau = 0;
  }

    //get_crc(low_cmd_);  // Check motor cmd crc
    //low_cmd_pub_->publish(low_cmd_);

}

void LowLevelCmdNode::LowCmdWrite() {

    get_crc(low_cmd_);  // Check motor cmd crc
    low_cmd_pub_->publish(low_cmd_);

}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LowLevelCmdNode>();
  node->Init();
  node->Start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
