#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_go/msg/lidar_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

#include "g1/g1_motion_switch_client.hpp"

// DLS2 related message
#include "dls2_interface/msg/imu.hpp"
#include "dls2_interface/msg/base_state.hpp"
#include "dls2_interface/msg/blind_state.hpp"
#include "dls2_interface/msg/control_signal.hpp"


class LowLevelCmdNode : public rclcpp::Node {
  public:
    explicit LowLevelCmdNode() : Node("low_level_cmd_node") {
      Init();
      Start();
      std::cout << "HAL GO2 started correctly." << std::endl;
  }

  void Init();
  void Start();

  private:
    void InitLowCmd();
    void LidarStateMessageHandler(unitree_go::msg::LidarState::SharedPtr msg);
    void LowStateMessageHandler(unitree_go::msg::LowState::SharedPtr msg);
    void ControlSignalMessageHandler(dls2_interface::msg::ControlSignal::SharedPtr msg);
    void LowCmdWrite();
    double SynchronizedTimestamp();


    // Don't know what this function does
    std::string queryServiceName(std::string form, std::string name);
    int queryMotionStatus();
    std::shared_ptr<unitree::robot::g1::MotionSwitchClient> client_;

    unitree_go::msg::LowCmd low_cmd_;      // default init
    unitree_go::msg::LowState low_state_;  // default init
    unitree_go::msg::MotorState motor_[12];  // Unitree motor state message

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
    rclcpp::Subscription<unitree_go::msg::LidarState>::SharedPtr lidar_state_sub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double lidar_time_offset_{0.0};
    double last_synchronized_stamp_{0.0};
    bool lidar_time_synchronized_{false};

    // DLS2 related publisher and subscriber
    dls2_interface::msg::Imu imu_;              // default init
    dls2_interface::msg::BlindState blind_state_; // default init
    dls2_interface::msg::ControlSignal control_signal_; // default init
    sensor_msgs::msg::JointState joint_state_; // default init

    rclcpp::Publisher<dls2_interface::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<dls2_interface::msg::BlindState>::SharedPtr blind_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<dls2_interface::msg::ControlSignal>::SharedPtr control_signal_sub_;

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

  // Create publishers and subscribers to talk with Unitree
  InitLowCmd(); 
  low_cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 1);
  
  low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
    "/lowstate", 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
      LowStateMessageHandler(msg);
    }
  );
  
  lidar_state_sub_ = this->create_subscription<unitree_go::msg::LidarState>(
    "/utlidar/lidar_state", rclcpp::SensorDataQoS(),
    [this](const unitree_go::msg::LidarState::SharedPtr msg) {
      LidarStateMessageHandler(msg);
    }
  );
  
  // Create publishers and subscribers to talk with the controller/DLS2
  imu_pub_ = this->create_publisher<dls2_interface::msg::Imu>("/imu", 1);
  blind_state_pub_ = this->create_publisher<dls2_interface::msg::BlindState>("/blind_state_quadruped", 1);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_quadruped", 1);
  joint_state_.name = {
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  blind_state_.joints_name = joint_state_.name;
  control_signal_sub_ = this->create_subscription<dls2_interface::msg::ControlSignal>(
    "/control_signal_quadruped", 1, [this](const dls2_interface::msg::ControlSignal::SharedPtr msg) {
      ControlSignalMessageHandler(msg);
    }
  );
}


void LowLevelCmdNode::LidarStateMessageHandler(
    const unitree_go::msg::LidarState::SharedPtr msg) {
  if (!std::isfinite(msg->stamp) || msg->stamp <= 0.0) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Ignoring invalid LidarState stamp: %.9f", msg->stamp);
    return;
  }

  const double observed_offset = msg->stamp - this->now().seconds();
  constexpr double kOffsetFilterGain = 0.05;
  constexpr double kClockResetThreshold = 1.0;

  if (!lidar_time_synchronized_) {
    lidar_time_offset_ = observed_offset;
    lidar_time_synchronized_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "GO2 timestamps synchronized to %s (offset: %.6f s)",
        "/utlidar/lidar_state", lidar_time_offset_);
  } 
  else if (std::abs(observed_offset - lidar_time_offset_) >
             kClockResetThreshold) {
    RCLCPP_WARN(
        this->get_logger(),
        "Lidar clock discontinuity detected; resetting timestamp offset "
        "from %.6f s to %.6f s",
        lidar_time_offset_, observed_offset);
    lidar_time_offset_ = observed_offset;
    last_synchronized_stamp_ = 0.0;
  } 
  else {
    lidar_time_offset_ +=
        kOffsetFilterGain * (observed_offset - lidar_time_offset_);
  }
}


double LowLevelCmdNode::SynchronizedTimestamp() {
  double stamp = this->now().seconds();
  if (lidar_time_synchronized_) {
    stamp += lidar_time_offset_;
  }

  // Keep the published time monotonic while the offset filter converges.
  if (stamp <= last_synchronized_stamp_) {
    stamp = std::nextafter(last_synchronized_stamp_, INFINITY);
  }
  last_synchronized_stamp_ = stamp;
  return stamp;
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
  timer_ = this->create_wall_timer(std::chrono::milliseconds(2), [this] {LowCmdWrite();});
}


// Publish states to DLS2 topics
void LowLevelCmdNode::LowStateMessageHandler(const unitree_go::msg::LowState::SharedPtr msg) {
  low_state_ = *msg;
  const double synchronized_stamp = SynchronizedTimestamp();


  // Publish DLS2 IMU message
  // We use xywz convention
  imu_.orientation[0] = low_state_.imu_state.quaternion[1]; 
  imu_.orientation[1] = low_state_.imu_state.quaternion[2];
  imu_.orientation[2] = low_state_.imu_state.quaternion[3];
  imu_.orientation[3] = low_state_.imu_state.quaternion[0];
  for (int i = 0; i < 3; i++) {
    imu_.angular_velocity[i] = low_state_.imu_state.gyroscope[i];
    imu_.linear_acceleration[i] = low_state_.imu_state.accelerometer[i];
  }
  imu_.timestamp = synchronized_stamp;
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

  blind_state_.timestamp = synchronized_stamp;
  blind_state_pub_->publish(blind_state_);

  joint_state_.header.stamp = rclcpp::Time(std::llround(synchronized_stamp * 1e9));
  joint_state_.position = blind_state_.joints_position;
  joint_state_.velocity = blind_state_.joints_velocity;
  joint_state_.effort = blind_state_.joints_effort;
  joint_state_pub_->publish(joint_state_);
  
}


// Subscribe to DLS2 control signal message and publish low level command to Unitree
void LowLevelCmdNode::ControlSignalMessageHandler(const dls2_interface::msg::ControlSignal::SharedPtr msg) {

  control_signal_ = *msg;
  
  // First motors state is FR
  for (int i = 0; i < 3; i++) {  
    low_cmd_.motor_cmd[i+3].q = control_signal_.joints_position[i];
    low_cmd_.motor_cmd[i+3].dq = control_signal_.joints_velocity[i];
    low_cmd_.motor_cmd[i+3].tau = control_signal_.joints_torques[i];
    low_cmd_.motor_cmd[i+3].kp = control_signal_.kp[i];
    low_cmd_.motor_cmd[i+3].kd = control_signal_.kd[i];
  }
  
  // Second motors state is FL
  for (int i = 3; i < 6; i++) {  
    low_cmd_.motor_cmd[i-3].q = control_signal_.joints_position[i];
    low_cmd_.motor_cmd[i-3].dq = control_signal_.joints_velocity[i];
    low_cmd_.motor_cmd[i-3].tau = control_signal_.joints_torques[i];
    low_cmd_.motor_cmd[i-3].kp = control_signal_.kp[i];
    low_cmd_.motor_cmd[i-3].kd = control_signal_.kd[i];
  }

  // Third motors state is RR
  for (int i = 6; i < 9; i++) {  
    low_cmd_.motor_cmd[i+3].q = control_signal_.joints_position[i];
    low_cmd_.motor_cmd[i+3].dq = control_signal_.joints_velocity[i];
    low_cmd_.motor_cmd[i+3].tau = control_signal_.joints_torques[i];
    low_cmd_.motor_cmd[i+3].kp = control_signal_.kp[i];
    low_cmd_.motor_cmd[i+3].kd = control_signal_.kd[i];
  }

  // Fourth motors state is RL
  for (int i = 9; i < 12; i++) {  
    low_cmd_.motor_cmd[i-3].q = control_signal_.joints_position[i];
    low_cmd_.motor_cmd[i-3].dq = control_signal_.joints_velocity[i];
    low_cmd_.motor_cmd[i-3].tau = control_signal_.joints_torques[i];
    low_cmd_.motor_cmd[i-3].kp = control_signal_.kp[i];
    low_cmd_.motor_cmd[i-3].kd = control_signal_.kd[i];
  }
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
