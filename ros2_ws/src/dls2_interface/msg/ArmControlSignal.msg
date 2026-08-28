# Header
string frame_id
uint32 sequence_id
float64 timestamp

uint8 control_mode;

float64[] arm_joints_position;
float64[] arm_joints_velocity;
float64[] arm_joints_torques;
float64[] arm_kp;
float64[] arm_kd;

float64[] gripper_joints_position;
float64[] gripper_joints_velocity;
float64[] gripper_joints_torques;
float64[] gripper_kp;
float64[] gripper_kd;