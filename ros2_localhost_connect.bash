#!/bin/bash
echo "Setup unitree ros2 local environment"
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#CURRENT_DIR="$(pwd)"
echo "Current dir: $CURRENT_DIR"
source $CURRENT_DIR/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=100 # Set a unique domain ID for your local environment
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

# Determine script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR/ros2_ws"
if [ -f "$ROS2_WS/install/setup.bash" ]; then
	source "$ROS2_WS/install/setup.bash"
	echo "Sourced dls2_interface ROS2 workspace: $ROS2_WS"
else
	echo "Warning: $ROS2_WS/install/setup.bash not found."
    
	# Build the workspace (assume the workspace directory exists and colcon is available)
	echo "Building ROS2 workspace with colcon in: $ROS2_WS"
	(cd "$ROS2_WS" && colcon build) || {
		echo "colcon build failed."
		return 1
	}
	if [ -f "$ROS2_WS/install/setup.bash" ]; then
		source "$ROS2_WS/install/setup.bash"
		echo "Sourced ROS2 workspace after build: $ROS2_WS"
	else
		echo "Build finished but $ROS2_WS/install/setup.bash still not found."
		return 1
	fi
fi