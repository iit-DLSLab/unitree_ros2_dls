#!/bin/bash
echo "Setup unitree ros2 environment"
CURRENT_DIR="$(pwd)"
echo "Current dir: $CURRENT_DIR"
source $CURRENT_DIR/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
