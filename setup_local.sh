#!/bin/bash
echo "Setup unitree ros2 local environment"
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#CURRENT_DIR="$(pwd)"
echo "Current dir: $CURRENT_DIR"
source $CURRENT_DIR/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=100 # Set a unique domain ID for your local environment
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'


