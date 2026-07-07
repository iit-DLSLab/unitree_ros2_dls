#!/bin/bash
echo "Setup unitree ros2 environment: Remember to change inside this file the NetworkInterface"
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#CURRENT_DIR="$(pwd)"
echo "Current dir: $CURRENT_DIR"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0 # Set a unique domain ID for your local environment
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enx6c1ff71cdfe2" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
