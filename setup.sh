#!/bin/bash
echo "Setup unitree ros2 environment"
CURRENT_DIR="$(pwd)"
echo "Current dir: $CURRENT_DIR"
source $CURRENT_DIR/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enx6c1ff71cdfe2" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
