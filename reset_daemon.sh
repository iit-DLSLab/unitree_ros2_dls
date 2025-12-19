#!/bin/bash
# For some reason, the z1 topic needs us to reset the ros2 daemon to be visible
echo "Remeber to run this file with: source reset_daemon.sh"
ros2 daemon stop &&
ros2 daemon start