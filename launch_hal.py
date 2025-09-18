import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path+"/../")

import time

submodule_path = dir_path + "/example/install"
print("submodule_path:", submodule_path)
if not os.path.exists(submodule_path):
    print("Build the HAL by hand first!!")
else:
    print("\n\n")
    print("HAL already built - if you have any modifications, please delete the build folder in the submodule")
    print("\n\n")
    time.sleep(2)
    os.system(" \
    cd /example && \
    colcon build && \
    source install/setup.bash && \
    ./install/unitree_ros2_example/bin/robot_hal")

