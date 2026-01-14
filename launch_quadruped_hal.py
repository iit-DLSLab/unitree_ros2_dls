import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path+"/../")

import time

submodule_path = dir_path + "/ros2_ws/install"
print("submodule_path:", submodule_path)
if not os.path.exists(submodule_path):
    print("Build the HAL by hand first!!")
else:
    print("\n\n")
    print("HAL already built - if you have any modifications, please delete the build folder in the submodule")
    print("\n\n")
    time.sleep(2)
    os.system("bash -c 'source setup.bash && source ros2_ws/install/setup.bash && ./ros2_ws/install/quadruped_hal/bin/quadruped_hal'")

