import sys
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path+"/../")

import time

os.system("bash -c 'cd unitree_z1/z1_controller/build && ./z1_ctrl'")

