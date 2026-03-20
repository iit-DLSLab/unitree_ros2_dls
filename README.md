# Introduction
This repo uses [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) from unitree for controlling GO2, B2, Aliengo, and [z1_sdk](https://github.com/iit-DLSLab/z1_sdk)/[z1_controller](https://github.com/unitreerobotics/z1_controller) for the Z1 arm. 

We add on top of these some new messages and a complete robot_hal script to talk with the robots in **ROS2**, compatible with some **dls controllers and state estimators**, like [basic-locomotion-dls-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-dls-isaaclab), and soon [Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC) and [muse](https://github.com/iit-DLSLab/muse).



## Dependencies
The main dependencies other ros-humble are
```bash
ros-humble-rmw-cyclonedds-cpp
ros-humble-rosidl-generator-dds-idl
```

**Note that if you are using one of our other repos with conda, you will likely have them installed. In that case, just activate that conda env!**

Otherwise, an example of conda environment containing these dependencies can be found in the folder installation, which can be installed by doing

```bash
conda env create -f installation/mamba_environment_humble.yaml
conda activate unitree_ros2_humble_env
```

Be sure to have these before proceeding. 

## Unitree ROS2 package

Follow [README_unitree_ros2](https://github.com/iit-DLSLab/unitree_ros2_dls/blob/main/README_unitree_ros2.md) to install the thing needed for GO2, B2, A2.


## Z1 sdk/controller packages

Follow [README_unitree_z1](https://github.com/iit-DLSLab/unitree_ros2_dls/blob/main/README_unitree_z1.md) to install the thing needed for Z1.
