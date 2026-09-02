## Overwiew
This repo uses [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) from unitree for controlling GO2 and A2 (and probably B2, the hal should be the same as the one of A2), and [z1_sdk](https://github.com/iit-DLSLab/z1_sdk)/[z1_controller](https://github.com/unitreerobotics/z1_controller) for the Z1 arm. 

We add on top of these some new messages to be compatible with different **dls repositories**, like: 

- [basic-locomotion-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-isaaclab) for RL locomotion
- [get-up-isaaclab](https://github.com/iit-DLSLab/get-up-isaaclab) for RL get-up task
- [sim2real-robot-identification](https://github.com/iit-DLSLab/sim2real-robot-identification) for IsaacLab and Mujoco identification
- [Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC) for MPC
- [muse](https://github.com/iit-DLSLab/muse/tree/unitree_sdk) for State Estimation


## Dependencies

**Note that if you are using one of our other repos with conda, you will likely have everything you need installed. In that case, just activate that conda env!**

1. install [miniforge/conda](https://github.com/conda-forge/miniforge/releases) (x86_64 or arm64 depending on your platform)

2. create an environment using the file in the folder [installation](./installation/) by doing:

```bash
conda env create -f installation/mamba_environment_humble.yaml
conda activate unitree_ros2_humble_env
```


## Setup Go2, B2, A2

Follow [README_unitree_ros2](./README_unitree_ros2.md) to install the things needed for GO2, B2 and A2.

## Setup Z1 arm

Follow [README_unitree_z1](./README_unitree_z1.md) to install the things needed for Z1 arm.

## Setup Aliengo

Follow [README_unitree_legged_sdk](./README_unitree_legged_sdk.md) to install the things needed for Aliengo.

## How to contribute

PRs are very welcome (search for **TODO** in the issue, or add what you like)!

## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi).
