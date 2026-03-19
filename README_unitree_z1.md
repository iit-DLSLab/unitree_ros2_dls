## Install Z1 sdk and controller packages

First, initialise the submodules (required before any cmake step):

```bash
git submodule update --init --recursive
```

1. Go inside `unitree_z1/z1_controller` and build:

```bash
cd unitree_z1/z1_controller
mkdir build && cd build
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
make -j4
```

2. Go inside `unitree_z1/z1_sdk` and build:

```bash
cd ../../z1_sdk
mkdir build && cd build
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
make -j4
```

3. Build the ROS2 messages from the repo root:

```bash
cd ../../ros2_ws
colcon build --packages-select dls2_interface
source install/setup.bash
```

## Running the Z1 HAL

**(TERMINAL 1)** From the repo root, start the Z1 controller:

```bash
python3 launch_z1_controller.py
```

**(TERMINAL 2)** From the repo root, start the HAL node:

```bash
python3 launch_z1_hal.py
```