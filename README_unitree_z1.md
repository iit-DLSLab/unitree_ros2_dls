## Install Z1 sdk and controller packages

1. Go inside unitree_z1/z1_controller and press:

```bash
mkdir build
cd build
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
make -j4
```


2. Go inside unitree_z1/z1_sdk and press:

```bash
mkdir build
cd build
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
make -j4
```

3. Go inside ros2_ws
```bash
colcon build --packages-select dls2_interface
source install/setup.bash
```

## RUN

