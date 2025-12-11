# Introduction
This repo uses [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) from unitree. It's just adding some new messages and a complete robot_hal script to talk with the robot, compatible with some dls controllers.



## Install Unitree ROS2 package

### 1. Dependencies
The main dependencies other ros-humble are
```bash
ros-humble-rmw-cyclonedds-cpp
ros-humble-rosidl-generator-dds-idl
```
An example of conda environment containing these dependencies can be found in the folder installation, which can be used by doing

```bash
conda env create -f mamba_environment.yml
conda activate unitree_ros2_humble_env
```

### 2. Compile unitree packages packages
First, compile unitree stuff:

```bash
git submodule update --init --recursive
cd unitree_ros2/cyclonedds_ws
colcon build # Compile all packages in the workspace
source install/setup.bash
```

## Connect to Unitree robot

### 1. Network configuration
Connect Unitree robot and the computer using Ethernet cable. Then, use ifconfig to view the network interface that the robot connected. 

Next, open the network settings, find the network interface that the robot connected. In IPv4 setting, **change the IPv4 mode to manual, set the address to 192.168.123.99, and set the mask to 255.255.255.0**. After completion, click apply and wait for the network to reconnect.

Open setup.sh file.
```bash
sudo gedit setup.bash
```
```bash
#!/bin/bash
.......
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```
where "enp3s0" is the network interface name of unitree robot connected.
Modify it to the corresponding network interface according to the actual situation. 

Source the environment to setup the ROS2 support of Unitree robot: 
```bash
source setup.bash
```

If your computer is not connected to the robot but you still want to use Unitree ROS2 for simulation and other functions, you can use the local loopback "lo" as the network interface.
```bash
source setup_local.bash # use "lo" as the network interface
```


### 2. Connect and test
After completing the above configuration, it is recommended to restart the computer before conducting the test.

Ensure that the network of robot is connected correctly, open a terminal and input:  
```bash
source setup.bash
ros2 topic list
```


If you have any problem in seeing, listening the topic, maybe it's the firewall.

The firewall gives us problem!!
```bash
sudo ufw disable
```

or

```bash
sudo ufw allow from 192.168.123.18 to any port 7400:65535 proto udp
sudo ufw allow from 192.168.123.161 to any port 7400:65535 proto udp
```


### 3. Running the hal

The source code of the hal is located at `/hal/src/hal`.

Open a terminal and input:
```bash
source setup.bash
cd hal
colcon build 
```
After compilation, run in the terminal:
```bash
source /install/setup.bash
./install/hal/bin/robot_hal 
```

You can even use
```bash
python3 launch_hal.py
```

