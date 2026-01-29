## Install Unitree ROS2 package


1. Compile unitree packages packages
First, compile unitree stuff:

```bash
git submodule update --init --recursive
cd unitree_ros2/cyclonedds_ws
colcon build # Compile all packages in the workspace
source install/setup.bash
```

If you have any problem complaining **rosidl_generate_interfaces** (likely), search globally from **cmake_minimum_required(VERSION** and bump each version to 3.15


## Network configuration

1. Connect Unitree robot and the computer using Ethernet cable. Then, use ifconfig to view the network interface that the robot connected. 

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


2. After completing the above configuration, it is recommended to restart the computer before conducting the test.

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


## Running the hal

The source code of the hal is located at `/ros2_ws/src/quadruped_hal`.

Open a terminal and input:
```bash
source setup.bash
cd ros2_ws
colcon build 
```
After compilation, run in the terminal:
```bash
source /install/setup.bash
./install/quadruped_hal/bin/quadruped_hal 
```

You can even use
```bash
python3 launch_quadruped_hal.py
```

