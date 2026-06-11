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

1. Connect Unitree robot and the computer using an Ethernet cable. Then, use the command **ifconfig** to view the network interface that the robot connected to **(remember its name for step 4!!!!)**.
```bash
ifconfig
```

3. Next, open the network settings, find the network interface that the robot is connected. In IPv4 setting, **change the IPv4 mode to manual, set the address to 192.168.123.99, and set the mask to 255.255.255.0**. After completion, click apply and wait for the network to reconnect.

4. Then open the file unitree_ros2_connect
```bash
sudo gedit unitree_ros2_connect.bash
```
```bash

4. ...and write inside your interface name found in step 1 instead of **enp3s0**
```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```



5. Ensure that the network of robot is connected correctly, open a terminal and input:  
```bash
source unitree_ros2_connect.bash
ros2 topic list
```


6. **Note that in every terminal where you launch your other scripts, you need to source this file first. Put it in .bashrc as an alias!** 
```bash
gedit .bashrc
alias unitree_ros2_connect='source unitree_ros2_connect.bash'
Restart the terminal!!!!
```


## Running the quadruped hal

The source code of the hal is located at `/ros2_ws/src/quadruped_hal`.

Open a terminal and input:
```bash
unitree_ros2_connect
```
**Note that in every terminal where you launch your other scripts, you need to source this file first. Put it in .bashrc as an alias!**

After compilation, run in the terminal:
```bash
python3 launch_quadruped_hal.py 
```


