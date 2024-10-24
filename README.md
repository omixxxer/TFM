# Person-following Python ROS 2 template
-----------------------------------------------------

---------------------------------------------
TURTLE BOT
---------------------------------------------
Power on

    Turn on the switch on the Kobuki base
    Turn on the external battery pack
    Set the voltage of the battery pack to 20V
    Turn on the NUC
    Plug in the Kobuki USB cable to the NUC (first)
    Plug in the Lidar USB cable to the NUC (second)

Power off

    Unplug the USB cables of the Kobuki and the Lidar
    Turn off the NUC (the battery pack will go off automatically)
    Turn off the switch on the Kobuki base
    Put the Kobuki on a charging station
    If necessary, charge the external battery pack

Connection between a laptop and the TurtleBot 2

Prerequisites:

    The laptop must be connected to the WiFi PIROBOTNET6 or PIROBOTNET6_5G
    The laptop must have ROS 2 Foxy installed

Connection with ssh:

Open a terminal in the laptop and execute the command:

ssh user@192.168.0.224

The password is "qwerty".

Each robot has a different IP address 192.168.0.XXX ending in 224, 225, 226 or 227 (see label on top of the NUC).

Run the ROS Kobuki node:

In an ssh terminal execute the commands:

    source /opt/ros/foxy/setup.bash
    source ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24
    ros2 launch kobuki_node kobuki_node-launch.py 

*Depending on the ROS distribution could be foxy or humble

Use these numbers for each robot:
ROS_DOMAIN_ID 	IP address
24 	192.168.0.224
25 	192.168.0.225
26 	192.168.0.226
27 	192.168.0.227

Teleoperate the TurtleBot 2 from a laptop terminal:

In a laptop terminal execute the commands:

    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=24 # or 25,26,27
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity

Run the ROS Lidar node:

In an ssh terminal execute the commands:

    source /opt/ros/foxy/setup.bash
    source ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24 # or 25,26,27
    ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar

The Lidar should start to turn. In another ssh terminal execute the commands:

    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=24 # or 25,26,27
    ros2 run tf2_ros static_transform_publisher 0 0 0 3.141592 0 0 base_footprint laser

Check  the /scan topic from a laptop terminal:

In a laptop terminal execute the commands:

    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=24 # or 25,26,27
    ros2 topic hz /scan

The displayed average rate should be around 10.9 Hz.

Visualize the pose of the robot and the Lidar scan from a laptop terminal:

Download the tb2.rviz configuration file and execute the commands in a laptop terminal:

    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=24 # or 25,26,27
    rviz2 -d tb2.rviz

Create a ROS 2 workspace
```
    mkdir -p ~/ros2_ws/src
```

Clone this repository and build the package
```
    cd ~/ros2_ws/src
    git clone https://github.com/omixxxer/TFM.git
    cd ..
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install
```

```
Run the person-following node
```
    source /opt/ros/foxy/setup.bash
    source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24 # or 25,26,27
    ros2 run person_follower person_follower 


---------------------------------------------
SIM
---------------------------------------------

We assume that [ROS 2](https://docs.ros.org/) and [Webots](https://cyberbotics.com/) are installed in the system. 

For the steps below we use ROS2 Foxy and Webots R2022b.

1. Install the prerequisites
```
sudo apt install ros-foxy-webots-ros2-turtlebot
```
2. Create a ROS 2 workspace
```
mkdir -p ~/ros2_ws/src
```
3. Clone this repository and build the package
```
cd ~/ros2_ws/src
git clone https://github.com/RobInLabUJI/person_follower.git
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```
4. Copy the Webots world file to the ROS package folder
```
sudo cp ~/ros2_ws/src/person_follower/webots/*.wbt \
        /opt/ros/foxy/share/webots_ros2_turtlebot/worlds/.
```
5. Run the person-following node
```
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 run person_follower person_follower 
```
6. In a new terminal, launch the Webots simulator

In a room with walls:
```
export WEBOTS_HOME=~/webots-R2022b
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch webots_ros2_turtlebot robot_launch.py \
  world:=turtlebot3_burger_pedestrian_simple.wbt
```

Or a room without walls:
```
export WEBOTS_HOME=~/webots-R2022b
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch webots_ros2_turtlebot robot_launch.py \
  world:=turtlebot3_burger_pedestrian_no_walls.wbt
```

7. In a new terminal, launch RViz
```
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1
rviz2 -d ~/ros2_ws/src/person_follower/webots/config.rviz
```
