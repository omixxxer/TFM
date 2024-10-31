# Person-following Python ROS 2 template
-----------------------------------------------------

---------------------------------------------
TURTLE BOT 2
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
    The laptop must have ROS 2 Humble installed

Connection with ssh:

Open a terminal in the laptop and execute the command:

ssh user@192.168.0.224

The password is "qwerty".

Each robot has a different IP address 192.168.0.XXX ending in 224, 225, 226 or 227 (see label on top of the NUC).

Run the ROS Kobuki node:

In an ssh terminal execute the commands:

    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27 según el robot
    ros2 launch kobuki_node kobuki_node-launch.py


Teleoperate the TurtleBot 2 from a laptop terminal:

In a laptop terminal execute the commands:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/commands/velocity
    
Run the ROS Lidar node:

In an ssh terminal execute the commands:

    source /opt/ros/humble/setup.bash
    source ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar

The Lidar should start to turn. In another ssh terminal execute the commands:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 run tf2_ros static_transform_publisher 0 0 0 3.141592 0 0 base_footprint laser

Check  the /scan topic from a laptop terminal:

In a laptop terminal execute the commands:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    ros2 topic hz /scan

The displayed average rate should be around 10.9 Hz.


Run the ROS Camera node:

In an ssh terminal execute the commands:

    source /opt/ros/humble/setup.bash
    ros2 run usb_cam usb_cam_node_exe --ros-args -p image_width:=320 -p image_height:=240 -p framerate:=30.0


Check  the /scan topic from a laptop terminal:

In a laptop terminal execute the commands:  

    source /opt/ros/humble/setup.bash
    ros2 topic hz /image_raw

The displayed average rate should be around 20-30 Hz.


Create a ROS 2 workspace

    mkdir -p ~/ros2_ws/src

Clone this repository and build the package

    cd ~/ros2_ws/src
    git clone https://github.com/omixxxer/TFM.git .
    cd ..
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install


Visualize the pose of the robot and the Lidar scan from a laptop terminal:

Execute the commands in a laptop terminal:

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    rviz2 -d ros2_ws/rviz/tb2.rviz
    
Visualize the camera footage, we can do it with the Rviz2 or with a opencv window:

RVIZ
---
Execute the commands in a laptop terminal   

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=24  # o 25, 26, 27
    rviz2 -d ros2_ws/rviz/camera.rviz

LAPTOP TERMINAL
-----
Visualize the camera footage in real time from a laptop terminal:

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run person_follower camera_viewer.py


Run the person-following node

    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=24 # or 25,26,27
    ros2 launch person_follower start_person_follower.launch.py

