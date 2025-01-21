#!/bin/bash

export WEBOTS_HOME=$HOME/webots_R2022b
source /opt/ros/foxy/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch webots_ros2_turtlebot robot_launch.py \
  world:=turtlebot3_burger_pedestrian_simple.wbt
