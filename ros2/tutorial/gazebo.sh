#!/bin/bash

# install gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs

# create new package
cd ros2/ros2_ws/src/
ros2 pkg create --build-type ament_python gazebo_robot

# add the urdf file, launch file, config file
# update setup.py
# update package.xml

# lauch file - launch gazebo, robot state publisher, spawn robot, joint state broadcaster, joint trajectory controller
# check file launch/gazebo_robot_xacro.launch.py

# launch all nodes
colcon build --packages-select gazebo_robot
source install/local_setup.bash
ros2 launch gazebo_robot gazebo_robot_xacro.launch.py



