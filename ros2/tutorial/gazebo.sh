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

# if gazebo acts weird, use this to kill it and launch it again
sudo killall -9 gazebo gzserver gzclient

# With everything we have done until now, it will show a static robot. But it is a good step to verify that everything works fine

# CONTROLS
# now we want to control the robot
# we need to edit the urdf file for adding controls. check the urdf file with comments added to check where we add the controls
