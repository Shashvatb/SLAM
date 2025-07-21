#!/bin/bash

## URDF - Unified Robotics Description Format
# It is the description of the robot (real or virtual)

# check simple_robot.xml file for example. dont forget to add it to setup.py
# also install joint state publisher
sudo apt install ros-humble-joint-state-publisher


# URDF FILE STRUCTURE
# starts with a header for xml version
# then a robot name. It has two sections: link and joint
# link has visual, collision and inertial properties
# join has axis, parent link, child link and origin

# LINKS - A segment of a robot

# visual (optional) has geometry (required), origin and material properties - can be accurate (3D models:stl or dae file) or a simplified representation
# collision (optional) has geometry (required), and origin
# intertial (optional) has mass and inertia

# geometry can be used to describe a box, sphere cylinder, mesh etc
# origin has rpy (roll, pitch, yaw) and xyz values relative to the link's coordinate frame(visual and collision).
# For intertial, the origin is the centre of mass relative to link's frame
# rotation applied first and then the translation
# axis color RGB(XYZ)
# material describes the color and transperancy of the link (rgba)

# collision describes the bounding shape around the link for collision detection (usually simplified shape)

# inertia describes mass (in kgs) and inertia ixx, ixy, ixz
#                                                  iyy, iyz
#                                                       izz


# JOINTS - connection between links
# name and type of joint are described first
# child and parent link names are optional
# origin, axis, limit (depends on type of joint) and dynamics (damping, friction etc) are required 
# types of join - fixed (no motion), continuous (unrestricted rotation about an axis), revolute (limited rotation about an axis)
# prismatic (translation about axis), floating (6 DOF 3 R and 3 T), planer (motion on plate - 1 R 2 T)

# parent and child link names will just be the names
# origin describes the child's frame relative to the parent frame (rpy, xyz)
# axis describes axis of rotation for the joint (rpy, xyz)
# dynamics describes damping (N s/m) and static friction (N)
# limit are for revolute and prismatic joints only. includes lower (rad), upper (rad), effort (Nm), velocity (rad/s)
colcon build --packages-select simple_robot
source install/local_setup.bash

# run the state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(< ros2/ros2_ws/src/simple_robot/urdf/simple_robot.xml)"

# check if the publisher works
ros2 run tf2_tools view_frames

# run joint state publisher. this shows us parameters for the links
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# run rviz. 
# make sure the correct frame is selected in the global settings on the left panel
# make sure to add -> RobotModel
# Make sure to select the right robot description
rviz2

# recap: we have set the fixed frame to "world", added RobotModle and selected the description as /robot_description
# now we will save this rviz config with file -> save config as. I am saving it inside my package under a new folder called rviz
# now we want to run all of this with a launch file (simple_robot.launch.py)
# lets rebuild the package (we might not need it, but just to be safe)
# dont forget to add the config file in your setup.py
colcon build --packages-select simple_robot
source install/local_setup.bash
ros2 launch simple_robot simple_robot.launch.py

## XACRO - we can make the URDF file better by calling in fuctions inside it
# check simple_robot_xacro.urdf.xacro file for example
# Convert xacro file to a urdf file
xacro model.xacro > model.urdf
# Robot state Publisher - can add this to the launch file. [check simple_robot.launch.py]

# xacro properties - kind of like variables in xacro to be used throughout the file
# <xacro:property name="property_name" value="..." />
# USAGE <... variable="${property_name}" ... />

# xacro macro - replace several lines of code
# <xacro:macro name="macro_name" params="param1 param2 ...">
#   ...
# </xacro:macro
# USAGE <xacro:macro_name param1=""/>

#launch and check the file\
sudo apt install ros-humble-xacro
ros2 launch simple_robot simple_robot_xacro.launch.py
