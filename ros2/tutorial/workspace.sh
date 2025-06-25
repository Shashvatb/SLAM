#!/bin/bash

# we want to arrange our workspace
# organize folder structure: packages inside src 
mkdir ros2_ws
mkdir ros2_ws/src

# our build tool is colcon
sudo apt install python3-colcon-common-extensions
cd ros2_ws/src

# instead of reinstalling ros 2, we can just create a link to it to save space
cd ..
colcon build --symlink-install

# source the workspace
source install/setup.bash
# add this to make sure we dont have to do is again and again
echo "source /mnt/c/job/repos/SLAM/ros2/ros2_ws/install/setup.bash" >> ~/.bashrc

# check if the new sourcing worked
ros2 run turtlesim turtlesim_node
# we can edit turtle_frame.cpp to verify we have a new copy of it

# create package
cd src 
ros2 pkg create --build-type ament_python --node-name my_node my_pkg

# build the package
cd ..
colcon build --packages-select my_pkg
source install/local_setup.bash

# check if node runs
ros2 run my_pkg my_node