#!/bin/bash

# if we have a lot of nodes we want to start, we can a launch file
# we can have a launch file that calls different kinds of files in the same launch file
# we want to place the launch file inside a new directory called launch
cd /mnt/c/job/repos/SLAM/ros2/ros2_ws/src/pub_sub
mkdir launch
# launch file is placed in launch/simple.launch.py

# now we have to add more lines in setup.py (check comments in the file)

# build the new package
colcon build --packages-select pub_sub
source install/local_setup.bash

# run the launch file
ros2 launch pub_sub simple.launch.py