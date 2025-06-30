#!/bin/bash

cd ros2/ros2_ws/src/
ros2 pkg create --build-type ament_python pub_sub

# Create a publisher and subscriber python files (see publisher.py and subscriber.py)
# update package.xml to add dependencies used in our publisher-subscriber (see package.xml for the changes)
# update setup.py to add entry points (see setup.py)

# check for missing dependencies
cd ..
apt install python3-rosdep2
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build the new package
colcon build --packages-select pub_sub
source install/local_setup.bash

# run packages (the names should be consistent with setup.py)
ros2 run pub_sub minimal_publisher 
ros2 run pub_sub minimal_subscriber
