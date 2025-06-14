#!/bin/bash

# install turtle sim
sudo apt update
sudo apt install ros-humble-turtlesim

# check if turtle sim present
ros2 pkg list

# list the executables (we can check what executables are present in turtlesim)
ros2 pkg executables turtlesim

# we will need multiple terminals to run ros executables since they are multiple nodes. We can use tmux (terminal multiplexer) to have them in the same window
tmux new -s ros
# we can use ctrl+b > " and ctrl+b > % to make multiple terminals
# we can switch between the terminals with ctrl+b > arrow key 

# run turtle sim
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# check running nodes
ros2 node list

# get info about specific running node
ros2 node info /turtlesim

# TOPICS
# we see publishers and subscribers listed here
# if the way of communication is topics, a publisher will broadcast the information and the subscribers can read the information from it
# list out all the active topics with type
ros2 topic list -t 

# view information on a topic 
ros2 topic info /turtle1/cmd_vel

# to visualize the communication we can use the following built-in functionality
rqt_graph

# if we want to read the output of a specific topic, we can use the following command
ros2 topic echo /turtle1/cmd_vel
# we can understand the output and see what values it is displaying with the help of the type of the topic mentioned
ros2 interface show geometry_msgs/msg/Twist

# we can publish to the topic -> frequency, topic name, message type, payload (make sure there is a space between ":" and key value)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 2.0, z: 3.0}, angular: {x: 1.0, y: 0.0, z: 0.0}}"
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 2.0, z: 3.0}, angular: {x: 1.0, y: 0.0, z: 0.0}}"

# check the frequency of a topic
ros2 topic hz /turtle1/cmd_vel

# SERVICE
# in this kind of communication we have a client-server relationship
# list all services with type
ros2 service list -t

# check the type of service
ros2 service type /clear
# find services of specific types
ros2 service find std_srvs/srv/Empty
# we can understand the output and see what values it is displaying with the help of the type of the service mentioned
ros2 interface show turtlesim/srv/Spawn

# Call a service -> service name, service type
ros2 service call /clear std_srvs/srv/Empty

# ACTIONS 
# also uses server-client relationship
# it has goals, feedback and results which are also part of the communication
# the results can be seen on the server node
# get the list of actions with type
ros2 action list -t 
# get information about the actions
ros2 action info /turtle1/rotate_absolute
#get data type
ros2 interface show turtlesim/action/RotateAbsolute
# send action goals -> action name, action type, goal
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"



# PARAMETERS
# similar to passing args for a function but for specific node or service
# get the list of parameters
ros2 param list
# get params for a specific node
ros2 param dump /turtlesim
# save the params as a yaml file
ros2 param dump /turtlesim > turtlesim_param.yaml

# get param value -> node, param
ros2 param get /turtlesim background_g
# set param value -> node, param, value
ros2 param set /turtlesim background_g 255

# load params from a yaml file for a specific node -> node, yaml file
ros2 param load /turtlesim turtlesim_param.yaml

# we can load up the params at startup 
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_param.yaml