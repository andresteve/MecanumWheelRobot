#!/bin/bash

echo "sourcing ROS files"
source /opt/ros/melodic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "setting ROS master at this IP 192.168.43.67"
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=192.168.43.67
export ROS_IP=192.168.43.67

echo "running roscore node"
screen -dmS roscore roscore
sleep 2



