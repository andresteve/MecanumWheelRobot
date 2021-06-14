#!/bin/bash

echo "sourcing ROS files"
source /opt/ros/melodic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "setting ROS master at this IP 192.168.43.67"
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=192.168.43.67
export ROS_IP=192.168.43.67

echo "running rosserial node"
screen -dmS rosserial rosrun rosserial_python serial_node.py _port:=/dev/serial0 _baud:=250000
sleep 3
echo "running ydlidar node"
screen -dmS ydlidar roslaunch ydlidar_ros lidar.launch
sleep 3
echo "running slam node"
screen -dmS slam roslaunch hector_slam_launch tutorial.launch


echo "done. view with screen -ls"
echo "	    display node with screen -r <name>"
echo "	    kill all with pkill screen"


