#!/usr/bin/env bash
#PI3A-ubuntu-18.04.2-armhf
source /opt/ros/melodic/setup.bash
source /home/ubuntu/Robots/Ekarst/catkin_ws/devel/setup.bash
export PI=192.168.1.23
export ROS_MASTER_URI=http://$PI:11311
export ROS_IP=$PI
export ROS_HOSTNAME=$PI

