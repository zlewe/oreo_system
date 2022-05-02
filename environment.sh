#! /bin/bash
#export ROS_HOSTNAME=oreo-desktop
export ROS_MASTER_URI=http://192.168.0.223:11311
export ROS_IP=192.168.0.223

source $HOME/oreo_system/motor.sh
source $HOME/oreo_system/catkin_ws/devel/setup.bash
