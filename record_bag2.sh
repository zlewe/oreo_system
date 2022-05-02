#! /bin/bash

CPATH="$HOME/oreo_system/bags/$(date +%m%d_%H%M)" 

mkdir $CPATH 
rosbag record -o $CPATH/oreo --split --size=1024 \
/tf \
/tf_static \
/velodyne_points \
/odometry_odm \
/temp_cmd_vel \
/scan3
