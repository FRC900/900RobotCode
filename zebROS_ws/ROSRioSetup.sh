#!/usr/bin/env bash

# Setup Rio ROS Environment
source /opt/ros/kinetic/setup.bash
source ~/2018RobotCode/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.9.0.29:11311
export ROS_IP=$(hostname -I)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
