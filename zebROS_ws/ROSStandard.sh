#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/melodic/setup.bash
source ~/2020RobotCode/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
