#!/usr/bin/env bash

# Setup ROS for secondary jetsons - might be ok to use master if ROS_IP shell magic is good
source /home/ubuntu/900RobotCode/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.9.0.8:5802
export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
export ROSLAUNCH_SSH_UNKNOWN=1
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/ctre/linux/arm64/shared:/usr/local/lib
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages
echo "ROS_IP set to $ROS_IP"

exec "$@"
