#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/noetic/setup.bash
source ~/900RobotCode/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/ctre/linux/arm64/shared:/home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/rev/linux/arm64/shared:/home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/rev/linux/x86-64/shared:/usr/local/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.10/dist-packages/nvidia/cublas/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.10/dist-packages/nvidia/cufft/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.10/dist-packages/nvidia/cusparse/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/python3.10/dist-packages/nvidia/cusolver/lib
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages

if [ $ROS_IP == "10.9.0.9" ] ; then
	export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
#        echo "Resetting time on secondary Jetson"
#        echo ubuntu | sudo -S systemctl stop ntp.service
#        echo ubuntu | sudo -S ntpd -gqx
#        echo ubuntu | sudo -S systemctl restart ntp.service
fi
exec "$@"
