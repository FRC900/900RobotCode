#!/bin/bash

cd ~/2019Offseason/zebROS_ws/

if [ -z $ROS_ROOT ]; then
	PATH=$PATH:$HOME/frc2019/roborio/bin
	source ~/frc2019/roborio/arm-frc2019-linux-gnueabi/opt/ros/melodic/setup.bash
elif [[ ! $ROS_ROOT = "$HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/opt/ros/melodic/share/ros" ]]; then
	echo "ROS is not configured for a cross build (maybe set up for a native build instead?)"
	echo "Run ./cross_build.sh in a new terminal window"
	exit 1
fi

catkin_make_isolated --install --use-ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`pwd`/rostoolchain.cmake -DCATKIN_ENABLE_TESTING=OFF "$@"
