#!/bin/bash

if [ -z $ROS_ROOT ]; then
	source /usr/arm-frc-linux-gnueabi/opt/ros/kinetic/setup.bash
elif [[ ! $ROS_ROOT = "/usr/arm-frc-linux-gnueabi/opt/ros/kinetic/share/ros" ]]; then
	echo "ROS is not configured for a cross build (maybe set up for a native build instead?)"
	echo "Run ./cross_build.sh in a new terminal window"
	exit 0
fi

catkin_make_isolated --install --use-ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`pwd`/rostoolchain.cmake -DCATKIN_ENABLE_TESTING=OFF "$@"
