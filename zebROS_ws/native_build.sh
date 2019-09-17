#!/bin/bash

cd ~/2019Offseason/zebROS_ws/

if [ -z $ROS_ROOT ]; then
	source /opt/ros/melodic/setup.bash
	if [ ! -z devel/setup.bash ]; then
		source devel/setup.bash
	fi
elif [[ ! $ROS_ROOT = "/opt/ros/melodic/share/ros" ]]; then
	echo "ROS is not configured for a native build (maybe set up for a cross build instead?)"
	echo "Run ./native_build.sh in a new terminal window"
	exit 1
fi

catkin_make --use-ninja -DCATKIN_ENABLE_TESTING=OFF "$@"
