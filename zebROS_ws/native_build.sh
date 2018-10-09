#!/bin/bash

if [ -z $ROS_ROOT ]; then
	source /opt/ros/kinetic/setup.bash
	if [ ! -z install_native/setup.bash]; then
		source install_native/setup.bash
	fi
elif [[ ! $ROS_ROOT = "/opt/ros/kinetic/share/ros" ]]; then
	echo "ROS is not configured for a native build (maybe set up for a cross build instead?)"
	echo "Run ./native_build.sh in a new terminal window"
	exit 0
fi


catkin_make_isolated --install --use-ninja --build-space build_native --devel-space devel_native --install-space install_native -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF "$@"
