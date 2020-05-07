#!/bin/bash

cd ~/2020RobotCode/zebROS_ws/
echo INCOMPLETE > .native_build.status

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

EXTRA_BLACKLIST_PACKAGES=""
uname -a | grep -q x86_64
if [ $? -eq 1 ]; then
	EXTRA_BLACKLIST_PACKAGES="robot_characterization robot_visualizer rosbag_scripts rospy_message_converter rqt_driver_station_sim stage_ros visualize_profile zms_writer"
fi

catkin config --blacklist \
	velocity_controllers \
	zed_ros \
	$EXTRA_BLACKLIST_PACKAGES

catkin build -DCATKIN_ENABLE_TESTING=OFF -DBUILD_WITH_OPENMP=ON "$@"

if [ $? -ne 0 ] ; then
	echo FAIL > .native_build.status
	/bin/false
else
	echo SUCCESS > .native_build.status
fi
