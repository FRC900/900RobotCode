#!/bin/bash

cd ~/2022RobotCode/zebROS_ws/
echo INCOMPLETE > .native_build.status

if [ -z $ROS_ROOT ]; then
	source /opt/ros/noetic/setup.bash
	if [ ! -z devel/setup.bash ]; then
		source devel/setup.bash
	fi
elif [[ ! $ROS_ROOT = "/opt/ros/noetic/share/ros" ]]; then
	echo "ROS is not configured for a native build (maybe set up for a cross build instead?)"
	echo "Run ./native_build.sh in a new terminal window"
	exit 1
fi
export PATH=$PATH:/usr/local/cuda/bin

EXTRA_BLACKLIST_PACKAGES=""
EXTRA_CMD_LINE=""
uname -a | grep -q x86_64
if [ $? -eq 1 ]; then
	EXTRA_BLACKLIST_PACKAGES="controllers_2020 demo_tf_node robot_characterization robot_visualizer rosbag_scripts rospy_message_converter rqt_driver_station_sim stage_ros template_controller visualize_profile zms_writer"
	EXTRA_CMD_LINE="--limit-status-rate 5"
fi

catkin config --skiplist \
	ar_track_alvar \
	color_spin \
	controllers_2019 \
	controllers_2019_msgs \
	controllers_2020 \
	controllers_2020_msgs \
	realsense2_camera \
	realsense2_description \
	velocity_controllers \
	zed_ros \
	$EXTRA_BLACKLIST_PACKAGES

catkin build -DCATKIN_ENABLE_TESTING=OFF -DBUILD_WITH_OPENMP=ON -DCMAKE_CXX_STANDARD=17 $EXTRA_CMD_LINE "$@"

if [ $? -ne 0 ] ; then
	echo FAIL > .native_build.status
	uname -a | grep -q x86_64
	if [ $? -eq 1 ]; then
		read -n 1 -s -r -p "Press any key to continue"
		echo
	fi
	/bin/false
else
	echo SUCCESS > .native_build.status
	/bin/true
fi

