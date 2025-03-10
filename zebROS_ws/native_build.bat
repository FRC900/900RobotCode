#!/bin/bash

cd ~/900RobotCode/zebROS_ws/

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

catkin_make -DCATKIN_BLACKLIST_PACKAGES="src/base_trajectory;src/behaviors;src/cmake_modules;src/goal_detection;src/laser_targets;src/navx_publisher;src/path_to_goal;src/pixy_get_lines;src/ros_control_boilerplate;src/rosbag_scripts;src/screen_to_world;src/state_listener;src/talon_swerve_drive_conroller;src/teleop_joystick_control;src/uptime;src/zms_writer"

EXTRA_BLACKLIST_PACKAGES=""
uname -a | grep -q x86_64
if [ $? -eq 1 ]; then
	EXTRA_BLACKLIST_PACKAGES="robot_characterization robot_visualizer rosbag_scripts rospy_message_converter rqt_driver_station_sim visualize_profile zms_writer"
fi

catkin config --skiplist \
	controllers_2019 \
	controllers_2019_msgs \
	controllers_2020 \
	controllers_2020_msgs \
	robot_characterization \
	velocity_controllers \
	zed_ar_track_alvar_example \
	zed_display_rviz \
	zed_depth_sub_tutorial \
	zed_nodelet_example \
	zed_ros \
	zed_rtabmap_example \
	zed_tracking_sub_tutorial \
	zed_video_sub_tutorial \
	$EXTRA_BLACKLIST_PACKAGES

catkin build -DCATKIN_ENABLE_TESTING=OFF -DBUILD_WITH_OPENMP=ON "$@"
