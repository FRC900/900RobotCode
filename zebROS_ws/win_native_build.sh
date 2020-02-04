#!/bin/bash

cd ~/2020RobotCode/zebROS_ws/

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
	EXTRA_BLACKLIST_PACKAGES="robot_characterization robot_visualizer rosbag_scripts rospy_message_converter rqt_driver_station_sim visualize_profile zms_writer"
fi

catkin config --blacklist \
	base_trajectory \
	behaviors \
	cmake_modules \
	goal_detection \
	laser_targets \
	navx_publisher \
	path_to_goal \
	pixy_get_lines \
	ros_control_boilerplate \
	rosbag_scripts \
	screen_to_world \
	state_listener \
	talon_swerve_drive_controller \
	teleop_joystick_control \
	zms_writer \
	velocity_controllers \
	zed_ar_track_alvar_example \
	zed_display_rviz \
	zed_depth_sub_tutorial \
	zed_nodelet_example \
	zed_ros \
	zed_rtabmap_example \
	zed_tracking_sub_tutorial \
	zed_video_sub_tutorial \
	teraranger \
	teraranger_array \
	realsense2_camera \
	robot_characterization \
	$EXTRA_BLACKLIST_PACKAGES

catkin build -DCATKIN_ENABLE_TESTING=OFF -DBUILD_WITH_OPENMP=ON "$@"
