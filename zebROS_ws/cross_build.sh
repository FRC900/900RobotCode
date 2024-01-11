#!/bin/bash

cd ~/2023RobotCode/zebROS_ws/

if [ -z $ROS_ROOT ]; then
	#PATH=$PATH:$HOME/wpilib/2024/roborio/bin
	source ~/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/opt/ros/noetic/setup.bash
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
elif [[ ! $ROS_ROOT = "$HOME/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/opt/ros/noetic/share/ros" ]]; then
	echo -e "\e[1m\e[31mROS is not configured for a cross build (maybe set up for a native build instead?)\e[0m"
	echo -e "\e[1m\e[31mRun ./cross_build.sh in a new terminal window\e[0m"
	exit 1
fi

catkin config --profile cross -x _isolated --install --skiplist \
	ackermann_steering_controller \
	adi_driver \
	adi_pico_driver \
	ar_track_alvar \
	apriltag_launch \
    apriltag_ros \
	base_trajectory \
	color_spin \
	controllers_2019 \
	controllers_2019_msgs \
	controllers_2020 \
	controllers_2020_msgs \
	cuda_apriltag_ros \
	deeptag_ros \
	demo_tf_node \
	diff_drive_controller \
	effort_controllers \
	fake_sensors \
	field \
	force_torque_sensor_controller \
	four_bar_elevator_2023 \
	four_wheel_steering_controller \
	frcrobot_description \
	frcrobot_gazebo \
	gazebo_frcrobot_control \
	goal_detection \
	navx_publisher \
	gripper_action_controller \
	pf_localization \
	realsense2_camera \
	realsense2_description \
	robot_characterization \
	robot_visualizer \
	rosbag_scripts \
	rospy_message_converter \
	rosserial_arduino \
	rosserial_chibios \
	rosserial_embeddedlinux \
	rosserial_mbed \
	rosserial_server \
	rosserial_test \
	rosserial_tivac \
	rosserial_vex_cortex \
	rosserial_vex_v5 \
	rosserial_windows \
	rosserial_xbee \
	rqt_driver_station_sim \
	rqt_joint_trajectory_controller \
	stage_ros \
	template_controller \
	teraranger_array \
	teraranger_array_converter \
	tf_object_detection \
	turing_smart_screen \
	velocity_controllers \
	visualize_profile \
	zed_nodelets \
	zed_ros \
	zed_wrapper \
	zms_writer
catkin build --profile cross -DCMAKE_TOOLCHAIN_FILE=`pwd`/rostoolchain.cmake -DCMAKE_CXX_STANDARD=17 -DCATKIN_ENABLE_TESTING=OFF -DSETUPTOOLS_DEB_LAYOUT=OFF -DCMAKE_CXX_FLAGS="-DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-psabi -Wno-deprecated-copy -DNON_POLLING" "$@"
