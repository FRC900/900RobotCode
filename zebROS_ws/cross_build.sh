#!/bin/bash

cd ~/900RobotCode/zebROS_ws/

if [ -z $ROS_ROOT ]; then
	#PATH=$PATH:$HOME/wpilib/2025/roborio/bin
	source ~/wpilib/2025/roborio/arm-nilrt-linux-gnueabi/sysroot/opt/ros/noetic/setup.bash
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages
elif [[ ! $ROS_ROOT = "$HOME/wpilib/2025/roborio/arm-nilrt-linux-gnueabi/sysroot/opt/ros/noetic/share/ros" ]]; then
	echo -e "\e[1m\e[31mROS is not configured for a cross build (maybe set up for a native build instead?)\e[0m"
	echo -e "\e[1m\e[31mRun ./cross_build.sh in a new terminal window\e[0m"
	exit 1
fi

# REMOVE JOINT TRAJECTORY CONTORLLER FROM THIS 
catkin config --profile cross -x _isolated --install --skiplist \
	ackermann_steering_controller \
	adi_driver \
	adi_pico_driver \
	ar_track_alvar \
	apriltag_launch \
    apriltag_ros \
	base_trajectory \
	compressed_image_transport \
	compressed_depth_image_transport \
	color_spin \
	controllers_2019 \
	controllers_2019_msgs \
	controllers_2020 \
	controllers_2020_msgs \
	controllers_2022 \
	controllers_2022_msgs \
	controllers_2023 \
	controllers_2023_msgs \
	cv_camera \
	cuda_apriltag_ros \
	cv_camera \
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
	gpu_apriltag \
	gripper_action_controller \
	image_reformatter \
	image_transport_plugins \
	joint_trajectory_controller \
	navx_publisher \
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
	spinnaker_camera_driver \
	spline_util \
	stage_ros \
	tagslam \
	tagslam_viz \
	tagslam_test \
	tagslam_launch \
	teleop_joystick_control \
	template_controller \
	teraranger_array \
	teraranger_array_converter \
	tf_object_detection \
	theora_image_transport \
	turing_smart_screen \
	velocity_controllers \
	visualize_profile \
	zed_nodelets \
	zed_ros \
	zed_wrapper \
	zms_writer \
	wpilib_swerve_odom \
	drive_to_object \
	norfair_ros
catkin build --profile cross -DCMAKE_TOOLCHAIN_FILE=`pwd`/rostoolchain.cmake -DCMAKE_CXX_STANDARD=17 -DCATKIN_ENABLE_TESTING=OFF -DSETUPTOOLS_DEB_LAYOUT=OFF -DCMAKE_CXX_FLAGS="-DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-psabi -Wno-deprecated-copy -DNON_POLLING" "$@"
