#!/bin/bash

cd ~/900RobotCode/zebROS_ws/
echo INCOMPLETE > .native_build.status

if [ -z $ROS_ROOT ]; then
	source /opt/ros/noetic/setup.bash
	if [ ! -z devel/setup.bash ]; then
		source devel/setup.bash
	fi
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
	PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages
elif [[ ! $ROS_ROOT = "/opt/ros/noetic/share/ros" ]]; then
	echo -e "\e[1m\e[31mROS is not configured for a native build (maybe set up for a cross build instead?)\e[0m"
	echo -e "\e[1m\e[31mRun ./native_build.sh in a new terminal window\e[0m"
	exit 1
fi
export PATH=$PATH:/usr/local/cuda/bin

EXTRA_SKIPLIST_PACKAGES=""
EXTRA_CMD_LINE=""
uname -a | grep -q x86_64
if [ $? -eq 1 ]; then
	EXTRA_SKIPLIST_PACKAGES="
		as726x_controllers \
		canifier_controller \
		demo_tf_node \
		field \
		frcrobot_description \
		frcrobot_gazebo \
		gazebo_frcrobot_control \
		robot_characterization \
		robot_visualizer \
		rosbag_scripts \
		rospy_message_converter \
		rqt_driver_station_sim \
		stage_ros \
		template_controller \
		visualize_profile \
		zms_writer"
	EXTRA_CMD_LINE="--limit-status-rate 5"
fi

# Xavier NX is id 25, we only want to build the gpu apriltag code
# for those devices since that's all they'll run
# Don't use --buildlist, since that doesn't auto-build the dependencies
# of the packages listed
cat /sys/devices/soc0/soc_id 2> /dev/null | grep -q 25 
if [ $? -eq 0 ]; then
	EXPLICIT_PACKAGE_LIST="cv_camera gpu_apriltag controller_node apriltag_launch"
fi
catkin config --skiplist \
	ackermann_steering_controller \
	adi_driver \
	adi_pico_driver \
	base_trajectory \
	chomp_motion_planner \
	color_spin \
	controllers_2019 \
	controllers_2019_msgs \
	controllers_2020 \
	controllers_2020_msgs \
	controllers_2022 \
	controllers_2022_msgs \
	controllers_2023 \
	controllers_2023_msgs \
	cuda_apriltag_ros \
	deeptag_ros \
	diff_drive_controller \
	effort_controllers \
	force_torque_sensor_controller \
	four_wheel_steering_controller \
	goal_detection \
	gripper_action_controller \
	moveit_chomp_optimizer_adapter \
	moveit_planners_chomp \
	moveit_ros_benchmarks \
	moveit_ros_manipulation \
	moveit_ros_robot_interaction \
	navx_publisher \
	pf_localization \
	robot_characterization \
	realsense2_camera \
	realsense2_description \
	robot_visualizer \
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
	spinnaker_camera_driver \
	teraranger_array \
	teraranger_array_converter \
	turing_smart_screen \
	turret_2023 \
	velocity_controllers \
	zed_ros \
	wpilib_swerve_odom \
	$EXTRA_SKIPLIST_PACKAGES

export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages

catkin build -DCATKIN_ENABLE_TESTING=OFF -DBUILD_WITH_OPENMP=ON -DCMAKE_CXX_STANDARD=17 -DSETUPTOOLS_DEB_LAYOUT=OFF -DCMAKE_CXX_FLAGS="-DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-psabi -DNON_POLLING" $EXTRA_CMD_LINE $EXPLICIT_PACKAGE_LIST "$@"

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

uname -a | grep -q x86_64
if [ $? -ne 1 ]; then
  ./merge_compile_commands.sh
  echo "build/compile_commands.json created"
fi

