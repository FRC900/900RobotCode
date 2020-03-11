#!/bin/bash
# ROS Setup install script for new jetsons
# Source: https://github.com/jetsonhacks/installROSTX1/blob/master/installROS.sh

# Run me after running through setup_environment.sh
# TODO - any reason not to merge these files?

# Setup Locale
# sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Grab new package lists from ros.org
sudo apt update

# For intel realsense - from apt for x86 laptops, not yet (if ever) available for AARCH64
#sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE 
#sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u 
#sudo apt update
#sudo apt install -y librealsense2-dev librealsense2-dkms librealsense2-utils

# From source for the Jetson
sudo apt install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev ninja-build
cd
mkdir realsense_src
cd realsense_src
wget https://github.com/IntelRealSense/librealsense/archive/v2.32.1.zip
unzip v2.32.1.zip
cd librealsense-2.32.1
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true  -GNinja ..
sudo ninja uninstall && sudo ninja clean && sudo ninja install
cd 
rm -rf realsense_src

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccach

# Add Individual Packages here
# You can install a specific ROS package (replace underscores with dashes of the package name):
# sudo apt-get install ros-melodic-PACKAGE
# e.g.
# sudo apt-get install ros-melodic-navigation
#
# To find available packages:
# apt-cache search ros-melodic
# 
# Keep each package on a separate line to aid in git merging
# Try to keep them in alphabetic order to make it easier to find duplicates
sudo apt install -y \
	ros-melodic-ros-base \
	liblua5.3-dev \
	libsuitesparse-dev \
	ninja-build \
	python-catkin-tools \
	python-pyqtgraph \
	python-rosdep \
	python-rosinstall \
	python-wstool \
	ros-melodic-ar-track-alvar \
	ros-melodic-controller-manager \
	ros-melodic-control-msgs \
	ros-melodic-cv-bridge \
	ros-melodic-ecl-geometry \
	ros-melodic-grid-map-core \
	ros-melodic-grid-map-cv \
	ros-melodic-grid-map-ros \
	ros-melodic-hardware-interface \
	ros-melodic-imu-filter-madgwick \
	ros-melodic-joint-limits-interface \
	ros-melodic-joint-state-publisher \
	ros-melodic-joystick-drivers \
	ros-melodic-map-server \
	ros-melodic*mux* \
	ros-melodic-navigation \
	ros-melodic-pcl-conversions \
	ros-melodic-pid \
	ros-melodic-robot-localization \
	ros-melodic-robot-state-publisher \
	ros-melodic-rosbridge-suite \
	ros-melodic-roslint \
	ros-melodic-rosparam-shortcuts \
	ros-melodic-rqt \
	ros-melodic-rqt-common-plugins \
	ros-melodic-rqt-controller-manager \
	ros-melodic-rtabmap-ros \
	ros-melodic-rviz \
	ros-melodic-rviz-imu-plugin \
	ros-melodic-serial \
	ros-melodic-teb-local-planner \
	ros-melodic-tf \
	ros-melodic-tf2-ros \
	ros-melodic-tf2-tools \
	ros-melodic-transmission-interface \
	ros-melodic-usb-cam \
	ros-melodic-xacro \
	terminator 

# Install gazebo sim - commented out because we don't
# want/need it taking up space on the Jetson
#curl -sSL http://get.gazebosim.org | sh
# sudo apt install -y \
#	ros-melodic-gazebo-ros-control \
#	ros-melodic-gazebo-ros-pkgs \

# Not for melodic - ros-melodic-hector-slam ros-melodic-hector-slam-launch ros-melodic-gmapping 
# handled by wstool for now ros-melodic-teraranger-* 

# Initialize rosdep
# ssl certificates can get messed up on TX1 for some reason
sudo c_rehash /etc/ssl/certs
# Initialize rosdep
sudo rosdep init
# To find available packages, use:
rosdep update

cd ~/2020RobotCode/zebROS_ws/src
#wstool init
#cd ..

# These should all be merged in the GIT repo version
# of the code.  Check 2020RobotCode/zebROS_ws/src/.rosinstall to
# verify.  Leaving the commands here just in case we need to recreate
# a workspace elsewhere

# Google Cartographer installation forked from https://google-cartographer-ros.readthedocs.io/en/latest/
# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
#wstool merge https://raw.githubusercontent.com/FRC900/cartographer_ros/master/cartographer_ros.rosinstall

# Code to use joystick to control robot
#wstool merge https://raw.githubusercontent.com/FRC900/teleop_twist_joy/indigo-devel/teleop_twist_joy.rosinstall
 
# Merge ZED wrapper
#wstool merge https://raw.githubusercontent.com/FRC900/zed-ros-wrapper/master/zed_ros_wrapper.rosinstall

# Boilerplate control code
#wstool merge https://raw.githubusercontent.com/FRC900/steered_wheel_base_controller/master/steered_wheel_base_controller.rosinstall
#wstool merge https://raw.githubusercontent.com/FRC900/ros_control_boilerplate/melodic-devel/ros_control_boilerplate.rosinstall

# LIDAR driver
#wstool merge https://raw.githubusercontent.com/FRC900/rplidar_ros/master/rplidar.rosinstall

# Pull latest version of merged repos 
wstool update -j 4

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro=melodic -y

#source /opt/ros/melodic/setup.bash

#cd ~/2020RobotCode/zebROS_ws
#catkin_make
