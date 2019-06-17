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
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
# Installation
sudo apt update

# For intel realsense - from apt for x86 laptops
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE 
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u 
sudo apt update

# From source for the Jetson
sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev 
mkdir realsense_src && cd realsense_src
wget https://github.com/IntelRealSense/librealsense/archive/v2.23.0.zip
unzip v2.23.0.zip
cd librealsense-2.23.0
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true ..
sudo make uninstall && make clean && make -j6 && sudo make install

# Add Individual Packages here
# You can install a specific ROS package (replace underscores with dashes of the package name):
# sudo apt-get install ros-kinetic-PACKAGE
# e.g.
# sudo apt-get install ros-kinetic-navigation
#
# To find available packages:
# apt-cache search ros-kinetic
# 
sudo apt install ros-kinetic-ros-base python-rosdep python-rosinstall terminator ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-tf2-ros ros-kinetic-pcl-conversions ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-map-server ros-kinetic-rviz ros-kinetic-hector-slam ros-kinetic-hector-slam-launch ros-kinetic-rtabmap-ros ros-kinetic-robot-localization ros-kinetic-navigation ros-kinetic-robot-state-publisher ros-kinetic-rosparam-shortcuts python-wstool ninja-build libsuitesparse-dev ros-kinetic-tf2-tools ros-kinetic-hardware-interface ros-kinetic-controller-manager ros-kinetic-control-msgs ros-kinetic-joint-limits-interface ros-kinetic-transmission-interface liblua5.3-dev ros-kinetic-joystick-drivers ros-kinetic-gmapping ros-kinetic-teb-local-planner ros-kinetic-roslint ros-kinetic-xacro ros-kinetic-rqt-controller-manager ros-kinetic-serial ros-kinetic-ecl-geometry ros-kinetic-rviz-imu-plugin ros-kinetic-rosbridge-suite ros-kinetic-grid-map-core ros-kinetic-grid-map-cv ros-kinetic-grid-map-ros ros-kinetic-ar-track-alvar ros-kinetic-teraranger-* ros-kinetic-pid ros-kinetic*mux* ros-kinetic-usb-cam python-pyqtgraph librealsense2-dev librealsense2-dkms librealsense2-utils -y

# Initialize rosdep
# ssl certificates can get messed up on TX1 for some reason
sudo c_rehash /etc/ssl/certs
# Initialize rosdep
sudo rosdep init
# To find available packages, use:
rosdep update

cd ~/2019RobotCode/zebROS_ws/src
#wstool init
#cd ..

# These should all be merged in the GIT repo version
# of the code.  Check 2019RobotCode/zebROS_ws/src/.rosinstall to
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
#wstool merge https://raw.githubusercontent.com/FRC900/ros_control_boilerplate/kinetic-devel/ros_control_boilerplate.rosinstall

# LIDAR driver
#wstool merge https://raw.githubusercontent.com/FRC900/rplidar_ros/master/rplidar.rosinstall

# Pull latest version of merged repos 
wstool update -j 4

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro=kinetic -y

source /opt/ros/kinetic/setup.bash

cd ~/2019RobotCode/zebROS_ws
catkin_make
