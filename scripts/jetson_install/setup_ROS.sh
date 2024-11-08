/bin/bash
# ROS Setup install script for new jetsons
# Source: https://github.com/jetsonhacks/installROSTX1/blob/master/installROS.sh

# Run me after running through setup_environment.sh
# TODO - any reason not to merge these files?

# Setup Locale
# sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
# Setup sources.lst
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# Grab new package lists from ros.org
sudo apt update

# For intel realsense - from apt for x86 laptops, not yet (if ever) available for AARCH64
# See https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md or our Dockerfile

# From source for the Jetson
sudo apt install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev ninja-build libcurl4
cd
mkdir realsense_src
cd realsense_src
wget https://github.com/IntelRealSense/librealsense/archive/v2.50.0.zip
unzip v2.50.0.zip
cd librealsense-2.50.0
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -GNinja ..
sudo ninja uninstall && sudo ninja clean && sudo ninja install
cd 
sudo rm -rf realsense_src

sudo ccache -C
sudo ccache -c
sudo rm -rf /home/ubuntu/.cache /home/ubuntu/.ccache

# Add Individual Packages here
# You can install a specific ROS package (replace underscores with dashes of the package name):
# sudo apt-get install ros-noetic-PACKAGE
# e.g.
# sudo apt-get install ros-noetic-navigation
#
# To find available packages:
# apt-cache search ros-noetic
# 
# Keep each package on a separate line to aid in git merging
# Try to keep them in alphabetic order to make it easier to find duplicates

# Pin stock ubuntu 20.04 opencv (version 4.2.0) instead of jetpack version 4.5.4
sudo bash -c "echo Package: libopencv >> /etc/apt/preferences.d/libopencv"
sudo bash -c "echo Pin: release a=* >> /etc/apt/preferences.d/libopencv"
sudo bash -c "echo Pin-Priority: -10 >> /etc/apt/preferences.d/libopencv"

sudo bash -c "echo Package: libopencv-dev >> /etc/apt/preferences.d/libopencv-dev"
sudo bash -c "echo Pin: version 4.2* >> /etc/apt/preferences.d/libopencv-dev"
sudo bash -c "echo Pin-Priority: 1000 >> /etc/apt/preferences.d/libopencv-dev"

sudo apt purge -y libopencv
sudo apt install -y \
	ros-noetic-ros-base \
	liblua5.3-dev \
	libsuitesparse-dev \
	ninja-build \
	python3-catkin-tools \
	python3-pyqtgraph \
	python3-rosdep \
	python3-rosinstall \
	python3-wstool \
	ros-noetic-actionlib-tools \
	ros-noetic-controller-manager \
	ros-noetic-control-msgs \
	ros-noetic-cv-bridge \
	ros-noetic-cv-camera \
	ros-noetic-ecl-geometry \
	ros-noetic-grid-map-core \
	ros-noetic-grid-map-cv \
	ros-noetic-grid-map-ros \
	ros-noetic-hardware-interface \
	ros-noetic-image-transport-plugins \
	ros-noetic-imu-filter-madgwick \
	ros-noetic-joint-limits-interface \
	ros-noetic-joint-state-publisher \
	ros-noetic-joy \
	ros-noetic-map-server \
	ros-noetic-marker-msgs \
	ros-noetic-moveit \
	ros-noetic*mux* \
	ros-noetic-navigation \
	ros-noetic-pcl-conversions \
	ros-noetic-robot-localization \
	ros-noetic-robot-state-publisher \
	ros-noetic-rosbridge-suite \
	ros-noetic-roslint \
	ros-noetic-rosparam-shortcuts \
	ros-noetic-rqt \
	ros-noetic-rqt-common-plugins \
	ros-noetic-rqt-controller-manager \
	ros-noetic-rtabmap-ros \
	ros-noetic-rviz \
	ros-noetic-rviz-imu-plugin \
	ros-noetic-serial \
	ros-noetic-teb-local-planner \
	ros-noetic-tf \
	ros-noetic-tf2-ros \
	ros-noetic-tf2-tools \
	ros-noetic-transmission-interface \
	ros-noetic-usb-cam \
	ros-noetic-xacro \
	terminator 

# Patch catkin tools/pkg for faster builds
cd /usr/lib/python3/dist-packages
sudo patch -p0 < ~/900RobotCode/scripts/jetson_install/catkin_pkg.patch
#sudo patch -p0 < ~/900RobotCode/scripts/jetson_install/catkin_tools.patch

# Fix bug in released version of catkin_tools - TODO check me
#sudo sed -i 's/ errno.EINTR/ errno.EINTR and e.errno != errno.EAGAIN/'  /usr/lib/python2.7/dist-packages/catkin_tools/execution/job_server.py


# Install gazebo sim - commented out because we don't
# want/need it taking up space on the Jetson
#curl -sSL http://get.gazebosim.org | sh
# sudo apt install -y \
#	ros-noetic-gazebo-ros-control \
#	ros-noetic-gazebo-ros-pkgs \

# Not for noetic - ros-noetic-hector-slam ros-noetic-hector-slam-launch ros-noetic-gmapping 
# handled by wstool for now ros-noetic-teraranger-* 

# Initialize rosdep
# ssl certificates can get messed up on TX1 for some reason
sudo c_rehash /etc/ssl/certs
# Initialize rosdep
sudo rosdep init
# To find available packages, use:
rosdep update

cd ~/900RobotCode/zebROS_ws/src
#wstool init
#cd ..

# These should all be merged in the GIT repo version
# of the code.  Check 900RobotCode/zebROS_ws/src/.rosinstall to
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
#wstool merge https://raw.githubusercontent.com/FRC900/ros_control_boilerplate/noetic-devel/ros_control_boilerplate.rosinstall

# LIDAR driver
#wstool merge https://raw.githubusercontent.com/FRC900/rplidar_ros/master/rplidar.rosinstall

# Pull latest version of merged repos 
wstool update -j 4

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro=noetic -y

#source /opt/ros/noetic/setup.bash

#cd ~/900RobotCode/zebROS_ws
#catkin_make


sudo apt clean
sudo apt autoclean
sudo apt clean
