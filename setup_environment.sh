#!/bin/bash
jetson=true
version="tx2"
gpu=true

#process args
while [ $# -gt 0 ]
do
	case "$1" in
	-tx1) jetson=true;verson="tx1";;
	-tx2) jetson=true;version="tx2";;
	-amd64) jetson=false;;
	-c) gpu=false;;
	-h) echo >&2 \
		"usage: $0 [-tx1 or -tx2 or -amd64] [-c] [-h]"
		exit 1;;
	*)  break;;	# terminate while loop
	esac
	shift
done

#install basic dependencies

sudo apt update
sudo apt -y upgrade
sudo apt install -y libeigen3-dev build-essential gfortran git cmake libleveldb-dev libsnappy-dev libhdf5-dev libhdf5-serial-dev liblmdb-dev vim-gtk libgflags-dev libgoogle-glog-dev libatlas-base-dev python-dev python-pip libtinyxml2-dev v4l-conf v4l-utils libgtk2.0-dev pkg-config exfat-fuse exfat-utils libprotobuf-dev protobuf-compiler unzip python-numpy python-scipy python-opencv python-matplotlib chromium-browser wget unzip ccache ntp ntpdate libflann-dev libpcl-dev libproj-dev htop can-utils gstreamer1.0-plugins-* rsync openssh-client openssh-client terminator ninja-build libsuitesparse-dev xfonts-scalable rsync libboost-all-dev

#install caffe
# cd
# git clone https://github.com/BVLC/caffe.git
# cd caffe
# mkdir build
# cd build

# if [ "$gpu" == "false" ] ; then
	# cmake -DCPU_ONLY=ON ..
# else
	# cmake -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF ..
# fi

# make -j4 all
#make test
#make runtest
# make -j4 install

# Install libsodium - this is a prereq for zeromq 1.0.14 released 9/21/2017
# cd
# wget --no-check-certificate https://download.libsodium.org/libsodium/releases/libsodium-1.0.14.tar.gz
# tar -zxvf libsodium-1.0.14.tar.gz
# cd libsodium-1.0.14
# ./configure
# make -j4 
# sudo make install
# cd ..
# rm -rf libsodium-1.0.14*

# install zeromq 4.2.2 is latest stable as of 9/20/2017
# cd
# wget --no-check-certificate https://github.com/zeromq/libzmq/releases/download/v4.2.2/zeromq-4.2.2.tar.gz
# tar -xzvf zeromq-4.2.2.tar.gz
# cd zeromq-4.2.2
# ./configure
# make -j4
# sudo make install
# cd ..
# rm -rf zeromq-4.2.2*
# cd /usr/local/include/
# sudo wget --no-check-certificate https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq.hpp

# Install tinyxml2
cd
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..
rm -rf tinyxml2

#install zed sdk
if [ "$version" = tx1 ] && [ "$jetson" = true ] ; then
	$zed_arch="JTX1_JP3.2"
elif [ "$version" = tx2 ] && [ "$jetson" = true ] ; then
	$zed_arch="JTX2_JP3.2"
else
	$zed_arch="Ubuntu16"
fi

zed_ver="2.7.1"
zed_fn="ZED_SDK_"$zed_arch"_v"$zed_ver".run"
wget --no-check-certificate https://www.stereolabs.com/download/$zed_fn
chmod 755 $zed_fn
./$zed_fn
rm ./$zed_fn

#clone repo
#TODO : rethink this - how are we getting the script if the
#       repo isn't there in the first place?
cd
git clone https://github.com/FRC900/2019RobotCode.git
cd 2019RobotCode
git submodule init
git submodule update

#mount and setup autostart script
if [ "$jetson" = true ] ; then
	sudo mkdir /mnt/900_2

	# For tx2 only - install drivers for USB
	# serial devices
	if [ "$version" = tx2 ] ; then
		cd ~/2019RobotCode
		sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
		sudo cp jetson_setup/cp210x.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x.ko
		sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
		sudo cp jetson_setup/cdc-acm.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
		sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/net/can/usb
		sudo cp jetson_setup/gs_usb.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/net/can/usb/gs_usb.ko
		sudo cp jetson_setup/can-dev.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/net/can/can-dev.ko 
		sudo mkdir -p /lib/modules/`uname -r`/kernel/net/can
		sudo cp jetson_setup/can.ko.`uname -r` /lib/modules/`uname -r`/kernel/net/can/can.ko 
		sudo cp jetson_setup/can-raw.ko.`uname -r` /lib/modules/`uname -r`/kernel/net/can/can-raw.ko 

		sudo depmod -a
        # edit /etc/init.d/ntp to contain the line: <ntpd -gq> before all content already there.
        sudo cp ntp-client.conf /etc/ntp.conf  # edit /etc/ntp.conf to be a copy of ntp-client.conf in 2019RobotCode

		# Set up can0 network interface
		cd
		echo "auto can0" > can0
		echo "iface can0 inet manual" >> can0
		echo "  pre-up /sbin/ip link set can0 type can bitrate 1000000" >> can0
		echo "  up /sbin/ifconfig can0 up" >> can0
		echo "  down /sbin/ifconfig can0 down" >> can0
		sudo mv can0 /etc/network/interfaces.d

		sudo bash -c "echo \"# Modules for CAN interface\" >> /etc/modules"
		sudo bash -c "echo can >> /etc/modules"
		sudo bash -c "echo can_raw >> /etc/modules"
		sudo bash -c "echo can_dev >> /etc/modules"
		sudo bash -c "echo gs_can >> /etc/modules"
		#sudo bash -c "echo mttcan >> /etc/modules"

		# This shouldn't be the least bit dangerous
		#sudo rm /etc/modprobe.d/blacklist-mttcan.conf 
	fi
	#find /etc/systemd/system -name 'nv-l4t-usb-device-mode*' -delete

	# Set up ssh host config (add port 5801) 
	sudo sed "s/Port 22/Port 22\nPort 5801/g" /etc/ssh/sshd_config > sshd_config && sudo mv sshd_config /etc/ssh
	
	# and keys for 
	# connections to Rio
	mkdir -p ~/.ssh
	cd ~/.ssh
	tar -xjf ~/2019RobotCode/jetson_setup/jetson_dot_ssh.tar.bz2 
	chmod 640 authorized_keys
	cd ~
	chmod 700 .ssh

	sudo mkdir -p /root/.ssh
	sudo tar -xjf /home/ubuntu/2019RobotCode/jetson_setup/jetson_dot_ssh.tar.bz2 -C /root/.ssh
	sudo chmod 640 /root/.ssh/authorized_keys
	sudo chmod 700 /root/.ssh

	cd ~/2019RobotCode
	sudo cp ./jetson_setup/10-local.rules /etc/udev/rules.d/
	sudo service udev reload
	sleep 2
	sudo service udev restart

	# Kernel module build steps for TX2 : https://gist.github.com/sauhaardac/9d7a82c23e4b283a1e79009903095655
	# Not needed unless Jetpack is updated with a new kernel version and modules
	# for a given kernel version aren't already built
	#
	# Note - need to switch back to the default linker to build the kernel image
	# cd ~
	# wget -N https://developer.download.nvidia.com/embedded/L4T/r28_Release_v2.1/public_sources.tbz2
	# tar -xf public_sources.tbz2 public_release/kernel_src.tbz2
	# tar -xf public_release/kernel_src.tbz2
	# cd ..
	# cd ~/kernel/kernel-4.4
	# zcat /proc/config.gz > .config
	# echo "CONFIG_USB_ACM=m" >> .config
	# echo "CONFIG_USB_SERIAL_CP210X=m" >> .config
	# echo "CONFIG_CAN_GS_USB=m" >> .config
	# echo "CONFIG_JOYSTICK_XPAD=m" >> .config
	## Apply realsense patches to modules
	# patch -p1 < ~/realsense_src/librealsense-2.19.2/scripts/realsense-camera-formats_ubuntu-xenial-master.patch 
	# patch -p1 < ~/realsense_src/librealsense-2.19.2/scripts/realsense-metadata-ubuntu-xenial-master.patch
	# patch -p1 < ~/realsense_src/librealsense-2.19.2/scripts/realsense-hid-ubuntu-xenial-master.patch
	# patch -p1 < ~/realsense_src/librealsense-2.19.2/scripts/realsense-powerlinefrequency-control-fix.patch
	# bash scripts/config --file .config --enable IIO_BUFFER --enable IIO_KFIFO_BUF --module IIO_TRIGGERED_BUFFER --enable IIO_TRIGGER --set-val IIO_CONSUMERS_PER_TRIGGER 2 --module HID_SENSOR_IIO_COMMON --module HID_SENSOR_IIO_TRIGGER --module HID_SENSOR_HUB --module HID_SENSOR_ACCEL_3D --module HID_SENSOR_GYRO_3D --module USB_ACM --module CAN_GS_USB --module JOYSTICK_XPAD
	# 
	# make -j6 clean
	# make -j6 prepare
	# make -j6 modules_prepare
    # make -j6 Image
	# sudo cp arch/arm64/boot/Image boot/Image
    # make -j6 modules
	# sudo make -j6 modules_install
	# make -j6 M=drivers/usb/class
	# make -j6 M=drivers/usb/serial
	# make -j6 M=drivers/net/can
	# make -j6 M=net/can
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
	# sudo cp drivers/usb/class/cp210x-acm.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x-acm.ko
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
	# sudo cp drivers/usb/serial/cdc-acm.ko /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
	# sudo cp drivers/net/can/usb/gs_usb.ko /lib/modules/`uname -r`/kernel/drivers/net/can/usb

	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/joystick
	# sudo cp xpad.ko /lib/modules/`uname -r`/kernel/drivers/joystick/xpad.ko
	# sudo depmod -a

	# Clean up Jetson
	sudo rm -rf /home/nvidia/cudnn /home/nvidia/OpenCV /home/nvidia/TensorRT /home/nvidia/libvisionworkd*
	# Save ~400MB
	sudo apt remove --purge -y thunderbird libreoffice-*

	# Install CTRE & navX libs
    mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include 
	mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre 
    cd /home/ubuntu 
	wget -e robots=off -U mozilla -r -np http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/ -A "*5.14.1*,firmware-sim*zip" -R "md5,sha1,pom,jar,*windows*"
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include 
	find /home/ubuntu/devsite.ctr-electronics.com -name \*headers\*zip | xargs -n 1 unzip -o 
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre 
	find /home/ubuntu/devsite.ctr-electronics.com -name \*linux\*zip | xargs -n 1 unzip -o 
    rm -rf /home/ubuntu/devsite.ctr-electronics.com 

    cd /home/ubuntu 
	wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.377/navx-cpp-3.1.377-headers.zip 
	mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/navx 
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/navx 
	unzip -o /home/ubuntu/navx-cpp-3.1.377-headers.zip 
	rm /home/ubuntu/navx-cpp-3.1.377-headers.zip 
    cd /home/ubuntu 
	wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.377/navx-cpp-3.1.377-linuxathena.zip 
	mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/navx 
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/navx 
	unzip -o /home/ubuntu/navx-cpp-3.1.377-linuxathena.zip 
	rm /home/ubuntu/navx-cpp-3.1.377-linuxathena.zip 
    cd /home/ubuntu 
	wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.377/navx-cpp-3.1.377-linuxathenastatic.zip 
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/navx 
	unzip -o /home/ubuntu/navx-cpp-3.1.377-linuxathenastatic.zip 
	rm /home/ubuntu/navx-cpp-3.1.377-linuxathenastatic.zip 

	# Install wpilib headers by copying them from the local maven dir
    cd /home/ubuntu 
	wget https://github.com/wpilibsuite/allwpilib/releases/download/v2019.4.1/WPILib_Linux-2019.4.1.tar.gz 
	mkdir -p /home/ubuntu/frc2019 
    cd /home/ubuntu/frc2019 
	tar -xzf /home/ubuntu/WPILib_Linux-2019.4.1.tar.gz 
	rm /home/ubuntu/WPILib_Linux-2019.4.1.tar.gz 
    cd /home/ubuntu/frc2019/tools 
	python ToolsUpdater.py 
    mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/wpilib 
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/wpilib 
	find ../../../.. -name \*athena\*zip | xargs -n1 unzip -o 
    mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/wpilib 
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/wpilib 
	find ../../../.. -name \*headers\*zip | xargs -n1 unzip -o 
    rm -rf /home/ubuntu/frc2019/maven /home/ubuntu/frc2019/jdk
	sed -i -e 's/   || defined(__thumb__) \\/   || defined(__thumb__) \\\n   || defined(__aarch64__) \\/' /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/wpilib/FRC_FPGA_ChipObject/fpgainterfacecapi/NiFpga.h

	# Set up prereqs for deploy script
	mv ~/2019RobotCode ~/2019RobotCode.orig
	ln -s ~/2019RobotCode.orig ~/2019RobotCode
	mkdir -p ~/2019RobotCode.prod/zebROS_ws
	mkdir -p ~/2019RobotCode.dev/zebROS_ws
fi

sudo mkdir -p /usr/local/zed/settings
sudo chmod 755 /usr/local/zed/settings
sudo cp ~/2019RobotCode/calibration_files/*.conf /usr/local/zed/settings
sudo chmod 644 /usr/local/zed/settings/*

cp ~/2019RobotCode/.vimrc ~/2019RobotCode/.gvimrc ~
sudo cp ~/2019RobotCode/kjaget.vim /usr/share/vim/vim74/colors

git config --global user.email "progammers@team900.org"
git config --global user.name "Team900 Jetson TX2"

# Set up Gold linker - speed up libPCL links
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.gold" 20
sudo update-alternatives --install "/usr/bin/ld" "ld" "/usr/bin/ld.bfd" 10

sudo update-alternatives --config ld
