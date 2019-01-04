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

sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get install -y libeigen3-dev build-essential gfortran git cmake libleveldb-dev libsnappy-dev libhdf5-dev libhdf5-serial-dev liblmdb-dev vim-gtk libgflags-dev libgoogle-glog-dev libatlas-base-dev python-dev python-pip libtinyxml2-dev v4l-conf v4l-utils libgtk2.0-dev pkg-config exfat-fuse exfat-utils libprotobuf-dev protobuf-compiler unzip python-numpy python-scipy python-opencv python-matplotlib chromium-browser wget unzip ccache ntp ntpdate libflann-dev libpcl-dev libproj-dev htop can-utils

sudo apt-get install --no-install-recommends -y libboost-all-dev

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

# Set up prereqs for deploy script
mv ~/2019RobotCode ~/2019RobotCode.orig
ln -s ~/2019RobotCode.orig ~/2019RobotCode
mkdir -p ~/2019RobotCode.prod/zebROS_ws
mkdir -p ~/2019RobotCode.dev/zebROS_ws

#mount and setup autostart script
if [ "$jetson" = true ] ; then
	sudo mkdir /mnt/900_2

	# TODO - add "Port 5801" to /etc/ssh/sshd_config

	# For tx2 only - install drivers for USB
	# serial devices
	if [ "$version" = tx2 ] ; then
		sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
		sudo cp jetson_setup/cp210x.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x.ko
		sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
		sudo cp jetson_setup/cdc-acm.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
		sudo depmod -a
        sudo apt-get install ntp # TODO work on this NIALL or OLIVIA
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
		sudo bash -c "echo mttcan >> /etc/modules"

		# This shouldn't be the least bit dangerous
		sudo rm /etc/modprobe.d/blacklist-mttcan.conf 
	fi

	# Set up ssh host config (add port 5801) 
	sudo sed "s/Port 22/Port 22\nPort 5801/g" /etc/ssh/sshd_config > sshd_config && sudo mv sshd_config /etc/ssh
	
	# and keys for 
	# connections to Rio
	mkdir -p ~/.ssh
	cd ~/.ssh
	tar -xjf ~/2019RobotCode/jetson_setup/jetson_dot_ssh.tar.bz2 

	sudo mkdir -p /root/.ssh
	sudo cd /root/.ssh
	sudo tar -xjf /home/ubuntu/2019RobotCode/jetson_setup/jetson_dot_ssh.tar.bz2 

	cd ~/2019RobotCode
	sudo cp ./jetson_setup/10-local.rules /etc/udev/rules.d/
	sudo service udev reload
	sleep 2
	sudo service udev restart

	# Kernel module build steps for TX2 : https://gist.github.com/sauhaardac/9d7a82c23e4b283a1e79009903095655
	# Not needed unless Jetpack is updated with a new kernel version and modules
	# for a given kernel version aren't already built
	# cd ~
	# mkdir l4t-kernel-surgery
	# mkdir kernel-stuff
	# cd kernel-stuff
	# wget http://developer.nvidia.com/embedded/dlc/l4t-sources-28-1 -O source.tbz2
	# tar xjf source.tbz2
	# tar xjf sources/kernel_src-tx2.tbz2 -C ~/l4t-kernel-surgery/
	# cd ..
	# rm -rf kernel-stuff
	# cd ~/l4t-kernel-surgery/kernel/kernel-4.4
	# Add EXTRAVERSION=-tegra to Makefile
	# zcat /proc/config.gz > .config
	# echo "CONFIG_USB_ACM=m" >> .config
	# echo "CONFIG_USB_SERIAL_CP210X=m" >> .config
	# make -j6 clean
	# make -j6 prepare
	# make -j6 modules_prepare
	# make -j6 M=drivers/usb/class
	# make -j6 M=drivers/usb/serial
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/serial
	# sudo cp drivers/usb/class/cp210x-acm.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/serial/cp210x-acm.ko
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/usb/class
	# sudo cp drivers/usb/serial/cdc-acm.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/usb/class/cdc-acm.ko
	# sudo mkdir -p /lib/modules/`uname -r`/kernel/drivers/joystick
	# sudo cp xpad.ko.`uname -r` /lib/modules/`uname -r`/kernel/drivers/joystick/xpad.ko
	# sudo depmod -a

	# Clean up Jetson
	sudo rm -rf /home/nvidia/cudnn /home/nvidia/OpenCV /home/nvidia/TensorRT /home/nvidia/libvisionworkd*
	# Save ~400MB
	sudo apt remove --purge thunderbird libreoffice-*

	# Install CTRE & navX libs
	mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/api-cpp/5.10.0/api-cpp-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/api-cpp-5.10.0-headers.zip
	rm /home/ubuntu/api-cpp-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/canutils/5.10.0/canutils-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/canutils-5.10.0-headers.zip
	rm /home/ubuntu/canutils-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/cci/5.10.0/cci-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/cci-5.10.0-headers.zip
	rm /home/ubuntu/cci-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/core/5.10.0/core-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/core-5.10.0-headers.zip
	rm /home/ubuntu/core-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-ics/5.10.0/platform-ics-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/platform-ics-5.10.0-headers.zip
	rm /home/ubuntu/platform-ics-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-sim/5.10.0/platform-sim-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/platform-sim-5.10.0-headers.zip
	rm /home/ubuntu/platform-sim-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-socketcan/5.10.0/platform-socketcan-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/platform-socketcan-5.10.0-headers.zip
	rm /home/ubuntu/platform-socketcan-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-stub/5.10.0/platform-stub-5.10.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include
	unzip -o /home/ubuntu/platform-stub-5.10.0-headers.zip
	rm /home/ubuntu/platform-stub-5.10.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/api-cpp/5.10.0/api-cpp-5.10.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/api-cpp-5.10.0-linuxaarch64.zip
	rm /home/ubuntu/api-cpp-5.10.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/api-cpp/5.10.0/api-cpp-5.10.0-linuxathena.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/api-cpp-5.10.0-linuxathena.zip
	rm /home/ubuntu/api-cpp-5.10.0-linuxathena.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/api-cpp/5.10.0/api-cpp-5.10.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/api-cpp-5.10.0-linuxarmhf.zip
	rm /home/ubuntu/api-cpp-5.10.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/api-cpp/5.10.0/api-cpp-5.10.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/api-cpp-5.10.0-linuxx86-64.zip
	rm /home/ubuntu/api-cpp-5.10.0-linuxx86-64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/canutils/5.10.0/canutils-5.10.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/canutils-5.10.0-linuxaarch64.zip
	rm /home/ubuntu/canutils-5.10.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/canutils/5.10.0/canutils-5.10.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/canutils-5.10.0-linuxarmhf.zip
	rm /home/ubuntu/canutils-5.10.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/canutils/5.10.0/canutils-5.10.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/canutils-5.10.0-linuxx86-64.zip
	rm /home/ubuntu/canutils-5.10.0-linuxx86-64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/cci/5.10.0/cci-5.10.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/cci-5.10.0-linuxaarch64.zip
	rm /home/ubuntu/cci-5.10.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/cci/5.10.0/cci-5.10.0-linuxathena.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/cci-5.10.0-linuxathena.zip
	rm /home/ubuntu/cci-5.10.0-linuxathena.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/cci/5.10.0/cci-5.10.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/cci-5.10.0-linuxarmhf.zip
	rm /home/ubuntu/cci-5.10.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/cci/5.10.0/cci-5.10.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/cci-5.10.0-linuxx86-64.zip
	rm /home/ubuntu/cci-5.10.0-linuxx86-64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/firmware-sim/0.1.0/firmware-sim-0.1.0-headers.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/ctre
	unzip -o /home/ubuntu/firmware-sim-0.1.0-headers.zip
	rm /home/ubuntu/firmware-sim-0.1.0-headers.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/firmware-sim/0.1.0/firmware-sim-0.1.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/firmware-sim-0.1.0-linuxaarch64.zip
	rm /home/ubuntu/firmware-sim-0.1.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/firmware-sim/0.1.0/firmware-sim-0.1.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/firmware-sim-0.1.0-linuxarmhf.zip
	rm /home/ubuntu/firmware-sim-0.1.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/firmware-sim/0.1.0/firmware-sim-0.1.0-linuxathena.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/firmware-sim-0.1.0-linuxathena.zip
	rm /home/ubuntu/firmware-sim-0.1.0-linuxathena.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/firmware-sim/0.1.0/firmware-sim-0.1.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/firmware-sim-0.1.0-linuxx86-64.zip
	rm /home/ubuntu/firmware-sim-0.1.0-linuxx86-64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-ics/5.10.0/platform-ics-5.10.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-ics-5.10.0-linuxaarch64.zip
	rm /home/ubuntu/platform-ics-5.10.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-ics/5.10.0/platform-ics-5.10.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-ics-5.10.0-linuxarmhf.zip
	rm /home/ubuntu/platform-ics-5.10.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-ics/5.10.0/platform-ics-5.10.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-ics-5.10.0-linuxx86-64.zip
	rm /home/ubuntu/platform-ics-5.10.0-linuxx86-64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-sim/5.10.0/platform-sim-5.10.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-sim-5.10.0-linuxaarch64.zip
	rm /home/ubuntu/platform-sim-5.10.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-sim/5.10.0/platform-sim-5.10.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-sim-5.10.0-linuxarmhf.zip
	rm /home/ubuntu/platform-sim-5.10.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-sim/5.10.0/platform-sim-5.10.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-sim-5.10.0-linuxx86-64.zip
	rm /home/ubuntu/platform-sim-5.10.0-linuxx86-64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-socketcan/5.10.0/platform-socketcan-5.10.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-socketcan-5.10.0-linuxaarch64.zip
	rm /home/ubuntu/platform-socketcan-5.10.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-socketcan/5.10.0/platform-socketcan-5.10.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-socketcan-5.10.0-linuxarmhf.zip
	rm /home/ubuntu/platform-socketcan-5.10.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-socketcan/5.10.0/platform-socketcan-5.10.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-socketcan-5.10.0-linuxx86-64.zip
	rm /home/ubuntu/platform-socketcan-5.10.0-linuxx86-64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-stub/5.10.0/platform-stub-5.10.0-linuxaarch64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-stub-5.10.0-linuxaarch64.zip
	rm /home/ubuntu/platform-stub-5.10.0-linuxaarch64.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-stub/5.10.0/platform-stub-5.10.0-linuxarmhf.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-stub-5.10.0-linuxarmhf.zip
	rm /home/ubuntu/platform-stub-5.10.0-linuxarmhf.zip

    cd /home/ubuntu
	wget http://devsite.ctr-electronics.com/maven/release/com/ctre/frcbeta/phoenix/platform-stub/5.10.0/platform-stub-5.10.0-linuxx86-64.zip
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre
	unzip -o /home/ubuntu/platform-stub-5.10.0-linuxx86-64.zip
	rm /home/ubuntu/platform-stub-5.10.0-linuxx86-64.zip

	cd /home/ubuntu
	wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.336/navx-cpp-3.1.336-headers.zip
	mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/navx
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/navx
	unzip -o /home/ubuntu/navx-cpp-3.1.336-headers.zip
	rm /home/ubuntu/navx-cpp-3.1.336-headers.zip

    cd /home/ubuntu
	wget http://www.kauailabs.com/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.336/navx-cpp-3.1.336-linuxathena.zip
	mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/navx
	cd /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/navx
	unzip -o /home/ubuntu/navx-cpp-3.1.336-linuxathena.zip
	rm /home/ubuntu/navx-cpp-3.1.336-linuxathena.zip 
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
