#!/bin/bash

# Run me with IP address of Rio as argument
#   Rio must be hooked up to the internet to grab packages correctly
# Must be run from inside docker container - this script copies wpilib
#   libraries to the Rio, and those only exist inside the container
#   matching the version of wpilib expected on the Rio
which docker | grep -q docker
if [ $? -ne 1 ] ; then
	echo "This script must be run from inside a docker conatiner"
fi

ssh -p 22 admin@$1 'swapon /dev/sda5'

# Set date on Rio. This should prevent weird auth errors
# from pip install commands below.
ssh -p 22 admin@$1 'opkg update'
ssh -p 22 admin@$1 'opkg install ntp ntp-tickadj ntp-utils ntpdate'
ssh -p 22 admin@$1 '/etc/init.d/ntpd stop'
ssh -p 22 admin@$1 'ntpdate us.pool.ntp.org'

# Split these up so the disk doesn't fill up with temp files
# Also need to install pyyaml first for some reason to avoid
# weird dependency hell issues with opkg (fixed in 2019 or 2020?)
#python3-argparse python3-netifaces

# i2c-tools, probably not needed anymore
# gperftools-dev, probably not needed anymore

ssh -p 22 admin@$1 'opkg install python3-pyyaml'
ssh -p 22 admin@$1 'opkg install python3-dev libpython3 python3-core python3-logging'
ssh -p 22 admin@$1 'opkg install python3-setuptools python3-pycrypto python3-pycrypto-dev python3-fcntl'
ssh -p 22 admin@$1 'opkg install python3-pkgutil python3-dateutil python3-nose python3-pydoc'
ssh -p 22 admin@$1 'opkg install libcurl4 lz4 libboost-thread1.66.0 libboost-chrono1.66.0 libboost-date-time1.66.0 libboost-atomic1.66.0'
ssh -p 22 admin@$1 'opkg install libeigen libbz2 libxml2 libgnutls-bin libgnutls-openssl27'
ssh -p 22 admin@$1 'opkg install libgnutls30 libgnutlsxx28 nettle libgmp10 libgmpxx4 libz1 cmake make'
ssh -p 22 admin@$1 'opkg install python3-pip coreutils python3-psutil'
ssh -p 22 admin@$1 'opkg install rsync htop curl libusb-1.0-dev'
ssh -p 22 admin@$1 'opkg install gflags gflags-bash-completion libglog0 openssl'
ssh -p 22 admin@$1 'opkg install gpgme-dev gpgme libgpg-error-dev libgpg-error0 libassuan-dev libassuan0'
ssh -p 22 admin@$1 'opkg clean'

#ssh -p 22 admin@$1 'opkg install gdb'

# Terrible hack. ROS is built with 3.10 from a default 22.04 x86_64 host
# but the Rio has 3.5. This might make things work?
ssh -p 22 admin@$1 'ln -sf /usr/bin/python3 /usr/bin/python3.10'

# Fails with 
#    Cannot remove entries from nonexistent file /usr/lib/python3.5/site-packages/easy-install.pth
ssh -p 22 admin@$1 'pip3 install --upgrade pip'

#ssh -p 22 admin@$1 'opkg install python3-numpy'
# Pip install has to build some packages from source - install GCC dev env to make that work
ssh -p 22 admin@$1 'opkg install gcc gcc-symlinks cpp-symlinks libgcc-s-dev binutils'
# netbase-dev
ssh -p 22 admin@$1 'pip3 --no-cache-dir install catkin_pkg catkin_tools rospkg rosdistro vcstools rosdep wstool rosinstall rosinstall_generator defusedxml empy python-gnupg netifaces pycryptodomex'
ssh -p 22 admin@$1 'opkg remove --autoremove gcc gcc-symlinks cpp-symlinks libgcc-s-dev binutils'
ssh -p 22 admin@$1 'rm -rf ~/.cache'

#scp -P 22 ~/2022RobotCode/scripts/RIO_setup/tflite_runtime-1.15.2-cp27-none-linux_armv7l.whl admin@$1:.
#ssh -p 22 admin@$1 'pip --no-cache-dir install tflite_runtime-1.15.2-cp27-none-linux_armv7l.whl'
#ssh -p 22 admin@$1 'rm tflite_runtime-1.15.2-cp27-none-linux_armv7l.whl'
#ssh -p 22 admin@$1 'rm -rf ~/.cache'

# Copy over ROS tar.bz2 file, extract to / on the Rio
scp -P 22 /home/ubuntu/roscore_roborio.tar.bz2 admin@$1:.
ssh -p 22 admin@$1 'cd / && tar -xjf ~/roscore_roborio.tar.bz2'
ssh -p 22 admin@$1 'rm ~/roscore_roborio.tar.bz2'
cd /home/ubuntu/wpilib/2023/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/local/lib
scp -P 22 libboost_atomic.so.1.74.0 libboost_chrono.so.1.74.0 libboost_program_options.so.1.74.0 libboost_regex.so.1.74.0 libboost_system.so.1.74.0 libboost_filesystem.so.1.74.0 libboost_thread.so.1.74.0 libboost_date_time.so.1.74.0 admin@$1:/usr/lib
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_atomic.so.1.74 /usr/lib/libboost_atomic.so.1.74.0
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_chrono.so.1.74 /usr/lib/libboost_chrono.so.1.74.0
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_program_options.so.1.74 /usr/lib/libboost_program_options.so.1.74.0
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_regex.so.1.74 /usr/lib/libboost_regex.so.1.74.0
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_system.so.1.74 /usr/lib/libboost_system.so.1.74.0
##ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_signals.so.1.74 /usr/lib/libboost_signals.so.1.74.0
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_filesystem.so.1.74 /usr/lib/libboost_filesystem.so.1.74.0
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_thread.so.1.74 /usr/lib/libboost_thread.so.1.74.0
#ssh -p 22 admin@$1 ln -sf /usr/lib/libboost_date_time.so.1.74 /usr/lib/libboost_date_time.so.1.74.0

#ssh -p 22 admin@$1 'opkg install python3-pyyaml'
#ssh -p 22 admin@$1 'opkg clean'
#ssh -p 22 admin@$1 'opkg install python3-setuptools python3-pyyaml python3-docutils python3-misc python3-xmlrpc python3-json python3-pydoc'
#ssh -p 22 admin@$1 'opkg clean'
#scp -P 22 ~/2022RobotCode/os_detect.py admin@$1:/usr/lib/python3.5/site-packages/rospkg/

# Try to simulate what the cross-build environment looks like 
# This will prevent weird bugs where sourcing install_isolated/setup.bash
#   will overwrite the settings from /opt/ros/noetic/setup.bash leading
#   to errors finding basic ROS tools
ssh -p 22 admin@$1 'mkdir -p /home/ubuntu/wpilib/2023/roborio'
ssh -p 22 admin@$1 'ln -s / /home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi'
ssh -p 22 admin@$1 'mkdir -p /home/ubuntu/wpilib/2023/roborio/arm-nilrt-linux-gnueabi'
ssh -p 22 admin@$1 'ln -s / /home/ubuntu/wpilib/2023/roborio/arm-nilrt-linux-gnueabi/sysroot'
# TODO -is this needed?
ssh -p 22 admin@$1 'ln -s /usr/include /include'

# Create workspace. Do a build in the empty workspace to set
# up various scripts for later use. TODO : See if this is needed?
ssh -p 22 admin@$1 'mkdir -p 2022RobotCode/zebROS_ws/src'
ssh -p 22 admin@$1 'source /opt/ros/noetic/setup.bash && cd 2022RobotCode/zebROS_ws && PYTHONPATH=/opt/ros/noetic/local/lib/python3.10/dist-packages catkin_make_isolated --install'

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
##################-----------------------------#################
#Edit /etc/ntp.conf to be a copy of ntpClient in 2022RobotCode#
scp -P 22 ~/2022RobotCode/scripts/RIO_setup/ntp.conf admin@$1:/etc/ntp.conf

# Copy wpilib to roborio
ssh -p 22 admin@$1 mkdir wpilib
cd ~/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/wpilib/linux/athena/shared
scp -P 22 *.so admin@$1:wpilib
cd ~/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/ctre/linux/athena/shared
scp -P 22 *.so admin@$1:wpilib
cd ~/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/rev/linux/athena/shared
scp -P 22 *.so admin@$1:wpilib
# Remove debugging versions of libraries to save space
ssh -p 22 admin@$1 rm wpilib/*d.so wpilib/*jni.so wpilib/*java*so

# Set up ssh keys
scp -P 22 ~/2022RobotCode/scripts/jetson_setup/roborio_dot_ssh.tar.bz2 admin@$1:.
ssh -p 22 admin@$1 'mkdir .ssh'
ssh -p 22 admin@$1 'cd .ssh && tar -xjf ../roborio_dot_ssh.tar.bz2'
ssh -p 22 admin@$1 'rm roborio_dot_ssh.tar.bz2'

# Edit /etc/ssh/sshd_config, uncomment Port 22, add Port 5801, 
# uncomment ChallengeResponseAuthentication and set it to no
ssh -p 22 admin@$1 "sed \"s/#Port 22/Port 22\\nPort 5801/g\" /etc/ssh/sshd_config | sed \"s/#ChallengeResponseAuthentication yes/ChallengeResponseAuthentication no/\" > sshd_config && mv sshd_config /etc/ssh"

# Restart sshd to pick up above changes
ssh -p 22 admin@$1 "/etc/init.d/sshd restart"
sleep 5

# Copy rio_bashrc.sh, ROSJetsonMaster.sh to /home/admin
scp -P 22 ~/2022RobotCode/scripts/RIO_setup/rio_bashrc.sh admin@$1:.
#scp -P 22 ~/2022RobotCode/zebROS_ws/ROSJetsonMaster.sh admin@$1:.

# Set up prereqs for deploy script
ssh -p 22 admin@$1 'mv ~/2022RobotCode ~/2022RobotCode.orig'
ssh -p 22 admin@$1 'ln -s ~/2022RobotCode.orig ~/2022RobotCode'
ssh -p 22 admin@$1 'mkdir -p ~/2022RobotCode.prod/zebROS_ws'
ssh -p 22 admin@$1 'mkdir -p ~/2022RobotCode.dev/zebROS_ws'
