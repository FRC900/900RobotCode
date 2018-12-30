#!/bin/bash

# Run me with IP address of Rio as argument
#   Rio must be hooked up to the internet to grab packages correctly

# Set time and date on Rio to match the system
# we're installing from.  This should be close enough
# to reality to get past ssl errors from pip if the 
# date on the Rio is totally wacky
ssh -p 22 admin@$1 date -u --set="\"`date -u +"%Y.%m.%d-%T"`\""

ssh -p 22 admin@$1 'swapon /dev/sda5'

ssh -p 22 admin@$1 'opkg update'

# Split these up so the disk doesn't fill up with temp files
# Also need to install pyyaml first for some reason to avoid
# weird dependency hell issues with opkg (fixed in 2019?)
ssh -p 22 admin@$1 'opkg install python-pyyaml'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install libeigen python-dev libpython2 python-core'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install libcurl4 lz4 libboost-filesystem1.63.0 libboost-program-options1.63.0 libboost-signals1.63.0 libboost-regex1.63.0 libboost-thread1.63.0 libboost-chrono1.63.0 libboost-date-time1.63.0 libboost-atomic1.63.0'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install libbz2 libxml2 libgnutls-bin libgnutls-openssl27'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install libgnutls30 libgnutlsxx28 nettle libgmp10 libgmpxx4 libz1 cmake make'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install python-setuptools python3-docutils'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install python-pkgutil python-dateutil python-argparse python-nose'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install python-netifaces python-pip coreutils gdb i2c-tools '
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install ntp ntp-tickadj ntp-utils ntpdate rsync htop curl libusb-1.0-dev'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install gflags gflags-bash-completion libglog0 openssl'

ssh -p 22 admin@$1 'pip install catkin_pkg rospkg rosdistro vcstools rosdep wstool rosinstall rosinstall_generator defusedxml empy'

# Copy over ROS tar.bz2 file, extract to / on the Rio
scp -P 22 ~/2018Offseason/roscore_roborio_2018.tar.bz2 admin@$1:.
ssh -p 22 admin@$1 'cd / && tar -xjf ~/roscore_roborio_2018.tar.bz2'
scp -P 22 ~/2018Offseason/os_detect.py admin@$1:/usr/lib/python2.7/site-packages/rospkg/
ssh -p 22 admin@$1 'rm ~/roscore_roborio_2018.tar.bz2'

# Try to simulate what the cross-build environment looks like 
# This will prevent weird bugs were sourcing install_isolated/setup.bash
#   will overwrite the settings from /opt/ros/kinetic/setup.bash leading
#   to errors finding basic ROS tools
ssh -p 22 admin@$1 'mkdir -p /home/ubuntu/frc2019/roborio'
ssh -p 22 admin@$1 'ln -s / /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi'
# TODO -is this needed?
ssh -p 22 admin@$1 'ln -s /usr/include /include'

# Create workspace. Do a build in the empty workspace to set
# up various scripts for later use. TODO : See if this is needed?
ssh -p 22 admin@$1 'mkdir -p 2018Offseason/zebROS_ws/src'
ssh -p 22 admin@$1 'source /opt/ros/kinetic/setup.bash && cd 2018Offseason/zebROS_ws && catkin_make_isolated --install'

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
##################-----------------------------#################
#Edit /etc/ntp.conf to be a copy of ntp-server in 2018Offseason#
scp -P 22 ~/2018Offseason/ntp-server.conf admin@$1:/etc/ntp.conf

#---------------------------------------------------------------#
#               to setup RTC on the rio
#   Plug the rtc into the i2c port on the rio while unpowered
#
scp -P 22 ~/2018Offseason/rtc-bq32k.ko admin@$1:.
ssh -p 22 admin@$1 'mv rtc-bq32k.ko /lib/modules/`uname -r`/kernel'
ssh -p 22 admin@$1 'depmod'
ssh -p 22 admin@$1 'i2cdetect -y 2'
ssh -p 22 admin@$1 'echo bq32000 0x68 | tee /sys/class/i2c-adapter/i2c-2/new_device'

# Copy wpilib to roborio
ssh -p 22 admin@$1 mkdir wpilib
cd ~/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/wpilib/linux/athena/shared
scp -P 22 *.so *.so.3.? admin@$1:wpilib

scp -P 22 ~/2018Offseason/setupClock admin@$1:/etc/init.d/setupClock
ssh -p 22 admin@$1 'chmod +x /etc/init.d/setupClock'
ssh -p 22 admin@$1 'ln -sf /etc/init.d/setupClock /etc/init.d/hwclock.sh'
ssh -p 22 admin@$1 '/usr/sbin/update-rc.d -f setupClock defaults'
ssh -p 22 admin@$1 '/usr/sbin/update-rc.d -f hwclock.sh defaults'

scp -P 22 ~/2018Offseason/jetson_setup/roborio_dot_ssh.tar.bz2 admin@$1:.
ssh -p 22 admin@$1 'mkdir .ssh'
ssh -p 22 admin@$1 'cd .ssh -p 22 && tar -xjf ../roborio_dot_ssh.tar.bz2'
ssh -p 22 admin@$1 'rm roborio_dot_ssh.tar.bz2'

# Edit /etc/ssh/sshd_config, uncomment Port 22, add Port 5801, 
# uncomment ChallengeResponseAuthentication and set it to no
ssh -p 22 admin@$1 "sed \"s/#Port 22/Port 22\\nPort 5801/g\" /etc/ssh/sshd_config | sed \"s/#ChallengeResponseAuthentication yes/ChallengeResponseAuthentication no/\" > sshd_config && mv sshd_config /etc/ssh"

# Restart sshd to pick up above changes
ssh -p 22 admin@$1 "/etc/init.d/sshd restart"

# Copy rio_bashrc.sh, ROSJetsonMaster.sh to /home/admin
scp -P 22 ~/2018Offseason/rio_bashrc.sh admin@$1:.
scp -P 22 ~/2018Offseason/zebROS_ws/ROSJetsonMaster.sh admin@$1:.

# Set up prereqs for deploy script
ssh -p 22 admin@$1 'mv ~/2018Offseason ~/2018Offseason.orig'
ssh -p 22 admin@$1 'ln -s ~/2018Offseason.orig ~/2018Offseason'
ssh -p 22 admin@$1 'mkdir -p ~/2018Offseason.prod/zebROS_ws'
ssh -p 22 admin@$1 'mkdir -p ~/2018Offseason.dev/zebROS_ws'
