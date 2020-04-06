#!/bin/bash

# Run me with IP address of Rio as argument
#   Rio must be hooked up to the internet to grab packages correctly
# Must be run from inside docker container - this script copies wpilib
#   libraries to the Rio, and those only exist inside the container
#   matching the version of wpilib expected on the Rio

# Set time and date on Rio to match the system
# we're installing from.  This should be close enough
# to reality to get past ssl errors from pip if the 
# date on the Rio is totally wacky
ssh -p 22 admin@$1 date -u --set=\"$(date)\"

ssh -p 22 admin@$1 'swapon /dev/sda5'

ssh -p 22 admin@$1 'opkg update'

# Split these up so the disk doesn't fill up with temp files
# Also need to install pyyaml first for some reason to avoid
# weird dependency hell issues with opkg (fixed in 2019 or 2020?)
ssh -p 22 admin@$1 'opkg install python-pyyaml'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install python-dev libpython2 python-core python-logging'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install python-setuptools python-pycrypto python-pycrypto-dev'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install python-pkgutil python-dateutil python-argparse python-nose python-netifaces'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install libcurl4 lz4 libboost-filesystem1.66.0 libboost-program-options1.66.0 libboost-signals1.66.0 libboost-regex1.66.0 libboost-thread1.66.0 libboost-chrono1.66.0 libboost-date-time1.66.0 libboost-atomic1.66.0'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install libeigen libbz2 libxml2 libgnutls-bin libgnutls-openssl27'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install libgnutls30 libgnutlsxx28 nettle libgmp10 libgmpxx4 libz1 cmake make'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install python-pip coreutils i2c-tools python-psutil'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install ntp ntp-tickadj ntp-utils ntpdate rsync htop curl libusb-1.0-dev'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install gflags gflags-bash-completion libglog0 gperftools-dev openssl'
ssh -p 22 admin@$1 'opkg clean'
ssh -p 22 admin@$1 'opkg install gpgme-dev gpgme libgpg-error-dev libgpg-error0 libassuan-dev libassuan0'
ssh -p 22 admin@$1 'opkg clean'

# This installs python3 which takes up a lot of space - avoid installing it if possible
#ssh -p 22 admin@$1 'opkg install gdb'

#ssh -p 22 admin@$1 'opkg remove --autoremove python3*'
ssh -p 22 admin@$1 'ln -sf /usr/bin/python2 /usr/bin/python'

ssh -p 22 admin@$1 'pip install --upgrade pip'

ssh -p 22 admin@$1 'opkg install python-numpy'
ssh -p 22 admin@$1 'pip --no-cache-dir install catkin_pkg catkin_tools rospkg rosdistro vcstools rosdep wstool rosinstall rosinstall_generator defusedxml empy python-gnupg'
ssh -p 22 admin@$1 'rm -rf ~/.cache'
scp -P 22 ~/2020RobotCode/tflite_runtime-1.15.2-cp27-none-linux_armv7l.whl admin@$1:.
ssh -p 22 admin@$1 'pip --no-cache-dir install tflite_runtime-1.15.2-cp27-none-linux_armv7l.whl'
ssh -p 22 admin@$1 'rm tflite_runtime-1.15.2-cp27-none-linux_armv7l.whl'
ssh -p 22 admin@$1 'rm -rf ~/.cache'

# Copy over ROS tar.bz2 file, extract to / on the Rio
scp -P 22 ~/2020RobotCode/roscore_roborio.tar.bz2 admin@$1:.
ssh -p 22 admin@$1 'cd / && tar -xjf ~/roscore_roborio.tar.bz2'
ssh -p 22 admin@$1 'rm ~/roscore_roborio.tar.bz2'
#scp -P 22 ~/2020RobotCode/os_detect.py admin@$1:/usr/lib/python2.7/site-packages/rospkg/

#ssh -p 22 admin@$1 'opkg install python3-pyyaml'
#ssh -p 22 admin@$1 'opkg clean'
#ssh -p 22 admin@$1 'opkg install python3-setuptools python3-pyyaml python3-docutils python3-misc python3-xmlrpc python3-json python3-pydoc'
#ssh -p 22 admin@$1 'opkg clean'
#scp -P 22 ~/2020RobotCode/os_detect.py admin@$1:/usr/lib/python3.5/site-packages/rospkg/

# Try to simulate what the cross-build environment looks like 
# This will prevent weird bugs where sourcing install_isolated/setup.bash
#   will overwrite the settings from /opt/ros/melodic/setup.bash leading
#   to errors finding basic ROS tools
ssh -p 22 admin@$1 'mkdir -p /home/ubuntu/wpilib/2020/roborio'
ssh -p 22 admin@$1 'ln -s / /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi'
# TODO -is this needed?
ssh -p 22 admin@$1 'ln -s /usr/include /include'

# Create workspace. Do a build in the empty workspace to set
# up various scripts for later use. TODO : See if this is needed?
ssh -p 22 admin@$1 'mkdir -p 2020RobotCode/zebROS_ws/src'
ssh -p 22 admin@$1 'source /opt/ros/melodic/setup.bash && cd 2020RobotCode/zebROS_ws && catkin_make_isolated --install'

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
##################-----------------------------#################
#Edit /etc/ntp.conf to be a copy of ntp-server in 2020RobotCode#
scp -P 22 ~/2020RobotCode/ntp-server.conf admin@$1:/etc/ntp.conf

#---------------------------------------------------------------#
#               to setup RTC on the rio
#   Plug the rtc into the i2c port on the rio while unpowered
#
scp -P 22 ~/2020RobotCode/rtc-bq32k.ko admin@$1:.
ssh -p 22 admin@$1 'mv rtc-bq32k.ko /lib/modules/`uname -r`/kernel'
ssh -p 22 admin@$1 'depmod'
ssh -p 22 admin@$1 'i2cdetect -y 2'
ssh -p 22 admin@$1 'echo bq32000 0x68 | tee /sys/class/i2c-adapter/i2c-2/new_device'
ssh -p 22 admin@$1 'hwclock -w'

scp -P 22 ~/2020RobotCode/setupClock admin@$1:/etc/init.d/setupClock
ssh -p 22 admin@$1 'chmod +x /etc/init.d/setupClock'
ssh -p 22 admin@$1 'ln -sf /etc/init.d/setupClock /etc/init.d/hwclock.sh'
ssh -p 22 admin@$1 '/usr/sbin/update-rc.d -f setupClock defaults'
ssh -p 22 admin@$1 '/usr/sbin/update-rc.d -f hwclock.sh defaults'

# Copy wpilib to roborio
ssh -p 22 admin@$1 mkdir wpilib
cd ~/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/wpilib/linux/athena/shared
scp -P 22 *.so admin@$1:wpilib
# Remove debugging versions of libraries to save space
ssh -p 22 admin@$1 rm wpilib/*d.so wpilib/*jni.so

# Set up ssh keys
scp -P 22 ~/2020RobotCode/jetson_setup/roborio_dot_ssh.tar.bz2 admin@$1:.
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
scp -P 22 ~/2020RobotCode/rio_bashrc.sh admin@$1:.
scp -P 22 ~/2020RobotCode/zebROS_ws/ROSJetsonMaster.sh admin@$1:.

# Set up prereqs for deploy script
ssh -p 22 admin@$1 'mv ~/2020RobotCode ~/2020RobotCode.orig'
ssh -p 22 admin@$1 'ln -s ~/2020RobotCode.orig ~/2020RobotCode'
ssh -p 22 admin@$1 'mkdir -p ~/2020RobotCode.prod/zebROS_ws'
ssh -p 22 admin@$1 'mkdir -p ~/2020RobotCode.dev/zebROS_ws'
