#!/bin/bash

# Reload IPv6 networking blocks
sudo sysctl -p

sudo rfkill block wifi  
sudo rfkill block bluetooth

# 20.04 change - CAN doesn't autostart, bring it up manually
sudo systemctl start systemd-networkd
sudo systemctl enable systemd-networkd

mkdir -p /home/ubuntu/bagfiles
chmod 777 /home/ubuntu/bagfiles
chown ubuntu.ubuntu /home/ubuntu/bagfiles
echo "=============================" >> /home/ubuntu/bagfiles/mounted.txt
chmod 666 /home/ubuntu/bagfiles/mounted.txt
chown ubuntu.ubuntu /home/ubuntu/bagfiles/mounted.txt
date >> /home/ubuntu/bagfiles/mounted.txt
/home/ubuntu/2023RobotCode/scripts/jetson_setup/can_up.sh
/home/ubuntu/2023RobotCode/scripts/jetson_setup/wait_for_ntp_sync.sh >> /home/ubuntu/bagfiles/mounted.txt
date >> /home/ubuntu/bagfiles/mounted.txt
# Allow scheduling of RT threads without cgroups
sysctl -w kernel.sched_rt_runtime_us=-1
ulimit -r unlimited

/home/ubuntu/2023RobotCode/scripts/jetson_setup/wait_for_ssh.sh 10.9.0.2 5801 >> /home/ubuntu/bagfiles/mounted.txt
#ssh 10.9.0.2 /etc/init.d/ntpd stop
#ssh 10.9.0.2 date -s @$(date -u +"%s")
#ssh 10.9.0.2 /etc/init.d/ntpd start

/home/ubuntu/2023RobotCode/scripts/jetson_setup/wait_for_ssh.sh 10.9.0.9 5801 >> /home/ubuntu/bagfiles/mounted.txt
echo ubuntu | ssh -tt 10.9.0.9 sudo -kS systemctl stop ntp.service
echo ubuntu | ssh -tt 10.9.0.9 sudo -kS date -s @$(date -u +"%s")
echo ubuntu | ssh -tt 10.9.0.9 sudo -kS systemctl start ntp.service

#echo 1100-1200,443,80,554,1735 > /proc/sys/net/ipv4/ip_local_reserved_ports

#echo 5800 5810 > /proc/sys/net/ipv4/ip_local_port_range
#systemctl restart networking

# TODO - this should be handled by 10-local.rules 
#sudo chmod a+rw /dev/ttyACM0
#sudo umount /mnt/900_2 --lazy

export CUDA_CACHE_MAXSIZE=104857600
export CUDA_CACHE_PATH=/home/ubuntu/.nv/ComputeCache

echo "mounted / recording" >> /home/ubuntu/bagfiles/mounted.txt
/home/ubuntu/2023RobotCode/zebROS_ws/ROSJetsonMaster.sh roslaunch controller_node 2023_compbot_combined.launch record:=true

top -b > /home/ubuntu/bagfiles/$(date +%Y%m%d%H%M%S)_top_log.txt

# Set brightness and exposure for C920 camera to low levels
# for retro-tape detection
#v4l2-ctl -d `find /dev/v4l/by-id/ -name \*Webcam_C9\*` -c exposure_auto=1,exposure_absolute=20,brightness=5

/home/ubuntu/2023RobotCode/scripts/jetson_setup/clocks.sh &

