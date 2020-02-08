#!/bin/bash

# Reload IPv6 networking blocks
sudo sysctl -p

sudo rfkill block wifi  
sudo rfkill block bluetooth

# Allow scheduling of RT threads without cgroups
sysctl -w kernel.sched_rt_runtime_us=-1
ulimit -r unlimited

systemctl stop ntp
ntpdate 10.9.0.2
systemctl start ntp

/home/ubuntu/2020RobotCode/jetson_setup/can_up.sh

. /home/ubuntu/2020RobotCode/zebROS_ws/ROSJetsonMaster.sh
#echo 1100-1200,443,80,554,1735 > /proc/sys/net/ipv4/ip_local_reserved_ports

#echo 5800 5810 > /proc/sys/net/ipv4/ip_local_port_range
#systemctl restart networking

# TODO - this should be handled by 10-local.rules 
#sudo chmod a+rw /dev/ttyACM0
sudo umount /mnt/900_2 --lazy

export CUDA_CACHE_MAXSIZE=104857600
export CUDA_CACHE_PATH=/home/ubuntu/.nv/ComputeCache

cd /home/ubuntu/2020RobotCode/jetson_setup/

if sudo mount /dev/nvme0n1p1 /mnt/900_2; then
		date >> /home/ubuntu/mounted.txt
		echo worked >> /home/ubuntu/mounted.txt
		sudo chmod a+rw /mnt/900_2/
		roslaunch controller_node 2019_compbot_combined.launch record:=true
else
		date >> /home/ubuntu/mounted.txt
		echo did not mount >> /home/ubuntu/mounted.txt
		roslaunch controller_node 2019_compbot_combined.launch
fi

top -b > /mnt/900_2/$(date +%Y%m%d%H%M%S)_top_log.txt

# Set brightness and exposure for C920 camera to low levels
# for retro-tape detection
#v4l2-ctl -d `find /dev/v4l/by-id/ -name \*Webcam_C9\*` -c exposure_auto=1,exposure_absolute=20,brightness=5

/home/ubuntu/2020RobotCode/jetson_setup/clocks.sh &
