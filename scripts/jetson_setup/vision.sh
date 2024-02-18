#!/bin/bash

# Reload IPv6 networking blocks
sudo sysctl -p

sudo rfkill block wifi  
sudo rfkill block bluetooth

# 20.04 change - CAN doesn't autostart, bring it up manually
#sudo systemctl start systemd-networkd
#sudo systemctl enable systemd-networkd

mkdir -p /home/ubuntu/bagfiles
chmod 777 /home/ubuntu/bagfiles
chown ubuntu.ubuntu /home/ubuntu/bagfiles
echo "=============================" >> /home/ubuntu/bagfiles/mounted.txt
chmod 666 /home/ubuntu/bagfiles/mounted.txt
chown ubuntu.ubuntu /home/ubuntu/bagfiles/mounted.txt
date >> /home/ubuntu/bagfiles/mounted.txt
/home/ubuntu/2023RobotCode/scripts/jetson_setup/wait_for_ntp_sync.sh >> /home/ubuntu/bagfiles/mounted.txt
date >> /home/ubuntu/bagfiles/mounted.txt

# Allow scheduling of RT threads without cgroups
sysctl -w kernel.sched_rt_runtime_us=-1
ulimit -r unlimited

sudo systemctl restart zed_x_daemon
sudo i2cset -y -f 30 0x29 0x04 0x15 0x30 i
sudo i2cset -y -f 30 0x29 0x04 0x18 0x30 i
sudo i2cset -y -f 30 0x29 0x04 0x1B 0x30 i
sudo i2cset -y -f 30 0x29 0x04 0x1E 0x30 i
sudo systemctl restart zed_x_daemon

export CUDA_CACHE_MAXSIZE=104857600
export CUDA_CACHE_PATH=/home/ubuntu/.nv/ComputeCache

echo "mounted / recording" >> /home/ubuntu/bagfiles/mounted.txt
#/home/ubuntu/2023RobotCode/zebROS_ws/ROSJetsonMaster.sh roslaunch controller_node 2024_compbot_combined.launch
# - TODO, record bag files on vision processor? record:=true

top -b > /home/ubuntu/bagfiles/$(date +%Y%m%d%H%M%S)_top_log.txt

sudo jetson-clocks --fan

