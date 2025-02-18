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
/home/ubuntu/900RobotCode/scripts/jetson_setup/can_up.sh
/home/ubuntu/900RobotCode/scripts/jetson_setup/wait_for_ntp_sync.sh >> /home/ubuntu/bagfiles/mounted.txt
date >> /home/ubuntu/bagfiles/mounted.txt
# Allow scheduling of RT threads without cgroups
sysctl -w kernel.sched_rt_runtime_us=-1
ulimit -r unlimited

# Sync time from each follower jetson to this one
JETSON_ADDR=(10.9.0.9 10.9.0.10)

for i in "${JETSON_ADDR[@]}"
do
    /home/ubuntu/900RobotCode/scripts/jetson_setup/sync_jetson_time.sh $i &
    JETSON_PROCESSES+=($!)
done

/home/ubuntu/900RobotCode/scripts/jetson_setup/wait_for_ssh.sh 10.9.0.2 5801 >> /home/ubuntu/bagfiles/mounted.txt
#ssh 10.9.0.2 /etc/init.d/ntpd stop
#ssh 10.9.0.2 date -s @$(date -u +"%s")
#ssh 10.9.0.2 /etc/init.d/ntpd start

for i in "${JETSON_PROCESSES[@]}"
do
    wait $i
done

export CUDA_CACHE_MAXSIZE=104857600
export CUDA_CACHE_PATH=/home/ubuntu/.nv/ComputeCache

echo "mounted / recording" >> /home/ubuntu/bagfiles/mounted.txt
/home/ubuntu/900RobotCode/zebROS_ws/ROSJetsonMaster.sh roslaunch controller_node 2025_compbot_combined.launch record:=true
# TODO - enable recording for comp
#record:=true

top -b > /home/ubuntu/bagfiles/$(date +%Y%m%d%H%M%S)_top_log.txt

/home/ubuntu/900RobotCode/scripts/jetson_setup/clocks.sh &