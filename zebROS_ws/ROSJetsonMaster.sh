#!/usr/bin/env bash

# Setup ROS for Jetson Master
if [ -f /.dockerenv ] ; then
    # Docker-specific configuration
    echo "Sourcing Docker environment"
    source /opt/ros/noetic/setup.bash
    source /home/ubuntu/2023RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
elif [ -f /home/ubuntu/2023RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Jetson / native Linux environment"
    source /opt/ros/noetic/setup.bash
    source /home/ubuntu/2023RobotCode/zebROS_ws/devel/setup.bash
    #export ROS_IP=10.9.0.8
    export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`

#    if [ $ROS_IP == "10.9.0.9" ] ; then
#        echo "Resetting time on secondary Jetson"
#        echo ubuntu | sudo -S systemctl stop ntp.service
#        echo ubuntu | sudo -S ntpd -gqx
#        echo ubuntu | sudo -S systemctl restart ntp.service
#    fi
elif [ -f /home/ofugikawa/2023RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Olivia / native Linux environment"
    source /home/ofugikawa/2023RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.14
elif [ -f /home/admin/rio_bashrc.sh ] ; then
    # roboRIO-specific configuration
    echo "Sourcing roboRIO environment"
    source /home/admin/rio_bashrc.sh
    export ROS_IP=10.9.0.2
    export LD_LIBRARY_PATH=/home/admin/wpilib:$LD_LIBRARY_PATH
    export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
    swapon /dev/sda5 > /dev/null
    ulimit -r unlimited
    /etc/init.d/nilvrt stop
    killall PhoenixDiagnosticsProgram
    # Force update to Rio time, hopefully will prevent
    # time updates while robot code is running
    # Moved to systemd startup script on master Jetson for now
    #/etc/init.d/ntpd stop
    #ntpd -gqx
    #/etc/init.d/ntpd restart
else
    echo "Unknown environment! Trying to proceed anyway using local environment."
    source /opt/ros/noetic/setup.bash
    source $HOME/2023RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
fi

# Common configuration
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/ctre/linux/arm64/shared:/home/ubuntu/wpilib/2023/roborio/arm-frc2023-linux-gnueabi/lib/rev/linux/arm64/shared:/usr/local/lib
export ROS_MASTER_URI=http://10.9.0.8:5802
export ROSLAUNCH_SSH_UNKNOWN=1
echo "ROS_IP set to $ROS_IP"

exec "$@"

if [ -f /home/admin/rio_bashrc.sh ] ; then
    /etc/init.d/ntpd restart
fi
