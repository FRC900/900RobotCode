#!/usr/bin/env bash

# Setup ROS for Jetson Master
if [ -f /.dockerenv ] ; then
    # Docker-specific configuration
    echo "Sourcing Docker environment"
    source /opt/ros/noetic/setup.bash
    source /home/ubuntu/900RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
elif [ -f /home/ubuntu/900RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Jetson / native Linux environment"
    source /opt/ros/noetic/setup.bash
    source /home/ubuntu/900RobotCode/zebROS_ws/devel/setup.bash
    #export ROS_IP=10.9.0.8
    export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
elif [ -f /home/ofugikawa/900RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Olivia / native Linux environment"
    source /home/ofugikawa/900RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.14
elif [ -f /home/admin/rio_bashrc.sh ] ; then
    # roboRIO-specific configuration
    echo "Sourcing roboRIO environment"
    source /home/admin/rio_bashrc.sh
    export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
    export ROS_IP=10.9.0.2
    export LD_LIBRARY_PATH=/home/admin/wpilib:$LD_LIBRARY_PATH
	export PYTHONPATH=/opt/ros/noetic/local/lib/python3.10/dist-packages
    swapon /dev/sda5
    ulimit -r unlimited
    /etc/init.d/nilvrt stop
    killall PhoenixDiagnosticsProgram
else
    echo "Unknown environment! Trying to proceed anyway using local environment."
    source /opt/ros/noetic/setup.bash
    source $HOME/900RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | tail -1`
fi

# Common configuration
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/dist-packages
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/local/lib/python3.10/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/ctre/linux/arm64/shared:/home/ubuntu/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/rev/linux/arm64/shared:/usr/local/lib
export ROS_MASTER_URI=http://10.9.0.2:5802
export ROSLAUNCH_SSH_UNKNOWN=1
echo "ROS_IP set to $ROS_IP"

exec "$@"
