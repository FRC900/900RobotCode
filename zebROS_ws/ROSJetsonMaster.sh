#!/usr/bin/env bash

# Setup ROS for Jetson Master
if [ -f /.dockerenv ] ; then
    # Docker-specific configuration
    echo "Sourcing Docker environment"
    source /opt/ros/melodic/setup.bash
    source /home/ubuntu/2019Offseason/zebROS_ws/devel/setup.bash
	export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | cut -d ' ' -f 5`
elif [ -f /home/ubuntu/2019Offseason/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Jetson / native Linux environment"
    source /opt/ros/melodic/setup.bash
    source /home/ubuntu/2019Offseason/zebROS_ws/devel/setup.bash
    #export ROS_IP=10.9.0.8
	export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | cut -d ' ' -f 5`
elif [ -f /home/ofugikawa/2019Offseason/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Olivia / native Linux environment"
    source /home/ofugikawa/2019Offseason/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.14
elif [ -f /home/admin/rio_bashrc.sh ] ; then
    # roboRIO-specific configuration
    echo "Sourcing roboRIO environment"
    source /home/admin/rio_bashrc.sh
    export ROS_IP=10.9.0.2
    export LD_LIBRARY_PATH=/home/admin/wpilib:$LD_LIBRARY_PATH
    swapon /dev/sda5
else
    echo "Unknown environment! Trying to proceed anyway using local environment."
    source /opt/ros/melodic/setup.bash
    source $HOME/2019Offseason/zebROS_ws/devel/setup.bash
	export ROS_IP=`ip route get 10.9.0.1 | sed 's/ via [[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+\.[[:digit:]]\+//' | sed 's/lo //' | head -1 | cut -d ' ' -f 5`
fi

# Common configuration
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ROS_MASTER_URI=http://10.9.0.8:5802
export ROSLAUNCH_SSH_UNKNOWN=1
echo "ROS_IP set to $ROS_IP"

exec "$@"
