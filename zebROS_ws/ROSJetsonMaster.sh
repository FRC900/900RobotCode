#!/usr/bin/env bash

# Setup ROS for Jetson Master
if [ -f /home/ubuntu/2018RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # Jetson-specific configuration
    echo "Sourcing Jetson / native Linux environment"
    source /opt/ros/kinetic/setup.bash
    source /home/ubuntu/2018RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.8
elif [ -f /home/admin/rio_bashrc.sh ] ; then
    # roboRIO-specific configuration
    echo "Sourcing roboRIO environment"
    source /home/admin/rio_bashrc.sh
    export ROS_IP=10.9.0.2
    export LD_LIBRARY_PATH=/home/admin/wpilib:$LD_LIBRARY_PATH
    swapon /dev/sda5
elif [ -f /home/ryan/2018RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # ryan-specific configuration
    echo "Sourcing ryan's environment"
    source /home/ryan/2018RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.12
elif [ -f /home/anjasheppard/2018RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # anja-specific configuration
    echo "Sourcing anja's environment"
    source /home/anjasheppard/2018RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.11
elif [ -f /home/ofugikawa/2018RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # olivia-specific configuration
    echo "Sourcing olivia's environment"
    source /home/ofugikawa/2018RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.14
elif [ -f /home/niall/2018RobotCode/zebROS_ws/devel/setup.bash ] ; then
    # niall-specific configuration
    echo "Sourcing niall's environment"
    source /home/niall/2018RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=10.9.0.13

else
    echo "Unknown environment! Trying to proceed anyway using local environment."
    source /opt/ros/kinetic/setup.bash
    source $HOME/2018RobotCode/zebROS_ws/devel/setup.bash
    export ROS_IP=`/bin/hostname -I | tr -d ' ' | tr -d '\n'`
fi

# Common configuration
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ROS_MASTER_URI=http://10.9.0.8:5802
#export ROS_IP=`/bin/hostname -I | tr -d ' ' | tr -d '\n'`
export ROSLAUNCH_SSH_UNKNOWN=1

exec "$@"
