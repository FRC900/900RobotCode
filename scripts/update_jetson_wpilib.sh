#!/bin/bash

# Update a jetson with the header files and libraries needed to build
# code for the robot
# Run me with IP address of the Jetson as argument
#   wpilib/2022 has to be installed on local machine - run from docker
#   env to make sure that the local build env is correct one
#   to push to the Jetson

which docker | grep -q docker
if [ $? -ne 1 ] ; then
	echo "This script must be run from inside a docker container"
	return
fi

ssh -p 22 ubuntu@$1 'rm -rf /home/ubuntu/wpilib/2022'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/include/ctre'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/include/navx'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/include/rev '
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/include/wpilib'
rsync -avru -e 'ssh -p 22' $HOME/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/include/ $1:/home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/include/

ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/lib/ctre'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/lib/navx'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/lib/rev'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/lib/wpilib'
rsync -avzu -e 'ssh -p 22' --exclude '*.debug' --exclude 'athena' $HOME/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/lib/ $1:/home/ubuntu/wpilib/2022/roborio/arm-frc2022-linux-gnueabi/lib/
