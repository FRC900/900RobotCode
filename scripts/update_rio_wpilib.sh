#!/bin/bash

# Update a jetson with the header files and libraries needed to build
# code for the robot
# Run me with IP address of the Jetson as argument
#   wpilib/2025 has to be installed on local machine - run from docker
#   env to make sure that the local build env is correct one
#   to push to the Jetson

which docker | grep -q docker
if [ $? -ne 1 ] ; then
	echo "This script must be run from inside a docker container"
	return
fi

ssh -p 22 admin@$1 mkdir wpilib
cd ~/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/wpilib/linux/athena/shared
scp -P 22 *.so admin@$1:wpilib
cd ~/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/ctre/linux/athena/shared
scp -P 22 *.so admin@$1:wpilib
cd ~/wpilib/2025/roborio/arm-frc2024-linux-gnueabi/lib/rev/linux/athena/shared
scp -P 22 *.so admin@$1:wpilib