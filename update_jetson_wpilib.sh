#!/bin/bash

# Run me with IP address of the Jetson as argument
#   wpilib/2020 has to be installed on local machine - run from docker
#   env to make sure that the local build env is correct one
#   to push to the Jetson

ssh -p 22 ubuntu@$1 'rm -rf /home/ubuntu/wpilib/2020'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/ctre'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/navx'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/wpilib'
rsync -avru $HOME/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/ $1:/home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/include/

ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/ctre'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/navx'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/wpilib'
rsync -avzu --exclude '*.debug' --exclude 'athena' $HOME/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/ $1:/home/ubuntu/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/lib/
