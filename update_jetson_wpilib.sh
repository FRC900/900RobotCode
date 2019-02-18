#!/bin/bash

# Run me with IP address of the Jetson as argument
#   frc2019 has to be installed on local machine - run from docker
#   env to make sure that the local build env is correct one
#   to push to the Jetson

ssh -p 22 ubuntu@$1 'rm -rf /home/ubuntu/frc2019'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/ctre'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/navx'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/wpilib'
rsync -avzru $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/include/ $1:/home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/include/

ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ctre'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/navx'
ssh -p 22 ubuntu@$1 'mkdir -p /home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/wpilib'
rsync -avrzu --exclude '*.debug' --exclude 'athena' $HOME/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/ $1:/home/ubuntu/frc2019/roborio/arm-frc2019-linux-gnueabi/lib/
