#!/bin/bash

ADDRESS=$1
/home/ubuntu/900RobotCode/scripts/jetson_setup/wait_for_ssh.sh ${ADDRESS} 5801 >> /home/ubuntu/bagfiles/mounted.txt
echo ubuntu | ssh -tt ${ADDRESS} sudo -kS systemctl stop ntp.service
#echo ubuntu | ssh -tt ${ADDRESS} sudo -kS date -s @$(date -u +"%s")
echo ubuntu | ssh -tt ${ADDRESS} sudo -kS ntpd -gqd >> /home/ubuntu/bagfiles/mounted.txt
echo ubuntu | ssh -tt ${ADDRESS} sudo -kS systemctl start ntp.service
echo ubuntu | ssh -tt ${ADDRESS} sudo -kS jetson_clocks --fan
/bin/true