#!/bin/bash

ROS_CODE_LOCATION=$1
INSTALL_ENV=$2
ROBORIO_ADDR=$3
RIO_INSTALL_LOCATION=$4

echo "Starting roboRIO cross build"
cd $ROS_CODE_LOCATION 
./cross_build.sh
if [ $? -ne 0 ] ; then 
	echo "roboRIO cross build failed"
	exit 1
fi

echo "roboRIO cross build complete"
echo "Synchronizing $INSTALL_ENV cross build to roboRIO"
ssh $ROBORIO_ADDR "/etc/init.d/nilvrt stop"
rsync -avz --delete \
	--exclude '*~' --exclude '*.sw[op]' \
	$ROS_CODE_LOCATION/install_isolated/ \
	$ROBORIO_ADDR:$RIO_INSTALL_LOCATION 
echo "Synchronization of the RIO complete"
exit 0
