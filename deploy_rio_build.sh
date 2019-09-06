#!/bin/bash

ROS_CODE_LOCATION=$1
INSTALL_ENV=$2
ROBORIO_ADDR=$3
RIO_INSTALL_LOCATION=$4

echo "Starting roboRIO cross build"
cd $ROS_CODE_LOCATION
./cross_build.sh
if [ $? -ne 0 ] ; then 
	# This makes sure this script has a non-zero error code
	# when returning to the calling function
	/bin/false
else
	echo "roboRIO cross build complete"
	echo "Synchronizing $INSTALL_ENV cross build to roboRIO"
	ssh $ROBORIO_ADDR "/etc/init.d/nilvrt stop"
	rsync -avz --delete \
		--exclude '*~' --exclude '*.sw[op]' \
		$ROS_CODE_LOCATION/install_isolated/ \
		$ROBORIO_ADDR:$RIO_INSTALL_LOCATION 
	if [ $? -ne 0 ] ; then
		echo "RIO RSYNC call failed"
		/bin/false
	else
		echo "Synchronization to RIO complete"
	fi
fi
