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
	arm-frc2020-linux-gnueabi-strip `find install_isolated/ -executable -type f | grep -v \.py | grep -v \.sh`

	rsync -avz --delete \
		--exclude '*~' --exclude '*.sw[op]' \
		--exclude '*.stl' --exclude '*.dae' \
		--exclude 'pixy2/documents' --exclude '*.a' \
		$ROS_CODE_LOCATION/install_isolated/ \
		$ROBORIO_ADDR:$RIO_INSTALL_LOCATION 
	if [ $? -ne 0 ] ; then
		echo "RIO RSYNC call failed"
		/bin/false
	else
		echo "Synchronization to RIO complete"
	fi
fi
