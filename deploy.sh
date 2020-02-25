#!/bin/bash

#set -e
set -o pipefail

# IP addresses of the roboRIO and Jetson to deploy code on.
ROBORIO_ADDR=10.9.0.2

# This can be an array of IP address if there are multiple Jetsons
JETSON_ADDR=(10.9.0.8)

# Environment to deploy to (prod or dev).
INSTALL_ENV=dev

# Whether we're doing a build or just updating symlinks.
UPDATE_LINKS_ONLY=0

# Location of the code == location of deploy script
# Get directory where the deploy.sh script is located
# https://stackoverflow.com/questions/59895/getting-the-source-directory-of-a-bash-script-from-within
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  THIS_DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$THIS_DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
LOCAL_CLONE_LOCATION="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
ROS_CODE_LOCATION=$LOCAL_CLONE_LOCATION/zebROS_ws
RSYNC_OPTIONS="--delete"

usage() {
    echo "Usage: $0 [-d|-p]"
    exit 1
}


# Command line argument parsing.
POSITIONAL=()
while [[ $# -gt 0 ]] ; do
    key="$1"

    case $key in
    -h|--help)
        usage
        exit 1
        ;;
    -p|--prod)
        INSTALL_ENV=prod
        shift
        ;;
    -d|--dev)
        INSTALL_ENV=dev
        shift
        ;;
    -o|--one-dir-sync)
        RSYNC_OPTIONS="--delete"
        shift
        ;;
    -t|--two-way-sync)
        RSYNC_OPTIONS=""
        shift
        ;;
    -u|--update-links-only)
        UPDATE_LINKS_ONLY=1
        shift
	;;
    *) # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

# Directory paths on the Jetson and roboRIO.
RIO_CLONE_LOCATION=/home/admin/2020RobotCode
RIO_ENV_LOCATION=$RIO_CLONE_LOCATION.$INSTALL_ENV
RIO_ROS_CODE_LOCATION=$RIO_ENV_LOCATION/zebROS_ws
RIO_INSTALL_LOCATION=$RIO_ROS_CODE_LOCATION/install_isolated

JETSON_CLONE_LOCATION=/home/ubuntu/2020RobotCode
JETSON_ENV_LOCATION=$JETSON_CLONE_LOCATION.$INSTALL_ENV
JETSON_ROS_CODE_LOCATION=$JETSON_ENV_LOCATION/zebROS_ws

update_links() {
    # Update symlinks on the roboRIO and Jetson.
    ssh $ROBORIO_ADDR "rm $RIO_CLONE_LOCATION && \
        ln -s $RIO_ENV_LOCATION $RIO_CLONE_LOCATION"

	for i in "${JETSON_ADDR[@]}"
	do
		ssh $i "rm $JETSON_CLONE_LOCATION && \
			ln -s $JETSON_ENV_LOCATION $JETSON_CLONE_LOCATION"
	done
    echo "Symlinks updated."
}

check_clockdiff() {
    #read -r -a TIMEDIFF <<< `clockdiff $1`
    #if [[ ${#TIMEDIFF[@]} -ge 3 ]]; then
    #    echo "${TIMEDIFF[1]} msec diff"
    #else
    #    echo "Could not parse clockdiff output!"
    #    exit 1
    #fi
    LOCALDATE=`date +%s`
    REMOTEDATE=`ssh $1 date +%s`
    let TIMEDIFF=$LOCALDATE-$REMOTEDATE
    TIMEDIFF=${TIMEDIFF#-}
    if [[ $TIMEDIFF -ge 600 ]]; then
        REMOTE_TIME=`ssh $1 date`
        echo -e "\e[1m\e[31mError\e[0m : Clock difference greater than 10 minutes."
        echo "    Local time: `date`"
        echo "    Time on $2: $REMOTE_TIME"
        exit 1
    fi
}

update_links
if [ $UPDATE_LINKS_ONLY -ne 0 ]; then
    exit 0
fi

echo "Checking time synchronization..."
check_clockdiff "$ROBORIO_ADDR" "roboRIO"
for i in "${JETSON_ADDR[@]}"
do
	check_clockdiff "$i" "Jetson.$i"
done
echo "Time synchronized."

echo "Killing code on remotes "
for i in "${JETSON_ADDR[@]}"
do
	echo ubuntu | ssh -tt ubuntu@$i "sudo /home/ubuntu/2020RobotCode/zebROS_ws/kill_ros_.sh"
done
echo "ROS Killed on Jetson"

# Bidirectional synchronization of the selected environment.
echo "Synchronizing local changes TO $INSTALL_ENV environment."
for i in "${JETSON_ADDR[@]}"
do
	scp $ROS_CODE_LOCATION/ROSJetsonMaster.sh $i:$JETSON_ROS_CODE_LOCATION
done
scp $ROS_CODE_LOCATION/ROSJetsonMaster.sh $ROBORIO_ADDR:$RIO_ROS_CODE_LOCATION

# If two-way syncing is enabled, copy newer files from the Jetson(s)
# to the dev laptop
if [ ${#RSYNC_OPTIONS} -eq 0 ] ; then
	echo "Synchronizing remote changes FROM $INSTALL_ENV environment."
	for i in "${JETSON_ADDR[@]}"
	do
		rsync -avzru --ignore-times --exclude '.git' --exclude 'zebROS_ws/build*' \
			--exclude 'zebROS_ws/devel*' --exclude 'zebROS_ws/install*' --exclude 'zebROS_ws/logs*' \
			--exclude '*~' --exclude '*.sw[op]'  --exclude '*CMakeFiles*' \
			--exclude '*.avi' --exclude '*.exe'  --exclude 'pixy2/documents' --exclude 'build' \
			--exclude '*.zms' --exclude '*.stl' --exclude '*.dae' --exclude 'roscore_roborio.tar.bz2' \
			--exclude 'j120_hardware_dtb_l4t32-2-3-1.tbz2' --exclude 'zebROS_ws/.catkin_tools' \
			--exclude 'desmos_js' \
			$i:$JETSON_ENV_LOCATION/ $LOCAL_CLONE_LOCATION/
		if [ $? -ne 0 ]; then
			echo -e "\e[1m\e[31mERROR\e[0m : Failed to synchronize source code FROM $INSTALL_ENV on Jetson!"
			exit 1
		fi
	done
	echo "Synchronization from Jetson complete"
fi

# Copy from laptop to jetson.  Make it an actual sync - don't ignore
# newer files on the target as this causes problems when switching
# between git branches or between host laptops with different
# versions of code
for i in "${JETSON_ADDR[@]}"
do
	rsync -avzr $RSYNC_OPTIONS --ignore-times --exclude '.git' --exclude 'zebROS_ws/build*' \
		--exclude 'zebROS_ws/devel*' --exclude 'zebROS_ws/install*' --exclude 'zebROS_ws/logs*' \
		--exclude '*~' --exclude '*.sw[op]' --exclude '*CMakeFiles*' \
		--exclude '*.avi' --exclude '*.exe'  --exclude 'pixy2/documents' --exclude 'build' \
		--exclude '*.zms' --exclude '*.stl' --exclude '*.dae' --exclude 'roscore_roborio.tar.bz2' \
		--exclude 'j120_hardware_dtb_l4t32-2-3-1.tbz2' --exclude 'zebROS_ws/.catkin_tools' \
		--exclude 'desmos_js' \
		$LOCAL_CLONE_LOCATION/ $i:$JETSON_ENV_LOCATION/
	if [ $? -ne 0 ]; then
		echo -e "\e[1m\e[31mERROR\e[0m : Failed to synchronize source code TO $INSTALL_ENV on Jetson $i!"
		exit 1
	fi
done
echo "Synchronization to Jetson complete"

# Run local roboRIO cross build as one process.
# Then synchronize cross build products to roboRIO.
$LOCAL_CLONE_LOCATION/deploy_rio_build.sh $ROS_CODE_LOCATION $INSTALL_ENV $ROBORIO_ADDR $RIO_INSTALL_LOCATION &
RIO_BUILD_PROCESS=$!

# Run Jetson native build(s) as a separate process(es).
JETSON_BUILD_PROCESSES=()
for i in "${JETSON_ADDR[@]}"
do
	echo "Starting Jetson $i native build" 
	(
		ssh -XC $i terminator -T \"Jetson $i\" -x "$JETSON_CLONE_LOCATION/zebROS_ws/native_build.sh || \
		     	                        read -p 'Jetson Build FAILED - press ENTER to close window'" && \
		echo "Jetson $i native build complete"
	) &
	JETSON_BUILD_PROCESSES+=($!)
done

# Capture return code from Rio build processes
echo "Waiting for RIO_BUILD_PROCESS $RIO_BUILD_PROCESS"
wait $RIO_BUILD_PROCESS
RIO_RC=$?
echo " ... RIO_BUILD_PROCESS $RIO_BUILD_PROCESS returned $RIO_RC"

# Capture return code from Jetson build process(es)
# TODO - this doesn't actually capture the return code from
# the jetson native build, but instead from the terminator command
# which returns success if it was able to launch the command
JETSON_RCS=()
for i in "${JETSON_BUILD_PROCESSES[@]}"
do
	echo "Waiting for JETSON_BUILD_PROCESS $i"
	wait $i
	JETSON_RC=$?
	JETSON_RCS+=($JETSON_RC)
	echo " ... JETSON_BUILD_PROCESS $i returned $JETSON_RC"
done

# Print diagnostic info after all builds / deploys
# have run their course to make errors easier to see
EXIT_FAIL=0
if [ $RIO_RC -ne 0 ] ; then
	echo -e "Rio build/deploy \e[1m\e[31mFAILED\e[0m"
	EXIT_FAIL=1
fi

# JETSON_RCS will be the return code for terminator
for i in "${JETSON_RCS[@]}"
do
	if [ $i -ne 0 ] ; then
		echo -e "Jetson build/deploy \e[1m\e[31mFAILED\e[0m"
		EXIT_FAIL=1
	fi
done

# Also check the text in .native_build.status on each
# Jetson for the return code of the actual catkin build
# This is the more likely failure mode - compile errors, etc
for i in "${JETSON_ADDR[@]}"
do
	ssh -C $i cat $JETSON_ROS_CODE_LOCATION/.native_build.status | grep -q SUCCESS
	if [ $? -ne 0 ] ; then
		echo -e "Jetson native_build \e[1m\e[31mFAILED\e[0m"
		EXIT_FAIL=1
	fi
done

if [ $EXIT_FAIL -ne 0 ] ; then
	exit 1
fi

update_links
echo -e "\e[1m\e[32mFINISHED SUCCESSFULLY\e[0m"
