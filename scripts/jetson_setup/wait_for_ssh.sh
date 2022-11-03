#!/bin/bash
# From https://serverfault.com/questions/152795/linux-command-to-wait-for-a-ssh-server-to-be-up
HOST=$1
PORT=$2
#HOST="localhost"
#PORT=8022
if [ -z "$1" ]
  then
    echo "Missing argument for host."
    exit 1 
fi

if [ -z "$2" ]
  then
    echo "Missing argument for port."
    exit 1 
fi
echo "polling to see that host is up and ready"
RESULT=1 # 0 upon success
TIMEOUT=60 # number of iterations (5 minutes?)
while :; do 
    echo "waiting for server $1:$2 ping ..."
    # https://serverfault.com/questions/152795/linux-command-to-wait-for-a-ssh-server-to-be-up
    # https://unix.stackexchange.com/questions/6809/how-can-i-check-that-a-remote-computer-is-online-for-ssh-script-access
    # https://stackoverflow.com/questions/1405324/how-to-create-a-bash-script-to-check-the-ssh-connection
    status=$(ssh -o BatchMode=yes -o ConnectTimeout=5 ${HOST} -p ${PORT} echo ok 2>&1)
    RESULT=$?
    if [ $RESULT -eq 0 ]; then
        # this is not really expected unless a key lets you log in
        echo "connected ok"
        break
    fi
    if [ $RESULT -eq 255 ]; then
        # connection refused also gets you here
        if [[ $status == *"Permission denied"* ]] ; then
            # permission denied indicates the ssh link is okay
            echo "server response found"
            break
        fi
    fi
    TIMEOUT=$((TIMEOUT-1))
    if [ $TIMEOUT -eq 0 ]; then
        echo "timed out"
        # error for jenkins to see
        exit 1 
    fi
    sleep 5
done
