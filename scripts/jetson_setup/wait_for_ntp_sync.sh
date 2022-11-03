#!/bin/bash
TIMEOUT=25 #seconds

until ntpstat; do
	echo "Waiting for NTP time sync, previous rc = $?"
	TIMEOUT=$((TIMEOUT-1))
    if [ $TIMEOUT -eq 0 ]; then
        echo "Timed out waiting for NTP time sync"
        exit 0  # Don't flag an error since we want to run robot code regardless
    fi
	sleep 1
done
