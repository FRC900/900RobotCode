#!/bin/bash

NVP_MODEL=0
# High perf on TX is 'nvpmodel -m 0' while
# on the NX is it 'nvpmodel -m 2'
cat /proc/cpuinfo | grep "CPU part" | grep -q 0x004
if [ $? -eq 0 ]
	NVPMODEL=8
fi

for i in `seq 20`
do
    # Need to figure out model for new Jetson NX parts
	nvpmodel -m $NVPMODEL
	/usr/bin/jetson_clocks
	sleep 10
done

