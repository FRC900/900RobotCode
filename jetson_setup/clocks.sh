#!/bin/bash

NVP_MODEL=0
# High perf on TX is 'nvpmodel -m 0' while
# on the NX is it 'nvpmodel -m 2'
cat /proc/cpuinfo | grep "CPU part" | grep -q 0x004
if [ $? -eq 0 ]
	NVPMODEL=2
fi

for i in `seq 20`
do
	nvpmodel -m 0
	/usr/bin/jetson_clocks
	sleep 10
done

