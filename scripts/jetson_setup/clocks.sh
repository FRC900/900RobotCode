#!/bin/bash

NVPMODEL=0
# High perf on TX is 'nvpmodel -m 0' while
# on the Xavier NX is it 'nvpmodel -m 8'
# The orin NX is 'nvpmodel -m 0'
cat /proc/cpuinfo | grep "CPU part" | grep -q 0x004
if [ $? -eq 0 ]; then
    NVPMODEL=8
fi

nvpmodel -m $NVPMODEL
/usr/bin/jetson_clocks
/usr/bin/jetson_clocks --fan
