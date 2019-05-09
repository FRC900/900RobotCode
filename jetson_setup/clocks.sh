#!/bin/bash

for i in `seq 20`
do
	nvpmodel -m 0
	/usr/bin/jetson_clocks
	sleep 10
done

