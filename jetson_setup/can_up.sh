#!/bin/bash

log=/mnt/900_2/$(date +%Y%m%d%H%M%S)_gs_usb_log.txt

echo "Running can_up.sh" > $log

until $(ip addr | grep -q can0)
do
	echo "Can0 not found" >> $log
	ip addr >> $log
	echo "-----------------------" >> $log
	lsusb -v >> $log
	echo "-----------------------" >> $log
	lsmod >> $log
	echo "-----------------------" >> $log
	dmesg >> $log
	echo "-----------------------" >> $log
	sleep 1
	rmmod gs_usb >> $log
	sleep 1
	insmod /lib/modules/`uname -r`/kernel/drivers/net/can/usb/gs_usb.ko >> $log
done

echo "Can0 successfully loaded" >> $log
