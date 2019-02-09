#!/bin/bash

echo "starting gstreamer..."
echo "make sure the IP address has been updated properly"

gst-launch-1.0 -v v4l2src device=/dev/video0 ! 'video/x-raw,width=640,height=480' ! queue ! x264enc pass=qual quantizer=20 tune=zerolatency key-int-max=30 ! rtph264pay mtu=512 ! udpsink host=192.168.118.46 port=5000

echo "done"
