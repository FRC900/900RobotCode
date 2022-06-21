#!/bin/bash

source /opt/ros/noetic/setup.bash
rosnode kill -a
killall -9 rosmaster
sleep 5
