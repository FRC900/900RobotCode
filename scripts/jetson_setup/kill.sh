#!/bin/bash

source /opt/ros/melodic/setup.bash
rosnode kill -a
killall -9 rosmaster
sleep 5
