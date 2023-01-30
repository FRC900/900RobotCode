#! /usr/bin/env python3

# try to simulate the odometry and position of the robot on the charging station
# was going to do pyhisics sim but as it almost always is, its not worth it. 

from collections import namedtuple
from enum import Enum
import cv2 # for visualizing
import numpy as np
import time
import math
import rospy
import std_msgs.msg
import rosgraph_msgs.msg

from sim_balance_base import *

def step(msg):
    global charging_station #sim_clock, clock_pub
    rospy.loginfo_throttle(1, f"step {charging_station.time}, angle {charging_station.angle*180/np.pi}, msg {msg.data}")
    #print(f"angle {charging_station.angle*180/np.pi}")
    charging_station.step(msg.data, TIME_STEP)
    #sim_clock.clock = rospy.Time.from_sec(charging_station.time)
    #clock_pub.publish(sim_clock)
    #pub.publish(charging_station.angle)
    charging_station.visualize()


if __name__ == "__main__":
    global charging_station, pub, sub

    charging_station = ChargingStationSim()
    #while charging_station.state != States.ON_MIDDLE_2_WHEEL:
    #    charging_station.step(0.5, TIME_STEP)
    
    i = 0
    vel = 0.5
    while True:
        i += 1
        ##print(i)
        inp = input("Enter a command: ")
        if inp.upper() == "I":
            #print("-----------Increasing velocity-------------")
            vel = -0.5
        elif inp.upper() == "O":
            vel = 0.5
        elif inp.upper() == "S":
            pass
            #charging_station.df.to_csv("charging_station.csv")
        elif inp == "L":
            vel = 0
        charging_station.step(vel, TIME_STEP)
        ##print(charging_station.left_wheel)
        ##print(charging_station.right_wheel)
        #print(charging_station.state)
        charging_station.visualize()
        time.sleep(0.01)
        if charging_station.state == States.ON_RAMP_LEFT_1_WHEEL:
            #_ = input()
            i += 0
            #print("on ramp left 1 wheel")
    