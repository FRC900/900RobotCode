#!/usr/bin/env python3

#Tasks for this
# 1. Display 3d face and objects in a GUI (show 3d model)
# 2. Allow selection of LED lights on robot
# 3. Animate LED light in simulation by subscribing to led states
# 4. (MAYBE): Another window to allow manual selection of led display state

#import rospy
import numpy as np
import cv2 as cv

def nothing(x):
    pass

img = np.zeros((400,600,3), np.uint8)
cv.namedWindow('LED')

cv.createTrackbar('R','LED',0,255,nothing)
cv.createTrackbar('G','LED',0,255,nothing)
cv.createTrackbar('B','LED',0,255,nothing)
switch = 'OFF | ON'
cv.createTrackbar(switch, 'LED',0,1,nothing)

#rospy.Subscriber('ctre_interfaces/candle_state_interface.h', animation, color, brightness, speed, led_callback)

"""
def led_callback(msg):
    global animation
    global brightness
    global color 
    global speed
"""

"""
start_point = (1,2)
end_point = (3,4)
color = (123,123,123)
thickness = -1
rect = cv.rectangle(start_point, end_point, color, thickness)
"""

while(1):
    cv.imshow('LED',img)

    #Random break to keep while loop from running forever in generating GUI
    k = cv.waitKey(1) & 0xFF
    if k == 2:
        break
    
    # get current positions of four trackbars
    start_point = (1,200)
    end_point = (300,400)
    color = (123,123,123)
    thickness = -1
    rect = cv.rectangle(img, start_point, end_point, color, thickness)
    r = cv.getTrackbarPos('R','LED')
    g = cv.getTrackbarPos('G','LED')
    b = cv.getTrackbarPos('B','LED')
    s = cv.getTrackbarPos(switch,'LED')
    
    if s == 0:
        img[:] = 0
    else:
        img[:] = [b,g,r]

cv.destroyAllWindows()