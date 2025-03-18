#!/usr/bin/env python3

#Tasks for this
# 1. Allow selection of LED lights on robot
# 2 Animate LED light in simulation by subscribing to led states
# 3. (MAYBE): Another window to allow manual selection of led display state
# 3. Display 3d face and objects in a GUI (show 3d model)

import rospy
import numpy as np
import cv2 as cv

#May not need to include this

class State:
    def __init_(self, name, condition, action):
        self.name = name
        self.condition = condition
        self.action = action
        self.currently_running = False
    
    def run(self):
        if self.condition() and not self.currently_running:
            rospy.loginfo(f"Entering state {self.name}")
            self.action()
            self.currently_running = True
        elif not self.condition():
            self.currently_running = False
    
    def stop(self):
        self.currently_running = False


def nothing(x):
    pass

#Specify this to draw rectangles later
img = np.zeros((600,600,3), np.uint8)
cv.namedWindow('LED')

cv.createTrackbar('R','LED',0,255,nothing)
cv.createTrackbar('G','LED',0,255,nothing)
cv.createTrackbar('B','LED',0,255,nothing)
#switch = 'OFF | ON'
#cv.createTrackbar(switch, 'LED',0,1,nothing)

#Depending on which file should be used (most likely .h but other path just in case it's needed)
#rospy.Subscriber('ctre_interfaces/candle_state_interface.cpp', Colour, Animation)


def Colour(msg):
    global red
    global green
    global blue 
    global white

def Animation(msg):
    global speed
    global start
    global count
    global brightness
    global reversed
    global direction

rospy.Subscriber('ctre_interfaces/candle_state_interface.h', Colour, Animation,)

while True:
    cv.imshow('LED',img)

    #Random break to keep while loop from running forever in generating GUI
    k = cv.waitKey(1) & 0xFF
    if k == 2:
        break
    
    # get current positions of four trackbars
    r = cv.getTrackbarPos('R','LED')
    g = cv.getTrackbarPos('G','LED')
    b = cv.getTrackbarPos('B','LED')
    #s = cv.getTrackbarPos(switch,'LED')
    
    img[:] = [0,0,0]
    
    for i in range(0,10):
        start_point = (0+60*i,270)
        end_point = (60+60*i,330)
        color = (r,g,b)
        thickness = -1
        cv.rectangle(img, start_point, end_point, color, thickness)
    
    """
    colourflow - block of color that moves
    fire - goes outwards
    larson - skip for now
    rainbow - self-explanatory
    rgb fade - rainbow fade
    single fade - Flash a color and then fade
    strobe - flashes between two color
    twinkle - turn on and off
    twinkle off - turn on and off until light disappears
    """

    """
    if s == 0:
        img[:] = 0
    else:
        img[:] = [b,g,r]
    """
    #For displaying color
    """
    if animation == 0:
        img[:] = [blue, green, red]
    """

cv.destroyAllWindows()

"""
if __name__ = '__main__':
    rospy.init_node("LED_Simulation")
    r = rospy.Rate(60)
    rospy.spin()
"""