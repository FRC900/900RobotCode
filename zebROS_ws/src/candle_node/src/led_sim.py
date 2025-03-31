#!/usr/bin/env python3

#Tasks for this
# 1. Allow selection of LED lights on robot
# 2 Animate LED light in simulation by subscribing to led states
# 3. (MAYBE): Another window to allow manual selection of led display state
# 3. Display 3d face and objects in a GUI (show 3d model)

import rospy
import numpy as np
import cv2 as cv
from candle_controller_msgs.srv import Animation, ColourRequest
from talon_state_msgs.msg import CANdleStateArray

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

class LEDSim():
    def __init__(self):
        self.img = cv.imread("/home/ubuntu/900RobotCode/zebROS_ws/src/candle_node/assets/2025_LED_Mappings_white_bg.png")
        self.img = cv.resize(self.img, (0, 0), fx=0.5, fy=0.5)
        cv.namedWindow('LED')

        self.led_state_sub = rospy.Subscriber("/frcrobot_jetson/candle_states", CANdleStateArray, self.led_state_callback)

        # rospy.Subscriber('ctre_interfaces/candle_state_interface.h', ColourRequest, Animation)

        nothing = lambda x: ()

        cv.createTrackbar('R','LED',0,255,nothing)
        cv.createTrackbar('G','LED',0,255,nothing)
        cv.createTrackbar('B','LED',0,255,nothing)

    def draw_led(self, r, g, b, w):
        cv.circle(self.img, (27,135), 25, (r,g,b), 5)

    def led_state_callback(self, msg: CANdleStateArray):
        print("called")
        for led in msg[0].leds:
            self.draw_led(led.red, led.green, led.blue, led.white)
        
        # get current positions of four trackbars
        r = cv.getTrackbarPos('R','LED')
        g = cv.getTrackbarPos('G','LED')
        b = cv.getTrackbarPos('B','LED')
        #s = cv.getTrackbarPos(switch,'LED')
            
        for i in range(0,10):
            start_point = (0+60*i,270)
            end_point = (60+60*i,330)
            color = (r,g,b)
            thickness = -1
            cv.rectangle(self.img, start_point, end_point, color, thickness)


led_sim = LEDSim()

while True:
    k = cv.waitKey(1) & 0xFF

    # Window closes on Esc
    if k == 27:
        break

    cv.imshow('LED', led_sim.img)

led_sim.led_state_sub.unregister()
cv.destroyAllWindows()