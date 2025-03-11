#!/usr/bin/env python3
import rospy
from talon_state_msgs.msg import CANdleStateArray
import numpy as np
import cv2

LED_RADIUS = 10 # pixels
LED_DIAMETER = 2 * LED_RADIUS

LED_COORDS_TOP = [(x * LED_DIAMETER + LED_RADIUS, LED_DIAMETER) for x in range(22)]
# Offset bottom LEDs by one LED diameter to match layout on robot
LED_COORDS_BOTTOM = [((x + 1) * LED_DIAMETER + LED_RADIUS, 2 * LED_DIAMETER) for x in range(21)]

CANDLE_COORDS_TOP = [(x * LED_DIAMETER + LED_RADIUS, 4 * LED_DIAMETER) for x in range(4)]
CANDLE_COORDS_BOTTOM = [(x * LED_DIAMETER + LED_RADIUS, 5 * LED_DIAMETER) for x in range(4)]

led_coords = CANDLE_COORDS_TOP + CANDLE_COORDS_BOTTOM[::-1] + LED_COORDS_BOTTOM + LED_COORDS_TOP[::-1]

def draw_leds(image, coords, colors):
    for coord, color in zip(coords, colors):
        cv2.circle(image, coord, LED_RADIUS, color, -1)
    return image

def callback(msg: CANdleStateArray):
    image = np.zeros((120, 450, 3), np.uint8)
    colors = [[l.blue, l.green, l.red] for l in msg.candles[0].leds]
    draw_leds(image, led_coords, colors)
    cv2.imshow('LEDs', image)
    cv2.waitKey(1)

def main():
    rospy.init_node('fake_leds')
    rospy.Subscriber('/frcrobot_jetson/candle_states', CANdleStateArray, callback)

    rospy.spin()

if __name__ == "__main__":
    main()