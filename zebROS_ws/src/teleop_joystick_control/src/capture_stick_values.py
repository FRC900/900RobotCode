#!/usr/bin/env python3

'''
Script to capture and print gamepad stick values seen while running

Press A to clear the buffer
Press left bumper to dump values seen since last clear

This is used to check the values reported at the limits of stick
travel.  The general idea

- move the stick to the edge of travel
- press A to clear previous results
- move the stick in a circle around the edge of travel a few times
- press left bumper to dump results

Results are comma-separated, so they should copy-paste into a 
spreadsheet with little problem.

'''
import math
import rospy
from frc_msgs.msg import JoystickState

class AxisData():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"{self.x}, {self.y}"

    def magnitude(self):
        return math.hypot(self.x, self.y)

    def direction(self):
        return math.atan2(self.y, self.x)

axis_data_map = {}

def callback(data):
    global axis_data_map
    if data.buttonAPress:
        rospy.loginfo("Clearing saved values")
        axis_data_map = {}

    x = data.leftStickX
    y = data.leftStickY
    axis_data = AxisData(x, y)
    direction = axis_data.direction()

    if direction not in axis_data_map:
        axis_data_map[direction] = axis_data
    elif axis_data.magnitude() > axis_data_map[direction].magnitude():
        axis_data_map[direction] = axis_data

    if data.bumperLeftPress:
        rospy.loginfo("="*50)
        for k in sorted(axis_data_map):
            rospy.loginfo(f", {k}, {axis_data_map[k]}")

    
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our node so that multiple can run simultaneously.
    rospy.init_node('capture_stick_values', anonymous=True)

    rospy.Subscriber("/frcrobot_rio/joystick_states1", JoystickState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
