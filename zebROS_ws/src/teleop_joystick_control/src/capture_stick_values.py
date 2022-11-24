#!/usr/bin/env python3

'''
Script to capture and print gamepad stick values seen while running

Press B to clear the buffer
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
from geometry_msgs.msg import Twist


class AxisData():
    def __init__(self, x : float, y : float):
        self.x = x
        self.y = y

    def __str__(self):
        return f"{self.x}, {self.y}"

    def magnitude(self):
        return math.hypot(self.x, self.y)

    def direction(self):
        return math.atan2(self.y, self.x)

axis_data_map = {}

def joystick_state_callback(data : JoystickState):
    global axis_data_map
    if data.buttonAPress:
        clear_axis_data_map()

    if data.buttonBackPress:
        dump_axis_data_map()

def clear_axis_data_map():
    rospy.loginfo("Clearing saved values")
    axis_data_map = {}

def dump_axis_data_map():
    rospy.loginfo("="*50)
    for k in sorted(axis_data_map):
        rospy.loginfo(f", {k}, {axis_data_map[k]}")

def cmd_vel_callback(data : Twist):
    x = data.linear.x
    y = data.linear.y
    if x == 0 and y == 0:
        return
    axis_data = AxisData(x, y)
    direction = axis_data.direction()

    if direction not in axis_data_map:
        axis_data_map[direction] = axis_data
    elif axis_data.magnitude() > axis_data_map[direction].magnitude():
        axis_data_map[direction] = axis_data


def publish_linear(start_coord : AxisData, end_coord : AxisData, pub : rospy.Publisher, num_steps=20):

    js = JoystickState()

    dx = (end_coord.x - start_coord.x) / float(num_steps)
    dy = (end_coord.y - start_coord.y) / float(num_steps)

    r = rospy.Rate(20)
    for i in range(num_steps):
        js.header.stamp = rospy.Time.now()
        js.leftStickX = start_coord.x + dx * i
        js.leftStickY = start_coord.y + dy * i
        pub.publish(js)
        r.sleep()
        pub.publish(js)
        r.sleep()

def generate_ideal_values():
    pub = rospy.Publisher('/frcrobot_rio/joystick_states1', JoystickState, queue_size=2)
    publish_linear(AxisData(-1,0.5), AxisData(-1,-0.5), pub)
    ##publish_linear(AxisData(-1,0.5), AxisData(-0.5,-1.0), pub)
    #publish_linear(AxisData(-0.5,-1.0), AxisData(0.5,-1), pub)
    #publish_linear(AxisData(0.5,-1), AxisData(1,-0.5), pub)
    #publish_linear(AxisData(1,0.5), AxisData(1,-0.5), pub)
    dump_axis_data_map()


def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our node so that multiple can run simultaneously.
    rospy.init_node('capture_stick_values', anonymous=True)

    #rospy.Subscriber("", JoystickState, joystick_state_callback)
    rospy.Subscriber("/teleop/swerve_drive_controller/cmd_vel", Twist, cmd_vel_callback)

    generate_ideal_values()

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    main()
