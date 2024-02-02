#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import beginner_tutorials.msg

import time

def done_callback(state, result):
    print(f"Final state was: {state}")
    print(f"Result: {result}")

def feedback_callback(feedback):
    print(f"Feedback: {feedback}")

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', beginner_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = beginner_tutorials.msg.FibonacciGoal()
    goal.order = int(input("Enter the order of the Fibonacci sequence: "))

    # Sends the goal to the action server.
    client.send_goal(goal, done_cb=done_callback, feedback_cb=feedback_callback)

    while not rospy.is_shutdown():
        print("Hi!")
        time.sleep(1)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        fibonacci_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")