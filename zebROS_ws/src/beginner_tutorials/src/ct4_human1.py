#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8
from beginner_tutorials.msg import CT4State

REPORTS = ['Invalid move.', 'Success!', 'P1 wins!', 'P2 wins!', 'Draw!']

pub = rospy.Publisher('/elias/ct4/moves', UInt8, queue_size=1)

def show_board(board):
    """Prints a text representation of the board"""
    for row in board:
        for cell in row:
            print(cell, end=' ')
        print('')
    print('')

def callback(data):
    print(REPORTS[data.report])
    if data.player == id:
        if data.report != 0:
            board = [data.row0, data.row1, data.row2, data.row3, data.row4, data.row5]
            show_board(board)
        publish()

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('ct4_human1', anonymous=True)
    rospy.Subscriber('/elias/ct4/board', CT4State, callback, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def publish():
    message = UInt8()
    message.data = int(input('Choose a column (1-7): '))
    if message.data in range(1,8):
        pub.publish(message)

id = 1
print("Running!")
listener()