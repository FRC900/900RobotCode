#!/usr/bin/env python3

import rospy
from beginner_tutorials.msg import ZebraData

zebra1 = None
zebra2 = None
merged = None

def b_callback(msg):
    global zebra1
    global zebra2

    if not zebra1:
        zebra1 = msg
    else:
        zebra2 = msg

engbools = {True: "yes", False: "no"}

def m_callback(merged):
    print('')
    print(f'{zebra1.name} + {zebra2.name} = {merged.name}')
    print(f"Stripes: {zebra1.num_stripes} | {zebra2.num_stripes} --> {merged.num_stripes}")
    print(f"Horn:  {engbools[zebra1.has_horn]} | {engbools[zebra1.has_horn]} --> {engbools[merged.has_horn]}")
    print(f"Location: ({zebra1.location.position.x}, {zebra1.location.position.y}, {zebra1.location.position.z})", end=" | ")
    print(f"({zebra2.location.position.x}, {zebra2.location.position.y}, {zebra2.location.position.z})", end=" --> ")
    print(f"({merged.location.position.x}, {merged.location.position.y}, {merged.location.position.z})")

b_sub = rospy.Subscriber("/clara/jeremy/claremy", ZebraData, b_callback, queue_size=1)
m_sub = rospy.Subscriber("/clara/jeremy/belt", ZebraData, m_callback, queue_size=1)
rospy.init_node("roenjeoull", anonymous=True)

rospy.spin()