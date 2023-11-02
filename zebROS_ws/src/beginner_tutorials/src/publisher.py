#!/usr/bin/env python3

import rospy
from beginner_tutorials.msg import ZebraData
from random import uniform

def main():
    pub = rospy.Publisher('/clara/jeremy/claremy', ZebraData, queue_size=1)
    rospy.init_node('joeren', anonymous=True)

    while not rospy.is_shutdown():
        message = ZebraData()
        message.num_stripes = int(input("# of stripes: "))
        message.has_horn = (input('Has horn? (y/n) ') == 'y')
        loc_preference = input("Location ('random' for random location, 'manual' for manual input): ")
        if loc_preference == "manual":
            message.location.position.x = float(input('x: '))
            message.location.position.y = float(input('y: '))
            message.location.position.z = float(input('z: '))
        else:
            message.location.position.x = uniform(-1000, 1000)
            message.location.position.y = uniform(-1000, 1000)
            message.location.position.z = uniform(-1000, 1000)
        message.name = input("Name: ")

        pub.publish(message)

main()