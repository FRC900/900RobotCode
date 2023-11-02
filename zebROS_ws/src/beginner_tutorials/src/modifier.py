#!/usr/bin/env python3

import rospy
from beginner_tutorials.msg import ZebraData
from random import choice
from string import ascii_lowercase, ascii_uppercase
from math import floor

def avg(a, b):
    return floor(a + b) / 2

def merge_letters(a, b):
    if a in ascii_lowercase:
        a_num = ascii_lowercase.index(a)
    else:
        a_num = ascii_uppercase.index(a)
    if b in ascii_lowercase:
        b_num = ascii_lowercase.index(b)
    else:
        b_num = ascii_uppercase.index(b)
    return floor(avg(a_num, b_num))

def merge_zebras(zebra1, zebra2):
    """Not associative."""
    merged = ZebraData()
    merged.num_stripes = floor(avg(zebra1.num_stripes, zebra2.num_stripes))

    if zebra1.has_horn == zebra2.has_horn:
        merged.has_horn = zebra1.has_horn
    else:
        merged.has_horn = choice([True, False])

    merged.location.position.x = avg(zebra1.location.position.x, zebra2.location.position.x)
    merged.location.position.y = avg(zebra1.location.position.y, zebra2.location.position.y)
    merged.location.position.z = avg(zebra1.location.position.z, zebra2.location.position.z)

    name = ascii_uppercase[merge_letters(zebra1.name[0], zebra2.name[0])]

    if len(zebra1.name) == len(zebra2.name):
        for i in range(1, len(zebra1.name)):
            name += ascii_lowercase[merge_letters(zebra1.name[i], zebra2.name[i])]
    elif len(zebra1.name) > len(zebra2.name):
        for i in range(1, len(zebra1.name)):
            name += ascii_lowercase[merge_letters(zebra1.name[i], zebra2.name[i])]
        name += zebra1.name[len(zebra2.name):]
    else:
        for i in range(1, len(zebra1.name)):
            name += ascii_lowercase[merge_letters(zebra1.name[i], zebra2.name[i])]
        name += zebra2.name[len(zebra1.name):]

    merged.name = name
    return merged


pub = rospy.Publisher('/clara/jeremy/belt', ZebraData, queue_size=1)

merge_ready = False
first_messge = ""
compare_ready = False

def callback(message):
    global compare_ready
    global first_message

    if compare_ready:
        merged = merge_zebras(first_message, message)
        compare_ready = False
        pub.publish(merged)
    else:
        first_message = message
        compare_ready = True

sub = rospy.Subscriber("/clara/jeremy/claremy", ZebraData, callback, queue_size=1)

rospy.init_node('eyjafjallajokull', anonymous=True)
rospy.spin()