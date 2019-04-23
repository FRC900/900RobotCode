#!/usr/bin/python

# Copyright 2010 Ankur Sinha 
# Author: Ankur Sinha <sanjay DOT ankur AT gmail DOT com> 
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# File : extractRawInfo.py
#

import rosbag
import sys
import os
import json
import pickle
from rospy_message_converter import json_message_converter
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from pylab import *
from tf import transformations

#quaternion = (0, 0, 0, 1);
#euler = transformations.euler_from_quaternion(quaternion)


# Global variable for input file name

def run():
    """
    Main run method. Calls other helper methods to get work done
    """

    if len(sys.argv) != 2:
        sys.stderr.write('[ERROR] This script only takes input bag file as argument.\n')
    else:
        inputFileName = sys.argv[1]
        print "[OK] Found bag: %s" % inputFileName

        bag = rosbag.Bag(inputFileName)
        topicList = readBagTopicList(bag)

        # figure out which topic is talon_states and which is pdp_states
        imuTopic = "/frcrobot_rio/navx_mxp"
        
        # read Talon data
        imu_data = extract_data(bag, imuTopic , inputFileName)

        imu_data_filtered = []
        roll = 0
        pitch = 0
        yaw = 0
        for msg in imu_data:
            quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
            euler = transformations.euler_from_quaternion(quaternion)
            if (euler[2] == euler[2]):
                imu_data_filtered.append(euler[2])

        #plt.set_title(msg.name[talon])
        plt.plot(range(0, 20000), imu_data_filtered[0:20000])

        plt.show()

        bag.close()

def extract_data (bag, topic, inputFileName):
    print "printing %s" % topic
    data_list = []
    
    for index, (topic, msg, t) in enumerate(bag.read_messages(topics=topic)):
        data_list.append(msg)

    return data_list

def readBagTopicList(bag):
    """
    Read and save the initial topic list from bag
    """
    print "[OK] Reading topics in this bag. Can take a while.."

    # TODO: There has got to be a faster way to do this
    topicList = []
    for topic, msg, t in bag.read_messages():
        if topicList.count(topic) == 0:
            topicList.append (topic)

    print '{0} topics found:'.format(len(topicList))
    return topicList


run()
