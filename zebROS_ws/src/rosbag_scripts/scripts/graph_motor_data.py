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
        talonTopic = "/frcrobot/talon_states"
        pdpTopic = "/frcrobot/pdp_states"
        
        # read Talon data
        talon_data = extract_data(bag, talonTopic, inputFileName)

        # read PDP data
        pdp_data = extract_data(bag, pdpTopic, inputFileName)

        # graph Talon speeds
        num_talons = len(talon_data[0].speed)
        subplots_dimension = ceil(sqrt(num_talons))
        print("num talons = ", num_talons)

        for talon in range(num_talons):
            talon_data_filtered = []
            for msg in talon_data:
                talon_data_filtered.append(msg.speed[talon])
            ax1 = subplot(subplots_dimension, subplots_dimension, talon + 1)
            ax1.set_title(msg.name[talon])
            ax1.plot(range(0, len(talon_data)), talon_data_filtered)

        plt.show()

        # graph PDP current for all channels and graph voltage
        num_channels = len(pdp_data[0].current)
        subplots_dimension = ceil(sqrt(num_channels))
        print("num channels = ", num_channels)

        # grid = plt.GridSpec(subplots_dimension + 1, subplots_dimension)

        for channel in range(num_channels):
            current_data_filtered = []
            for msg in pdp_data:
                current_data_filtered.append(msg.current[channel])
            ax1 = subplot(subplots_dimension, subplots_dimension, channel + 1)
            ax1.set_title("channel = %s" % (channel + 1))
            ax1.plot(range(0, len(pdp_data)), current_data_filtered)
            
        #int((channel % subplots_dimension) + 1)
        #int(floor(channel / subplots_dimension))]

        # voltage_data_filtered = []
        # for msg in pdp_data:
        #     voltage_data_filtered.append(msg.voltage)
        # ax1 = subplot(grid[subplots_dimension, 0:])
        # ax1.plot(range(0, len(pdp_data)), voltage_data_filtered)
        # ax1.set_title("voltage")

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
    topicList = []
    for topic, msg, t in bag.read_messages():
        if topicList.count(topic) == 0:
            topicList.append (topic)

    print '{0} topics found:'.format(len(topicList))
    return topicList


run()
