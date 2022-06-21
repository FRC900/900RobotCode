# Script to process bag files from matches
# It reads joystick values and generates a histogram
# of the magnitude of hypot(x,y) values
import rospy
import rosbag
from rospy_message_converter import message_converter
import math
import numpy as np
import os
import sys
import matplotlib.pyplot as plt

#Bagfile name from python script, assuming bagfile is in ../bagfiles
bagfile_name = sys.argv[1]

# Rename .bag.active to .bag, strip leading _ if present
filename, extension = os.path.splitext(bagfile_name)
if filename[0] == '_':
    filename = filename[1:]

if (extension == '.active') or (bagfile_name[0] == '_'):
    if os.path.isfile(filename):
        print 'Both ' + bagfile_name + ' and ' + filename + ' exist, exiting instead of overwiting on rename'
        sys.exit(-1)

    os.rename(bagfile_name, filename)
    bagfile_name = filename
    
# Open bagfile, reindex if necessary
try: 
    bag = rosbag.Bag(bagfile_name)
except rosbag.bag.ROSBagUnindexedException:
    print "Reindexing bag file " + str(bagfile_name)
    os.system("rosbag reindex " + str(bagfile_name))
    bag = rosbag.Bag(bagfile_name)

match_topic = '/frcrobot_rio/match_data'
joystick_topic = '/frcrobot_rio/joystick_states1'

teleop_start_time = None
match_end_time = None

isEnabled = False
this_time = None

#Reads through the bagfile until it sees that the robot is enabled then breaks.
#That means that the last message in each list is the first message since robot enable
for topic, msg, t in bag.read_messages([match_topic]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    this_time = t.to_sec()

    if teleop_start_time is None:
        if m['Enabled'] and not m['Autonomous']:
            teleop_start_time = this_time
    elif match_end_time is None:
        if not m['Enabled']:
            match_end_time = this_time

if teleop_start_time is not None:

    if match_end_time is None:
        match_end_time = this_time

    print("Start time = " + str(teleop_start_time) + ", end time = " + str(match_end_time))

    joystick_magnitudes = []
    for topic, msg, t in bag.read_messages([joystick_topic]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        this_time = t.to_sec()

        if this_time < teleop_start_time:
            continue
        if this_time >= match_end_time:
            continue

        joystick_magnitudes.append(math.hypot(m['leftStickX'], m['leftStickY']))

    joystick_np = np.array(joystick_magnitudes)

    hist, bin_edges = np.histogram(joystick_np)

    print(hist)
    print(bin_edges)

    # An "interface" to matplotlib.axes.Axes.hist() method
    n, bins, patches = plt.hist(x=joystick_np, bins='auto', color='#0504aa',
            alpha=0.7, rwidth=0.85)
    plt.grid(axis='y', alpha=0.75)
    plt.xlabel('Joystick hypot(x,y) magnitude')
    plt.ylabel('Count')
    plt.title(bagfile_name)
    plt.text(23, 45, r'$\mu=15, b=3$')
    maxfreq = n.max()
    # Set a clean upper y-axis limit.
    plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)
    plt.show()

