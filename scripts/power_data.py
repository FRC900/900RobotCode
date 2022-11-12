#!/usr/bin/env python3
# Script to process bag files from matches
# It reads power data and converts to csv
import csv
import os
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

#Bagfile name from command line
bagfile_name = sys.argv[1]

# Rename .bag.active to .bag, strip leading _ if present
filename, extension = os.path.splitext(bagfile_name)
if filename[0] == '_':
    filename = filename[1:]

if (extension == '.active') or (bagfile_name[0] == '_'):
    if os.path.isfile(filename):
        print(f'Both {bagfile_name} and {filename} exist, exiting instead of overwiting on rename')
        sys.exit(-1)

    os.rename(bagfile_name, filename)
    bagfile_name = filename
    
# Open bagfile, reindex if necessary
try: 
    bag = rosbag.Bag(bagfile_name)
except rosbag.bag.ROSBagUnindexedException:
    print(f"Reindexing bag file {bagfile_name}")
    os.system("rosbag reindex {bagfile_name}")
    bag = rosbag.Bag(bagfile_name)

match_topic = '/frcrobot_rio/match_data'
pdh_topic = '/frcrobot_rio/pdh_states'
power_topic = '/power_info'

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

    pdh_data = []
    for topic, msg, t in bag.read_messages([pdh_topic]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        this_time = t.to_sec()

        '''
        if this_time < teleop_start_time:
            continue
        if this_time >= match_end_time:
            continue
        '''

        this_pdh_data = []
        this_pdh_data.append(t.to_sec())
        this_pdh_data.append(m["voltage"])
        this_pdh_data.append(m["totalCurrent"])
        this_pdh_data.extend(m["current"])
        pdh_data.append(this_pdh_data)

    power_data = []
    for topic, msg, t in bag.read_messages([power_topic]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        this_time = t.to_sec()

        '''
        if this_time < teleop_start_time:
            continue
        if this_time >= match_end_time:
            continue
        '''

        this_power_data = []
        this_power_data.append(0)
        this_power_data.append(m["voltage1"])
        this_power_data.append(m["current"])
        this_power_data.append(m["coulombs"])
        this_power_data.append(m["temp"])
        power_data.append(this_power_data)

    print(len(pdh_data))
    print(len(power_data))
    print(float(len(pdh_data))/len(power_data))

    first_pdh_stamp = pdh_data[0][0]
    last_pdh_stamp = pdh_data[-1][0]
    elapsed_pdh_time = last_pdh_stamp - first_pdh_stamp
    time_per_power_message = elapsed_pdh_time / float(len(power_data))
    print(f'first_pdh_stamp = {first_pdh_stamp}, last_pdh_stamp = {last_pdh_stamp}, elapsed_pdh_time = {elapsed_pdh_time}, time_per_power_message = {time_per_power_message}')
    for i in range(len(power_data)):
        power_data[i][0] = first_pdh_stamp + i * time_per_power_message
    with open(bagfile_name + '.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=' ',
                                quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['PDH Data'])
        pdh_header = ['Timestamp', 'voltage', 'totalCurrent']
        for i in range(24):
            pdh_header.append(f"current channel {i}")
        writer.writerow(pdh_header)
        for p in pdh_data:
            writer.writerow(p)
        writer.writerow(['Power Data'])
        writer.writerow(['Interpolated Timestamp', 'Voltage1', 'Current', 'Coulombs', 'Temp'])
        for p in power_data:
            writer.writerow(p)



