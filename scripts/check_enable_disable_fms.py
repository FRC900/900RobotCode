#!/usr/bin/env python3
import rospy
import rosbag
from rospy_message_converter import message_converter
import sys

bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

match_topic = '/frcrobot_rio/match_data'
talon = '/frcrobot_jetson/talon_states'

last_enabled = False
last_fms = False
last_ds = False

for topic, msg, t in bag.read_messages([match_topic]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    this_time = t.to_sec()

    enabled = m['Enabled']
    if enabled != last_enabled:
        print(f"Match number is # {m['matchNumber']}. Now {'enabled' if enabled else 'disabled'} at timestamp {this_time}, time left: {m['matchTimeRemaining']}")
        last_enabled = enabled
    
    fms = m['FMSAttached']
    if fms != last_fms:
        print(f"FMS now {'attached' if fms else 'detached'} at timestamp {this_time}, time left: {m['matchTimeRemaining']}")
        last_fms = fms

    ds = m['DSAttached']
    if ds != last_ds:
        print(f"DS now {'attached' if ds else 'detached'} at timestamp {this_time}, time left: {m['matchTimeRemaining']}")
        last_ds = ds

last_talon_enabled = 0
last_elev = False
for topic, msg, t in bag.read_messages([talon]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    this_time = t.to_sec()

    enabled = False if m['talon_mode'][0] == 'Disabled' else True
    if enabled != last_talon_enabled:
        print(f"Now motor 0 {'enabled' if enabled else 'disabled'} at timestamp {this_time}, time left: {this_time}")
        last_talon_enabled = enabled
    
    elev = False
    for mo in m['output_current']:
        if mo < 0.05:
            elev = False or elev
        else:
            elev = True
    if elev != last_elev:
        print(f"Now motors {'move' if elev else 'zero'} at timestamp {this_time}, time left: {this_time}")
        last_elev = elev