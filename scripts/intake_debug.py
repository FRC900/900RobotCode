# Script to process bag files from matches
# It reads joystick values and generates a histogram
# of the magnitude of hypot(x,y) values
import rosbag
from rospy_message_converter import message_converter
import sys
import csv
import math

def to_match_time(teleop_start_time, this_timestamp):
    return this_timestamp - teleop_start_time + 15.

#Bagfile name from python script, assuming bagfile is in ../bagfiles
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

match_topic = '/frcrobot_rio/match_data'
joystick_topic = '/frcrobot_rio/joystick_states1'
talon_states_topic = '/frcrobot_jetson/talon_states'
intake_cancel = '/intaking/intaking_server_2023/cancel'
cmd_vel_topic = '/frcrobot_jetson/swerve_drive_controller/cmd_vel'

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

    # Track
    # 0 - match time
    # 1 - intake set point (talon states[11])
    # 2 - intake speed
    # 3 - intake current
    # 4 - 4bar set point (talon_states[8])
    # 5 - 4bar position
    # 6 - 4bar speed
    # 7 - joystick left trigger position
    # 8 - joystick right trigger position
    # 9 - intake cancel
    csv_rows = []
    for topic, msg, t in bag.read_messages(topics=[joystick_topic, talon_states_topic, intake_cancel, cmd_vel_topic]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        this_time = t.to_sec()

        if this_time < teleop_start_time:
            continue
        if this_time >= match_end_time:
            continue

        row = [to_match_time(teleop_start_time, this_time)]
        if topic == joystick_topic:
            row += [''] * 7
            row.append(m['leftTrigger'])
            row.append(m['rightTrigger'])

        if topic == talon_states_topic:
            row.append(m['set_point'][11])
            row.append(m['speed'][11])
            row.append(m['output_current'][11])
            row.append(m['set_point'][8])
            row.append(m['position'][8])
            row.append(m['speed'][8])
            row.append(m['motor_output_percent'][8])

            if (m['set_point'][8] > 1.0) and (m['position'][8] > 1.0) and (m['set_point'][11] < 0.1) and (m['set_point'][5] < 0.2) :
                print(f"This is not good : {row} {m['set_point'][5]} {m['position'][5]}")

        if topic == intake_cancel:
            row += [''] * 9
            row += ['1']

        if topic == cmd_vel_topic:
            row += [''] * 10
            row += [f'{math.hypot(m["linear"]["x"], m["linear"]["y"])}', f"{m['angular']['z']}"]
        
        csv_rows.append(row)

    with open('intake.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerows(csv_rows)

