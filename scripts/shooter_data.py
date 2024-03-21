# Script to process bag files from matches
# It reads joystick values and generates a histogram
# of the magnitude of hypot(x,y) values
import rosbag
from rospy_message_converter import message_converter
import sys
import csv
import math

def to_match_time(teleop_start_time, start_time, this_timestamp, is_auto):
    if is_auto:
        return 15 - (this_timestamp - start_time)
    else:
        return 135 - (this_timestamp - teleop_start_time)

#Bagfile name from python script, assuming bagfile is in ../bagfiles
bagfile_name = sys.argv[1]

bag = rosbag.Bag(bagfile_name)

match_topic = '/frcrobot_rio/match_data'
joint_topic = '/frcrobot_rio/joint_states'
dist_ang_topic = '/speaker_align/dist_and_ang'
talon_states_topic = '/frcrobot_jetson/talonfxpro_states'
cmd_vel_topic = '/frcrobot_jetson/swerve_drive_controller/cmd_vel'

last_talon_states = None
last_cmd_vel = None
last_dist_ang = None
last_match = None

last_limit_state = False

start_time = None
match_end_time = None
teleop_start_time = None

#Reads through the bagfile until it sees that the robot is enabled then breaks.
#That means that the last message in each list is the first message since robot enable
for topic, msg, t in bag.read_messages([match_topic]):
    m = message_converter.convert_ros_message_to_dictionary(msg)
    this_time = t.to_sec()

    if start_time is None:
        if m['Enabled']:
            start_time = this_time
    elif teleop_start_time is None:
        if m['Enabled'] and not m['Autonomous']:
            teleop_start_time = this_time
    elif match_end_time is None:
        if not m['Enabled'] and not m['Autonomous']:
            match_end_time = this_time

if start_time is not None:

    if match_end_time is None:
        match_end_time = this_time

    print("Start time = " + str(start_time) + ", end time = " + str(match_end_time))

    # Track
    # match time
    # x vel
    # y vel
    # angular vel
    # pivot control position
    # pivot position
    # pivot speed
    # pivot desired position
    # shooter current velocity then desired velocity, in order tl tr bl br
    # distance
    # angle
    # (todo)
    # 17 - IMU angle
    # 18 - orientation command
    csv_rows = [["time", "x_vel", "y_vel", "angular_vel", "pivot_control_position", "pivot_position", "pivot_velocity", "tl_vel", "tl_control_vel", "tr_vel", "tr_control_vel", "bl_vel", "bl_control_vel", "br_vel", "br_control_vel", "dist", "ang"]]
    for topic, msg, t in bag.read_messages(topics=[match_topic, joint_topic, dist_ang_topic, talon_states_topic, cmd_vel_topic]):
        m = message_converter.convert_ros_message_to_dictionary(msg)
        this_time = t.to_sec()

        if this_time < start_time:
            continue
        if this_time >= match_end_time:
            continue

        # if topic == joystick_topic:
        #     row += [''] * 7
        #     row.append(m['leftTrigger'])
        #     row.append(m['rightTrigger'])

        # if topic == talon_states_topic:
        #     row.append(m['set_point'][11])
        #     row.append(m['speed'][11])
        #     row.append(m['output_current'][11])
        #     row.append(m['set_point'][8])
        #     row.append(m['position'][8])
        #     row.append(m['speed'][8])
        #     row.append(m['motor_output_percent'][8])

        #     if (m['set_point'][8] > 1.0) and (m['position'][8] > 1.0) and (m['set_point'][11] < 0.1) and (m['set_point'][5] < 0.2) :
        #         print(f"This is not good : {row} {m['set_point'][5]} {m['position'][5]}")

        # if topic == intake_cancel:
        #     row += [''] * 9
        #     row += ['1']

        # if topic == cmd_vel_topic:
        #     row += [''] * 10
        #     row += [f'{math.hypot(m["linear"]["x"], m["linear"]["y"])}', f"{m['angular']['z']}"]

        if topic == match_topic:
            last_match = m

        if topic == dist_ang_topic:
            last_dist_ang = m

        if topic == talon_states_topic:
            last_talon_states = m

        if topic == cmd_vel_topic:
            last_cmd_vel = m

        if topic == joint_topic:
            limit_state = m["position"][1]
            if last_limit_state and not limit_state:
                row = [to_match_time(teleop_start_time, start_time, this_time, last_match["Autonomous"])]
                row += [f'{last_cmd_vel["linear"]["x"]}', f'{last_cmd_vel["linear"]["y"]}', f"{last_cmd_vel['angular']['z']}"]
                for i in range(len(last_talon_states["name"])):
                    if last_talon_states['name'][i] == "shooter_pivot_motionmagic_joint":
                        row += [f'{last_talon_states["control_position"][i]}', f'{last_talon_states["position"][i]}',f'{last_talon_states["velocity"][i]}']
                for motor in ["top_left_shooter_joint", "top_right_shooter_joint", "bottom_left_shooter_joint", "bottom_right_shooter_joint"]:
                    for i in range(len(last_talon_states["name"])):
                        if last_talon_states['name'][i] == motor:
                            row += [f'{last_talon_states["velocity"][i]}', f'{last_talon_states["control_velocity"][i]}']
                row += [f'{last_dist_ang["distance"]}', f'{last_dist_ang["angle"]}']
                csv_rows.append(row)
            last_limit_state = limit_state 

    with open(f'{bagfile_name}_shooter.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerows(csv_rows)