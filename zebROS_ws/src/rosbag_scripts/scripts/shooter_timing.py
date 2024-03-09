#!/usr/bin/env python3
# Given a rosbag in sys.argv[1], this script will analyze the time between the align and shoot goal being sent 
# and the note leaving the shooter (preshooter switch going from 1 to 0). This time will also be broken down into:
# - time between align goal being sent and align goal being done
# - time between align goal being done and shooting goal being sent
# - time between pivot goal being sent and pivot goal being done
# - time between shooter goal being sent and shooter being up to speed
# - time between shooter being up to speed (done waiting for pivot and shooter) and note leaving the shooter

import rosbag
import sys

align_and_shoot_goal_sent = None
align_goal_sent = None
align_goal_done = None
shooting_goal_sent = None
pivot_goal_sent = None
pivot_goal_done = None
shooter_goal_sent = None
shooter_done = None

last_preshooter_switch = False

orient_setpoint = None
teleop_control = False
control_effort = None

# Format:
# total_time_in_seconds,time_to_align,time_to_pivot,time_to_shoot,time_from_spun_up_to_shot
print("total_time_in_seconds,time_to_align,time_to_pivot,time_to_shoot,time_from_spun_up_to_shot")

for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages(["/align_and_shoot/align_and_shoot_2024/goal", "/align_to_speaker/align_to_speaker_2024/goal", "/align_to_speaker/align_to_speaker_2024/feedback", "/teleop/orient_strafing/setpoint", "/teleop/orient_strafing/state", "/shooting/shooting_server_2024/goal", "/shooter/set_shooter_pivot/goal", "/shooter/set_shooter_pivot/feedback", "/shooter/set_shooter_speed/goal", "/shooter/set_shooter_speed/feedback", "/frcrobot_rio/joint_states", "/frcrobot_rio/joystick_states1", "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out"]):
    if topic == "/align_and_shoot/align_and_shoot_2024/goal":
        align_and_shoot_goal_sent = t
        align_goal_sent = None
        align_goal_done = None
        shooting_goal_sent = None
        pivot_goal_sent = None
        pivot_goal_done = None
        shooter_goal_sent = None
        shooter_done = None
    elif topic == "/align_to_speaker/align_to_speaker_2024/goal":
        print(f"{t.to_sec()}")
        align_goal_sent = t
    elif topic == "/align_to_speaker/align_to_speaker_2024/feedback":
        if msg.feedback.aligned:
            align_goal_done = t
    elif topic == "/teleop/orient_strafing/setpoint":
        orient_setpoint = msg.position
    elif topic == "/frcrobot_jetson/swerve_drive_controller/cmd_vel_out":
        control_effort = msg.twist.angular.z
    elif topic == "/frcrobot_rio/joystick_states1":
        teleop_control = abs(msg.rightStickX) > 0.05
    # elif topic == "/teleop/orient_strafing/state" and align_and_shoot_goal_sent and align_goal_sent and not align_goal_done:
        # print(f"{t.to_sec()},{orient_setpoint},{msg.data},{control_effort},{teleop_control}")
    elif topic == "/shooting/shooting_server_2024/goal" and align_goal_done:
        shooting_goal_sent = t
    elif topic == "/shooter/set_shooter_pivot/goal" and shooting_goal_sent:
        pivot_goal_sent = t
    elif topic == "/shooter/set_shooter_pivot/feedback" and pivot_goal_sent:
        if msg.feedback.is_at_pivot_position:
            pivot_goal_done = t
    elif topic == "/shooter/set_shooter_speed/goal" and shooting_goal_sent:
        shooter_goal_sent = t
    elif topic == "/shooter/set_shooter_speed/feedback" and shooter_goal_sent:
        if msg.feedback.is_shooting_at_speed:
            shooter_done = t
    elif topic == "/frcrobot_rio/joint_states":
        for i in range(len(msg.name)):
            if msg.name[i] == "preshooter_limit_switch":
                if msg.position[i] == 0 and align_and_shoot_goal_sent and last_preshooter_switch:
                    # Format:
                    # total_time_in_seconds,time_to_align,time_to_pivot,time_to_shoot,time_from_spun_up_to_shot
                    print(f"{(t - align_and_shoot_goal_sent).to_sec()},{(align_goal_done - align_goal_sent).to_sec()},{(pivot_goal_done - pivot_goal_sent).to_sec()},{(shooter_done - shooter_goal_sent).to_sec()},{(t - shooter_done).to_sec()}")
                last_preshooter_switch = msg.position[i]