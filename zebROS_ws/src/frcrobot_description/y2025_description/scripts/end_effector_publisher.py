#!/usr/bin/env python3

import rospy
from talon_state_msgs.msg import TalonFXProState
import tf2_ros
from geometry_msgs.msg import TransformStamped

# rosrun tf2_ros static_transform_publisher 0 0 0.5 0 0 0 base_link end_effector_link

rospy.init_node("end_effector_publisher")

br = tf2_ros.TransformBroadcaster()

elevator_idx = None
current_position = 0

def talonfxpro_states_cb(states: TalonFXProState):
    global elevator_idx, current_position
    if elevator_idx == None:
        for i in range(len(states.name)):
            if states.name[i] == "elevator_leader":
                current_position = states.position[i]
                elevator_idx = i
                break
    else:
        current_position = states.position[elevator_idx]
    
    t = TransformStamped()
    t.header.stamp = states.header.stamp
    t.header.frame_id = "base_link"
    t.child_frame_id = "end_effector_link"
    for i in "xyz":
        exec(f"t.transform.rotation.{i} = 0")
        exec(f"t.transform.translation.{i} = 0")
    t.transform.rotation.w = 1
    t.transform.translation.z = current_position
    br.sendTransform(t)

sub = rospy.Subscriber("/frcrobot_jetson/talonfxpro_states", TalonFXProState, talonfxpro_states_cb)

rospy.spin()