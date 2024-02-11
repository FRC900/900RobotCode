#!/usr/bin/env python3

import rospy
import geometry_msgs
from frc_msgs.msg import MatchSpecificData
import tf2_ros

rospy.init_node('map_chooser', anonymous=True) #Initializes the node

map_broadcaster = tf2_ros.StaticTransformBroadcaster() #defines the broadcaster as map_broadcaster
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "blue_map"



t.child_frame_id = "red_map"
t.transform.translation.x = 16.541
t.transform.translation.y = 0
t.transform.translation.z = 0
t.transform.rotation.x = 1
t.transform.rotation.y = 0
t.transform.rotation.z = 0
t.transform.rotation.w = 0

map_broadcaster.sendTransform(t) #sends the transform

def callback(msg):
    #rospy.loginfo("callback")
    #rotates and moves the map if the aliance color is blue
    if msg.allianceColor == 1:
        t.header.frame_id = "map"
        t.child_frame_id = "blue_map"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        




    #rotates and moves the map if the alliance color is red
    elif msg.allianceColor == 0:
        t.header.frame_id = "map"
        t.child_frame_id = "red_map"
        t.transform.translation.x = 16.541
        t.transform.translation.y = 0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 1
        t.transform.rotation.z = 0
        t.transform.rotation.w = 0
    


    map_broadcaster.sendTransform(t) #sends the transform

rospy.Subscriber('/frcrobot_rio/match_data', MatchSpecificData, callback) #subscribes to MatchSpecificData
rospy.spin()