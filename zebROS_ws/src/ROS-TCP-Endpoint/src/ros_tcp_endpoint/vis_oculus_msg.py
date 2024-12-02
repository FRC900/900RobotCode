#!/usr/bin/env python3

import rospy
import tf
from unity_robotics_demo_msgs.msg import PosRot  # Replace with your package name

# Variables to store the offset
initial_pos = None
initial_rot = None
def pos_rot_callback(msg):

    global initial_pos, initial_rot

    # Store the first message as the offset
    if initial_pos is None and initial_rot is None:
        initial_pos = (msg.pos_x, msg.pos_y, msg.pos_z)
        initial_rot = (msg.rot_x, msg.rot_y, msg.rot_z, msg.rot_w)
        rospy.loginfo("Initial offset set: position=%s, orientation=%s", initial_pos, initial_rot)
        return  # Skip publishing for the first message to avoid unnecessary transform

    pos = (
        msg.pos_x - initial_pos[0],
        msg.pos_y - initial_pos[1],
        msg.pos_z - initial_pos[2]
    )

    # For quaternion subtraction, normalize the result (since quaternions don't subtract directly)
    rot = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_inverse(initial_rot),
        (msg.rot_x, msg.rot_y, msg.rot_z, msg.rot_w)
    )
    # Broadcast the transform
    br = tf.TransformBroadcaster()
    br.sendTransform(
        pos,
        rot,
        rospy.Time.now(),
        "quest",   # Child frame (to be visualized in RViz)
        "base_link"      # Parent frame (RViz reference frame)
    )

def main():
    rospy.init_node('pos_rot_to_tf_publisher')
    
    # Subscribe to the PosRot message topic
    rospy.Subscriber('pos_rot', PosRot, pos_rot_callback)
    
    rospy.loginfo("Transform broadcaster is running...")
    rospy.spin()

if __name__ == '__main__':
    main()
