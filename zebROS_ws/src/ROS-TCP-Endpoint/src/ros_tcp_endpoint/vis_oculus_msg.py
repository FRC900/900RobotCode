#!/usr/bin/env python3

import rospy
import math
import tf
from unity_robotics_demo_msgs.msg import QuestPoseStamped, UnityColor
import angles
from std_srvs.srv import Empty, EmptyResponse  # Standard empty service
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_geometry_msgs
import geometry_msgs

from wpimath import geometry as wpigeo


class OculusInterface:
    def __init__(self):
        rospy.init_node("oculus_interface", anonymous=True)
        
        # Yaw offset for calibration
        self.yaw_offset: float = 0.0
        self.reset_postion: wpigeo.Pose2d = wpigeo.Pose2d(0, 0, 0)

        self.quest_euler_angles: list[float, float, float] = [0.0, 0.0, 0.0] # instantly converted to radians
        self.quest_position: list[float, float, float] = [0.0, 0.0, 0.0]
        self.quest_timestamp: float = 0
        # Whether the gyro direction is reversed (set this in your launch/config)
        #self.gyro_reversed = rospy.get_param("~gyro_reversed", False)
        
        # Subscriber to the Oculus data topic
        self.quest_topic = "/pos_rot"
        quest_sub = rospy.Subscriber(self.quest_topic, QuestPoseStamped, self.oculus_data_callback)
        rospy.wait_for_message(self.quest_topic, QuestPoseStamped)
        # Publisher for the pose
        self.reset_quest_pose = rospy.Publisher("/color", UnityColor, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Service to zero the yaw
        rospy.Service("zero_quest_odom", Empty, self.zero_position)
        rospy.Service("zero_quest_yaw", Empty, self.__zero_heading)
        
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transform_initialized = False
        rospy.Timer(rospy.Duration(0.02), self.publish_pose)

    # zeroes the quests heading (should also zero robot heading to match)
    def __zero_heading(self):
        self.yaw_offset = self.quest_euler_angles[1]
    
    # zeros quest odom to by subtracing out current pose
    def __reset_quest_odom(self):
        self.reset_postion = self.__get_oculus_pose() 
        rospy.logerr(f"Reset positon {self.reset_postion}")
        self.static_transform_initialized = True

    def publish_pose(self, event):
        """Timer callback to publish the Oculus pose."""
        if not self.static_transform_initialized:
            self.__reset_quest_odom() 
        final_pose: wpigeo.Pose2d = self.__get_oculus_pose() - self.reset_postion
        # Create a TransformStamped message
        transform_stamped = geometry_msgs.msg.TransformStamped()

        # Set the header
        transform_stamped.header.stamp = rospy.Time.now()  # current time
        transform_stamped.header.frame_id = "quest_hardware_offset"  # reference frame (could be any frame)
        transform_stamped.child_frame_id = "oculus_pose"  # the child frame

        # Set the translation (position)
        transform_stamped.transform.translation.x = final_pose.X()
        transform_stamped.transform.translation.y = final_pose.Y()
        transform_stamped.transform.translation.z = 0.0  # Assuming 2D pose, set Z to 0

        # Set the rotation (orientation) from Euler angles (yaw is the only component needed)
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, final_pose.rotation().radians())
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        # Create a TransformBroadcaster and send the transform
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        tf_broadcaster.sendTransform(transform_stamped)

    def zero_position(self, req):
        """Service callback to zero the yaw AND call recenterPlayer"""
        self.reset_quest_pose.publish(UnityColor(0,0,0,0))
        rospy.sleep(0.5)
        rospy.wait_for_message(self.quest_topic, QuestPoseStamped)
        self.__reset_quest_odom()
        rospy.loginfo("Yaw zeroed. New offset: %f", self.yaw_offset)
        return EmptyResponse()

    def oculus_data_callback(self, msg):
        """Callback to handle incoming Oculus data messages."""
        self.quest_euler_angles = list(map(math.radians, [msg.eul_x, msg.eul_y, msg.eul_z]))
        self.quest_position = [msg.pos_x, msg.pos_y, msg.pos_z]
        #rospy.loginfo(f"Received Oculus Data: {msg}")

    def get_oculus_yaw(self):
        """Return the yaw Euler angle with the offset subtracted."""
        return self.quest_euler_angles[1] - self.yaw_offset
    
    def get_heading(self):
        """Return the robot's heading in degrees, between -pi, pi"""
        yaw = self.get_oculus_yaw()
        return angles.normalize_angle(yaw)

    def get_oculus_position(self) -> wpigeo.Translation2d:
        return wpigeo.Translation2d(self.quest_position[2], -self.quest_position[0])

    def __get_oculus_pose(self) -> wpigeo.Pose2d:
        return wpigeo.Pose2d(self.get_oculus_position(), wpigeo.Rotation2d(self.get_heading()))  # Use Pose2D for simplified 2D representation

def main():
    oculus_interface = OculusInterface()
    rospy.loginfo("Oculus Interface Node Started.")
    
    rate = rospy.Rate(100)  # 10 Hz loop
    while not rospy.is_shutdown():
        #pose = oculus_interface.get_oculus_pose()
        #rospy.loginfo(f"Oculus Pose: {pose}")
        rate.sleep()

if __name__ == "__main__":
    main()
