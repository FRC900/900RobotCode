#!/usr/bin/env python3

import rospy
import tf
from unity_robotics_demo_msgs.msg import PosRot  # Replace with your package name
from geometry_msgs.msg import PointStamped  # For publishing 3D points
from geometry_msgs.msg import Point  # For publishing 3D points
import math

# Variables to store the offset
initial_pos = None
initial_rot = None

class Quest3s:

    def __init__(self) -> None:
        self.__yaw_offset = 0
        self.latest_quest_msg: PosRot = None
        self.point_pub = rospy.Publisher('quest_position', PointStamped, queue_size=10)

        # Subscribe to the PosRot message topic
        rospy.Subscriber('pos_rot', PosRot, pos_rot_callback)

    def get_euler_angles(self):

    def zeroHeading():
        float[] eulerAngles = questEulerAngles.get()
        yaw_offset = eulerAngles[1]

    def get_oculus_position(self):
        """
        Computes a 2D point from Oculus/Quest 3D position.

        Args:
            quest_position (list): A list or array with [x, y, z] coordinates.

        Returns:
            Point: A geometry_msgs/Point representing the computed 2D position.
        """
        # https://github.com/juchong/swerve-testbed-bot-java/blob/fd0c3a5b40bd2c75c9b1088d68736093e2a7b855/src/main/java/frc/robot/subsystems/DriveSubsystem.java#L278
        oculus_x = quest_position[0]
        oculus_z = quest_position[2]

        # this is just a direct copy of what wpilib java does with these numbers to make a translation2d
        # Calculate radial distance in the XZ plane
        distance = math.sqrt(oculus_x ** 2 + oculus_z ** 2)

        # Calculate the angle and convert it to a 2D equivalent
        angle = math.atan2(-oculus_x, oculus_z)  # atan2 handles quadrant information

        # Create a geometry_msgs Point where x is radial distance and y is angle
        point = Point()
        point.x = distance * math.cos(angle)
        point.y = distance * math.sin(angle)
        point.z = 0.0  # Z is unused for a 2D point

        return point


    def pos_rot_callback(self, msg: PosRot):
        self.latest_quest_msg = msg

def main():
    global point_pub

    rospy.init_node('pos_rot_to_tf_publisher')
    
    rospy.loginfo("Transform broadcaster is running...")
    rospy.spin()

if __name__ == '__main__':
    main()
