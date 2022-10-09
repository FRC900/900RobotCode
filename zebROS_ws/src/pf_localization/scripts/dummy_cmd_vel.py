#!/usr/bin/env python3
#
# Publish a 0,0,0 cmd_vel message with a current timestamp
# Used to test pf on live cameras when not mounted to a robot
import rospy
from geometry_msgs.msg import TwistStamped

def talker():
    rospy.init_node('dummy_cmd_vel', anonymous=True)
    pub = rospy.Publisher('/frcrobot_jetson/swerve_drive_controller/cmd_vel_out', TwistStamped, queue_size=2)
    rate = rospy.Rate(100)
    ts = TwistStamped()
    while not rospy.is_shutdown():
        ts.header.stamp = rospy.Time(rospy.get_time())
        pub.publish(ts)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
