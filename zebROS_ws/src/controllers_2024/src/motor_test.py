# not the actual shooter controller, but probably the fastest way to get running

import rospy
import std_msgs.msg


percent = 0.5
speed = 420.0

def percent_cb(msg):
    global percent
    percent = msg.data
    rospy.loginfo(f"Set percent to {percent}")

# takes in speed in rad/s and percent from 0 to 1 
def speed_cb(msg):
    global left_pub, right_pub, percent
    speed = msg.data
    left_speed = speed + speed * percent
    right_speed = speed - speed * percent
    rospy.loginfo(f"Left speed {left_speed}, Right speed {right_speed}, percent {percent}")
    left_pub.publish(std_msgs.msg.Float64(left_speed))
    right_pub.publish(std_msgs.msg.Float64(right_speed))

    
# left speeed 630.0, Right speed 210.0

rospy.init_node("motor_test")
left_pub = rospy.Publisher("/frcrobot_jetson/left_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)
right_pub = rospy.Publisher("/frcrobot_jetson/right_shooter_voltage_velocity_controller/command", std_msgs.msg.Float64, queue_size=1)


rospy.Subscriber("/shooter_speed", std_msgs.msg.Float64, speed_cb)
rospy.Subscriber("/shooter_percent", std_msgs.msg.Float64, percent_cb)
rospy.spin()