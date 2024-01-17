#!/usr/bin/env python3
import rospy
import tf.transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from talon_state_msgs.msg import LatencyCompensationState
import math

rospy.init_node("odom_integrate_velocity_test_node", anonymous=True)

odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

current_odom = Odometry()

initial_theta = None

last_time = rospy.Time()

last_latency_compensation_message = LatencyCompensationState()

def odom_callback(msg: Odometry):
    global current_odom, last_time, initial_theta

    # pigeon2 is last, this isn't generic though
    theta = last_latency_compensation_message.latency_compensation_groups[0].value[-1] + last_latency_compensation_message.latency_compensation_groups[0].slope[-1] * (msg.header.stamp - last_latency_compensation_message.latency_compensation_groups[0].stamp[-1]).to_sec()

    current_odom.header = msg.header
    current_odom.header.frame_id = "odom"
    current_odom.child_frame_id = "base_link"

    # hacky fix since swerve drive state controller is a bit broken
    msg.twist.twist.linear.x = -msg.twist.twist.linear.x
    msg.twist.twist.linear.y = -msg.twist.twist.linear.y

    # Set twist covariances to 0.1*0.1
    msg.twist.covariance = [0.01 if i % 7 == 0 else 0 for i in range(36)]

    # Set pose covariances to 0.25*0.25
    msg.pose.covariance = [0.0625 if i % 7 == 0 else 0 for i in range(36)]

    current_odom.twist.twist.linear.x = msg.twist.twist.linear.x * math.cos(theta - initial_theta) - msg.twist.twist.linear.y * math.sin(theta - initial_theta)
    current_odom.twist.twist.linear.y = msg.twist.twist.linear.x * math.sin(theta - initial_theta) + msg.twist.twist.linear.y * math.cos(theta - initial_theta)

    dt = (msg.header.stamp - last_time).to_sec()
    current_odom.pose.pose.position.x += current_odom.twist.twist.linear.x * dt
    current_odom.pose.pose.position.y += current_odom.twist.twist.linear.y * dt

    current_odom.pose.pose.orientation.x = tf.transformations.quaternion_from_euler(0, 0, theta - initial_theta)[0]
    current_odom.pose.pose.orientation.y = tf.transformations.quaternion_from_euler(0, 0, theta - initial_theta)[1]
    current_odom.pose.pose.orientation.z = tf.transformations.quaternion_from_euler(0, 0, theta - initial_theta)[2]
    current_odom.pose.pose.orientation.w = tf.transformations.quaternion_from_euler(0, 0, theta - initial_theta)[3]

    last_time = msg.header.stamp

    odom_pub.publish(current_odom)

def latency_comp_callback(msg: LatencyCompensationState):
    global last_latency_compensation_message, initial_theta
    theta = msg.latency_compensation_groups[0].value[-1] # pigeon2 is last, this isn't generic though
    last_latency_compensation_message = msg
    if initial_theta is None:
        initial_theta = theta

odom_vel_sub = rospy.Subscriber("/frcrobot_jetson/swerve_drive_state_controller/odom", Odometry, odom_callback)

imu_sub = rospy.Subscriber("/frcrobot_jetson/latency_compensation_states", LatencyCompensationState, latency_comp_callback)

rospy.spin()