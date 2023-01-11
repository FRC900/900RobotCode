#!/usr/bin/env python3
import csv
import rospy
from sensor_msgs.msg import Imu
from field_obj.msg import Detection
from tf.transformations import euler_from_quaternion

yaw = 0
def imu_callback(msg : Imu):
    global yaw

    o = msg.orientation
    yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

frame_number = 0
def detection_callback(data : Detection):

    global frame_number
    global csv_writer
    global yaw
    frame_number += 1
    for o in data.objects:
        row = [data.header.stamp, frame_number]
        row.append(o.location.x)
        row.append(o.location.y)
        row.append(o.angle)
        row.append(o.id)
        row.append(o.confidence)
        row.append(yaw)

        csv_writer.writerow(row)


def listener():
    global frame_number
    global csv_writer

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('log_detections', anonymous=True)

    rospy.Subscriber("/imu/zeroed_imu", Imu, imu_callback)
    rospy.Subscriber("/tf_object_detection/object_detection_world", Detection, detection_callback)
    csvfile = open('log.csv', 'w')
    csv_writer = csv.writer(csvfile, delimiter=',')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


