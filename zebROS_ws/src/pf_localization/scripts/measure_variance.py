#!/usr/bin/env python
import sys
import math
import rospy
from geometry_msgs.msg import Twist
from goal_detection.msg import GoalDetection
import numpy as np
from scipy.cluster.vq import kmeans2,vq

dist_angle_measurements = []
avg_loc = []
stop = False

def twist_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # If any of the x, y, ang_z components are non-zero, calculate and display
    # variance of the readings from each target
    global stop
    if stop == True:
        return
    if ((data.linear.x != 0) or (data.linear.y != 0) or (data.angular.z != 0)):
        print("Non-zero cmd_vel found, done!")
        for i in range(len(avg_loc)):
            print("===========================")
            print(i)
            print(dist_angle_measurements[i])
            if (i == 0):
                all_dist_measurements = dist_angle_measurements[i]
            else:
                all_dist_measurements = np.append(all_dist_measurements, dist_angle_measurements[i], axis=0)
        stop = True

        dists = np.linalg.norm(all_dist_measurements, axis=1)
        angles = np.arctan2(all_dist_measurements[:,1], all_dist_measurements[:,0])
        print all_dist_measurements[:,1]
        print all_dist_measurements[:,0]
        print angles
        
        print ("cov", np.cov(dists, angles))
        centers, labels = kmeans2(all_dist_measurements, int(len(avg_loc)))
        print centers
        print labels

        for i in range(len(avg_loc)):
            d = dists[labels == i]
            a = angles[labels == i]
            print("len = ", len(d))
            print ("cov", np.cov(d, a))
        sys.exit(0)

def goal_detection_callback(data):
    global stop
    if stop == True:
        return
    # If time < previous time, reset data
    # Get distance, angle to each target
    # try to match to a previous detected target (delta x,y)
    # add to that list, or a new one if not matched
    #print ("dist_angle_measurements", dist_angle_measurements)
    #print ("avg_loc", avg_loc)
    for i in range(len(data.location)):
        x = data.location[i].x
        y = data.location[i].y
        #print("data.location[", i, "]", data.location[i])
        #d = math.hypot(x, y)
        #a = math.atan2(y, x)
        added = False
        for j in range(len(avg_loc)):
            #print ("dist_angle_measurements[", j,"]", dist_angle_measurements[j])
            #print("avg_loc[", j, "]", avg_loc[j])
            delta = math.hypot(x - avg_loc[j][0], y - avg_loc[j][1])
            #print("delta", delta)
            if (delta < .25):
                #print("adding to loc ", j)
                dist_angle_measurements[j] = np.append(dist_angle_measurements[j], [[x, y]], axis=0)
                avg_loc[j] = dist_angle_measurements[j].mean(axis=0)
                #print ("dist_angle_measurements[", j,"]", dist_angle_measurements[j])
                #print("avg_loc[", j, "]", avg_loc[j])
                added = True
                break
        if added == False:
            #print("Not matched")
            dist_angle_measurements.append(np.array([[x,y]]))
            avg_loc.append(np.array([x, y]))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('measure_variance', anonymous=True)

    rospy.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel", Twist, twist_callback)
    rospy.Subscriber("/goal_detect/goal_detect_msg", GoalDetection, goal_detection_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
