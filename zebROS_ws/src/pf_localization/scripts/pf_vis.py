#!/usr/bin/env python

import matplotlib.pyplot as plt

import rospy
from pf_localization.msg import pf_debug, pf_pose
from nav_msgs.msg import Odometry

particles_topic = "/pf_debug"
pf_pose_topic = "/predicted_pose"
ground_truth_topic = "/base_pose_ground_truth"

x_data = []
y_data = []
rot_data = []
predicted_x = []
predicted_y = []
ground_truth_x = []
ground_truth_y = []
error = []

fig, axes = plt.subplots(nrows=1, ncols=1)
ax = axes
pln, = ax.plot([], [], 'r.', markersize=2)
ln,  = ax.plot([], [], 'b.', markersize=.15)
gtn, = ax.plot([], [], 'g.', markersize=2)

def update_particles(particle_msg):
    global x_data, y_data, rot_data

    x_data = []
    y_data = []
    rot_data = []
    for p in particle_msg.particles:
        x_data.append(p.x)
        y_data.append(p.y)
        rot_data.append(p.rot)
    ln.set_data(x_data, y_data)
    plt.draw()

def update_pose(pose_msg):
    global predicted_x, predicted_y

    predicted_x.append(pose_msg.x)
    predicted_y.append(pose_msg.y)
    pln.set_data(predicted_x, predicted_y)
    plt.draw()

def update_ground_truth(ground_truth_msg):
    global ground_truth_x, ground_truth_y

    ground_truth_x.append(ground_truth_msg.pose.pose.position.x)
    ground_truth_y.append(ground_truth_msg.pose.pose.position.y)
    gtn.set_data(ground_truth_x, ground_truth_y)
    plt.draw()

def main():
    global ax

    rospy.init_node('pf_vis_node', anonymous = True)
    rospy.loginfo("pf visualization node initialized")

    beacons = []
    beacon_x = []
    beacon_y = []
    field_dims = []

    if rospy.has_param('beacons'):
        beacons = rospy.get_param('beacons')
    else:
        rospy.logerr("vis: failed to load beacons")
        return -1

    if rospy.has_param('field_dims/x_min'):
        field_dims.append(rospy.get_param('field_dims/x_min'))
    if rospy.has_param('field_dims/x_max'):
        field_dims.append(rospy.get_param('field_dims/x_max'))
    if rospy.has_param('field_dims/y_min'):
        field_dims.append(rospy.get_param('field_dims/y_min'))
    if rospy.has_param('field_dims/y_max'):
        field_dims.append(rospy.get_param('field_dims/y_max'))

    rospy.loginfo("assigned field dimensions")

    ax.set_xlim(field_dims[0], field_dims[1])
    ax.set_ylim(field_dims[2], field_dims[3])

    for b in beacons:
        beacon_x.append(b[0])
        beacon_y.append(b[1])
    ax.plot(beacon_x, beacon_y, 'ko')

    sub_particles = rospy.Subscriber(particles_topic, pf_debug, update_particles)
    sub_pose = rospy.Subscriber(pf_pose_topic, pf_pose, update_pose)
    sub_ground_truth = rospy.Subscriber(ground_truth_topic, Odometry, update_ground_truth)

    plt.show()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
