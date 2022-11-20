#!/usr/bin/env python3
import numpy as np
import math
import matplotlib.pyplot as plt
from threading import Lock

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped
from pf_localization.msg import PFDebug
from frc_msgs.msg import MatchSpecificData
import tf2_geometry_msgs

debug_topic = "pf_debug"
ground_truth_topic = "/base_pose_ground_truth"
match_data_topic = "/frcrobot_rio/match_data"

# From https://www.oreilly.com/library/view/python-cookbook/0596001673/ch05s19.html
class RingBuffer:
    """ class that implements a not-yet-full buffer """
    def __init__(self, size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """
        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max
        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]
        def newest(self):
            idx = self.cur - 1
            if idx < 0:
                idx += self.max
            return(self.data[idx])

    def append(self,x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data

    def newest(self):
        if len(self.data) == 0:
            return None
        return self.data[-1]

max_pose_points = 30
predicted_x = RingBuffer(max_pose_points)
predicted_y = RingBuffer(max_pose_points)
ground_truth_x = RingBuffer(max_pose_points)
ground_truth_y = RingBuffer(max_pose_points)
beacon_locations = {}
beacon_locations_blue = {}
beacon_locations_red = {}
beacon_annotations = []

plt_mutex = Lock()
current_alliance_color = 0

fig, axes = plt.subplots(nrows=1, ncols=1)
ax = axes
pln, = ax.plot([], [], 'r.', markersize=2)
dir, = ax.plot([], [], 'r-,')
ln,  = ax.plot([], [], 'b.', markersize=.15)
gtn, = ax.plot([], [], 'g.', markersize=2)
dtns = {}

def update_debug(debug_msg):
    global ln, pln, dir, dtns, predicted_x, predicted_y, beacon_locations

    x_data = []
    y_data = []
    rot_data = []
    for p in debug_msg.poses:
        x_data.append(p.position.x)
        y_data.append(p.position.y)
        angles = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]);
        rot_data.append(angles[2])

    this_x = debug_msg.predicted_pose.position.x
    this_y = debug_msg.predicted_pose.position.y
    predicted_x.append(this_x)
    predicted_y.append(this_y)

    o = debug_msg.predicted_pose.orientation
    angle = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

    detections = set()
    for o in debug_msg.beacons:
        detections.add(o)

    plt_mutex.acquire()
    for d in dtns:
        if d in detections and d in beacon_locations:
            xs = [predicted_x.newest(), beacon_locations[d][0]]
            ys = [predicted_y.newest(), beacon_locations[d][1]]
            dtns[d].set_data(xs,ys)
        else:
            dtns[d].set_data([], [])

    ln.set_data(x_data, y_data)
    pln.set_data(predicted_x.get(), predicted_y.get())
    dir.set_data([this_x, this_x + math.cos(angle)],[this_y, this_y + math.sin(angle)])
    plt.draw()
    plt_mutex.release()

def update_ground_truth(ground_truth_msg):
    global ground_truth_x, ground_truth_y

    ground_truth_x.append(ground_truth_msg.pose.pose.position.x)
    ground_truth_y.append(ground_truth_msg.pose.pose.position.y)
    plt_mutex.acquire()
    gtn.set_data(ground_truth_x.get(), ground_truth_y.get())
    plt.draw()
    plt_mutex.release()

def update_match_data(match_data_msg):
    global beacon_locations, beacon_locations_blue, beacon_locations_red, current_alliance_color, ax
    ac = match_data_msg.allianceColor
    if (ac != current_alliance_color):
        current_alliance_color = ac
        if match_data_msg.allianceColor == 0:
            beacon_locations = beacon_locations_red
        else:
            beacon_locations = beacon_locations_blue

        plt_mutex.acquire()
        for i, a in enumerate(beacon_annotations):
            a.remove()
        beacon_annotations[:] = []
        
        for id, p in beacon_locations.items():
            beacon_annotations.append(ax.annotate(id, (p[0], p[1]), textcoords="offset points", xytext=(0,10), ha='center'))

        plt.draw()
        plt_mutex.release()


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

    global beacon_locations, beacon_locations_blue, beacon_locations_red, current_alliance_color, dtns
    for b in beacons:
        beacon_x.append(b[0])
        beacon_y.append(b[1])
        # Set up a map of beacon_id -> [beacon x, beacon y] tuples
        beacon_locations_blue[b[2]] = [b[0], b[1]]

        # And a placeholder for each potential predicted_pose->beacon line
        dtns[b[2]], = ax.plot([], [], 'r-')

    # Create mirror image beacon positions for robot
    # on red alliance
    trans = TransformStamped()
    trans.header.frame_id = "blue0"
    trans.header.stamp = 0
    trans.child_frame_id = "red0"
    trans.transform.translation.x = field_dims[1]
    trans.transform.translation.y = field_dims[3]
    q = quaternion_from_euler(0, 0, math.pi)
    trans.transform.rotation = Quaternion(*q)

    for k, v in beacon_locations_blue.items():
        pose_s = PoseStamped()
        pose_s.header = trans.header
        pose_s.pose.position.x = v[0]
        pose_s.pose.position.y = v[1]
        pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
        beacon_locations_red[k] = [pose_t.pose.position.x, pose_t.pose.position.y]

    current_alliance_color = 1
    beacon_locations = beacon_locations_blue

    ax.plot(beacon_x, beacon_y, 'ko')
    for id, p in beacon_locations.items():
        beacon_annotations.append(ax.annotate(id, (p[0], p[1]), textcoords="offset points", xytext=(0,10), ha='center'))

    sub_debug = rospy.Subscriber(debug_topic, PFDebug, update_debug)
    sub_ground_truth = rospy.Subscriber(ground_truth_topic, Odometry, update_ground_truth)
    sub_match_data = rospy.Subscriber(match_data_topic, MatchSpecificData, update_match_data)

    plt.show()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
