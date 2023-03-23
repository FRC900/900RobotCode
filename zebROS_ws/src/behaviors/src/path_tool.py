#!/usr/bin/env python3
import sys
from tf2_geometry_msgs import *
import geometry_msgs.msg as gmsg
from tf.transformations import quaternion_from_euler
import tf.transformations as t
# specify points as [[x, y, rot],[x, y, rot]...]
origin = eval(sys.argv[1])
points = eval(sys.argv[2])

tfs = gmsg.TransformStamped()

tfs.header.frame_id = "map"
tfs.child_frame_id = "origin"
quat_tf = quaternion_from_euler(0, 0, origin[2])
tfs.transform.rotation = gmsg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
tfs.transform.translation.x = origin[0]
tfs.transform.translation.y = origin[1]
transform = t.concatenate_matrices(t.translation_matrix([origin[0], origin[1], 0]), t.quaternion_matrix(quat_tf))
inversed_transform = t.inverse_matrix(transform)
tfs.transform.translation = gmsg.Vector3(*t.translation_from_matrix(inversed_transform))
tfs.transform.rotation = gmsg.Quaternion(*t.quaternion_from_matrix(inversed_transform))

def qv_mult(q1, v1):
    # comment this out if v1 doesn't need to be a unit vector
    #v1 = t.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return t.quaternion_multiply(
        t.quaternion_multiply(q1, q2), 
        t.quaternion_conjugate(q1)
    )[:3]

oquat_tf = quat_tf

# nf = max(new_pose.pose.position.x, new_pose.pose.position.y)
#     unit_vector = quaternion_from_euler(new_pose.pose.position.x/nf, new_pose.pose.position.y/nf, 0)
#     q = t.quaternion_multiply(quat_tf, unit_vector)
#     print(t.euler_from_quaternion(q)[0] * nf, t.euler_from_quaternion(q)[1] * nf)

for point in points:
    quat_tf = quaternion_from_euler(0, 0, point[2])
    ps = PoseStamped(pose=gmsg.Pose(position=gmsg.Point(x=point[0],y=point[1],z=0), orientation=gmsg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])))
    ps.header.frame_id = "map"
    #print(ps)
    new_pose = tf2_geometry_msgs.do_transform_pose(ps, tfs)
    print(f"- [{round(new_pose.pose.position.x,2)}, {round(new_pose.pose.position.y,2)}, {round(t.euler_from_quaternion([new_pose.pose.orientation.x,new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w])[2], 2)}] # origin: [{origin[0]}, {origin[1]}, {origin[2]}], map-relative: [{point[0]}, {point[1]}, {point[2]}]")