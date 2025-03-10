#!/usr/bin/env python3

code = ""

with open("/home/ubuntu/900RobotCode/zebROS_ws/src/behaviors/src/2025_align_to_reef_tagslam.py") as f:
    code = f.read()

bak = __name__

__name__ = "not main"

exec(code)

__name__ = bak

import time, sys # other imports handled by above monstrosity7

rospy.init_node("reef_position_generator")

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

if len(sys.argv) < 3:
    print("Usage: rosrun auto_node y2025_generate_reef_positions.py [tag_id] [left or right]")
    exit()

try:
    tag_id = int(sys.argv[1])
except:
    print("Usage: rosrun auto_node y2025_generate_reef_positions.py [tag_id] [left or right]")
    exit()

try:
    if sys.argv[2][0].lower() == "l":
        is_left = True
    elif sys.argv[2][0].lower() == "r":
        is_left = False
    else:
        print("Usage: rosrun auto_node y2025_generate_reef_positions.py [tag_id] [left or right]")
        exit()
except:
    print("Usage: rosrun auto_node y2025_generate_reef_positions.py [tag_id] [left or right]")
    exit()

time.sleep(1.0) # wait for tf tree to be up

print(Aligner.calc_pipe_pose(tag_id, tf_buffer, is_left).pose.position.x, Aligner.calc_pipe_pose(tag_id, tf_buffer, is_left).pose.position.y)

# for i in Aligner.RED_TAGS.values():
#     print(f"Left {i}:")
#     print(Aligner.calc_pipe_pose(i, tf_buffer, True))
#     print("\n")
#     print(f"Right {i}:")
#     print(Aligner.calc_pipe_pose(i, tf_buffer, False))
#     print("\n\n\n")

# for i in Aligner.BLUE_TAGS.values():
#     print(f"Left {i}:")
#     print(Aligner.calc_pipe_pose(i, tf_buffer, True))
#     print("\n")
#     print(f"Right {i}:")
#     print(Aligner.calc_pipe_pose(i, tf_buffer, False))
#     print("\n\n\n")