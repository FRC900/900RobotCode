import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from field_obj.msg import Detection
from message_filters import ApproximateTimeSynchronizer, Subscriber
import statistics

TAG_ID = 2 # change based on what to test
MOVING_AVG_LEN = 5
moving_avg_tags = []
moving_avg_screen = []

x_diffs = []
y_diffs = []
z_diffs = []

PATH = "/home/ubuntu/2023RobotCode/zebROS_ws/src/tf_object_detection/src/apriltags_testing.txt"
def save(_):
    rospy.logwarn("SAVING RESULTS")
    
    with open(PATH, "w") as f:
        f.write(str(x_diffs))
        f.write("\n")
        f.write(str(y_diffs))
        f.write("\n")
        f.write(str(z_diffs))

    rospy.loginfo(f"Std Dev of X {statistics.stdev(x_diffs)}, avg {statistics.mean(x_diffs)}")
    rospy.loginfo(f"Std Dev of y {statistics.stdev(y_diffs)}, avg {statistics.mean(y_diffs)}")
    rospy.loginfo(f"Std Dev of z {statistics.stdev(z_diffs)}, avg {statistics.mean(z_diffs)}")


def calc_avg():
    # rospy.loginfo(f"{len(moving_avg_screen), len(moving_avg_tags), MOVING_AVG_LEN}")
    assert len(moving_avg_screen) == len(moving_avg_tags) == MOVING_AVG_LEN
    tags_x_avg = 0
    tags_y_avg = 0
    tags_z_avg = 0
    for point in moving_avg_tags:
        tags_x_avg += point.x
        tags_y_avg += point.y
        tags_z_avg += point.z

    tags_z_avg /= MOVING_AVG_LEN
    tags_x_avg /= MOVING_AVG_LEN
    tags_y_avg /= MOVING_AVG_LEN

    screen_x_avg = 0
    screen_y_avg = 0
    screen_z_avg = 0
    for point in moving_avg_screen:
        screen_x_avg += -point.y # INTENTIONAL
        screen_y_avg += -point.z # INTENTIONAL
        screen_z_avg += point.x # INTENTIONAL
    
    screen_z_avg /= MOVING_AVG_LEN
    screen_x_avg /= MOVING_AVG_LEN
    screen_y_avg /= MOVING_AVG_LEN
    
    # to CM
    x_diffs.append(abs(screen_x_avg - tags_x_avg) * 100)
    y_diffs.append(abs(screen_y_avg - tags_y_avg) * 100)
    z_diffs.append(abs(screen_z_avg - tags_z_avg) * 100)

    
# apriltag_ros/AprilTagDetectionArray, field_obj/Detection

def compare_apriltags_CB(apriltag_msg, world_msg):
    #rospy.logerr_throttle(0.5, "Callback with ATS") 
    #rospy.loginfo_throttle(2, apriltag_msg)
    #rospy.loginfo_throttle(2, world_msg)

    for detection in apriltag_msg.detections:
        if detection.id[0] != TAG_ID:
            rospy.logerr(f"Skipping tag with id {detection.id[0]}")
            continue
        position = detection.pose.pose.pose.position
        #rospy.loginfo_throttle(0.5, position)
        moving_avg_tags.append(position)
        if len(moving_avg_tags) > MOVING_AVG_LEN:
            moving_avg_tags.pop(0)

    for detection in world_msg.objects:
        if int(detection.id) != TAG_ID:
            rospy.logerr(f"Skipping tag with id {detection.id}")
            continue
        position = detection.location
        #rospy.loginfo_throttle(0.5, position)
        moving_avg_screen.append(position)
        if len(moving_avg_screen) > MOVING_AVG_LEN:
            moving_avg_screen.pop(0)

    if len(moving_avg_screen) == MOVING_AVG_LEN:
        calc_avg()

    

    
rospy.init_node("apriltag_testing")
apriltag_sub = Subscriber("/tag_detections", AprilTagDetectionArray)
screen_to_world_sub = Subscriber("/tf_object_detection/tag_detection_world", Detection)

ats = ApproximateTimeSynchronizer([apriltag_sub, screen_to_world_sub], queue_size=5, slop=0.1)
ats.registerCallback(compare_apriltags_CB)
rospy.Timer(rospy.Duration(2), save)

rospy.spin()
