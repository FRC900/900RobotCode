#! /usr/bin/env python3
'''
Remap from an input apriltag message (of various types) to
a TFDetection message to feed into screen to world
'''
import rospy
from field_obj.msg import TFDetection, TFObject
from apriltag_msgs.msg import ApriltagArrayStamped
#from cuda_apriltag_ros.msg import AprilTagDetectionArray as CUDAAprilTagDetectionArray
#from apriltag_msgs.msg import ApriltagPoseStamped, ApriltagArrayStamped, Apriltag

global pub

def depth_check_cb(msg):
    global pub
    tf_det = TFDetection()
    tf_det.header = msg.header
    if hasattr(msg, 'detections'):
        detections = msg.detections
    elif hasattr(msg, 'apriltags'):
        detections = msg.apriltags
    for detection in detections:
        obj = TFObject()        
        obj.br.x = max(detection.corners[0].x, detection.corners[1].x, detection.corners[2].x, detection.corners[3].x)
        obj.br.y = max(detection.corners[0].y, detection.corners[1].y, detection.corners[2].y, detection.corners[3].y)
        obj.tl.x = min(detection.corners[0].x, detection.corners[1].x, detection.corners[2].x, detection.corners[3].x)
        obj.tl.y = min(detection.corners[0].y, detection.corners[1].y, detection.corners[2].y, detection.corners[3].y)
        if type(detection.id) is int:
            obj.id = detection.id
            obj.label = str(detection.id)
        else:
            # non cuda
            obj.id = detection.id[0]
            obj.label = str(detection.id[0])
        obj.confidence = 1
        tf_det.objects.append(obj)

    pub.publish(tf_det)

def regular_main():
    global pub 
    sub_topic = "tags_in"
    pub_topic = "tag_detection_msg"
    rospy.init_node('tag_depth', anonymous=True)

    sub = rospy.Subscriber(sub_topic, ApriltagArrayStamped, depth_check_cb)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=3)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    regular_main()
