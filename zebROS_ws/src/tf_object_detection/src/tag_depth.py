#! /usr/bin/env python3
import rospy
from field_obj.msg import TFDetection, TFObject
#from cuda_apriltag_ros.msg import AprilTagDetectionArray as CUDAAprilTagDetectionArray
#from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from apriltag_msgs.msg import ApriltagArrayStamped

global pub

def depth_check_cb(msg):
    global pub
    TFdet = TFDetection()
    TFdet.header = msg.header
    detections = msg.apriltags
    for detection in detections:
        obj = TFObject()        
        obj.br.x = max(detection.corners[0].x, detection.corners[1].x, detection.corners[2].x, detection.corners[3].x)
        obj.br.y = max(detection.corners[0].y, detection.corners[1].y, detection.corners[2].y, detection.corners[3].y)
        obj.tl.x = min(detection.corners[0].x, detection.corners[1].x, detection.corners[2].x, detection.corners[3].x)
        obj.tl.y = min(detection.corners[0].y, detection.corners[1].y, detection.corners[2].y, detection.corners[3].y)
        if type(detection.id) is int:
            obj.id = detection.id
        else:
            # non cuda
            obj.id = detection.id[0]
        obj.confidence = 1
        if type(detection.id) is int:
            obj.label = str(detection.id)
        else:
            obj.label = str(detection.id[0])
        TFdet.objects.append(obj)

    pub.publish(TFdet)

# def cuda_main():
#     global pub 
#     sub_topic = "/cuda_tag_detections"
#     pub_topic = "tag_detection_msg"
#     rospy.init_node('tag_depth', anonymous=True)

#     sub = rospy.Subscriber(sub_topic, CUDAAprilTagDetectionArray, depth_check_cb)
#     pub = rospy.Publisher(pub_topic, TFDetection, queue_size=3)

#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")

def regular_main():
    global pub 
    sub_topic = "/apriltag_detection/tags"
    pub_topic = "tag_detection_msg"
    rospy.init_node('tag_depth', anonymous=True)

    sub = rospy.Subscriber(sub_topic, ApriltagArrayStamped, depth_check_cb)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=3)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

cuda = False

if __name__ == "__main__":
    # if cuda:
    #     cuda_main()
    # else:
        regular_main()
