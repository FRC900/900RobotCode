#! /usr/bin/env python3
import rospy
from field_obj.msg import TFDetection, TFObject
from cuda_apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection


global pub

def depth_check_cb(msg):
    global pub
    TFdet = TFDetection()
    TFdet.header = msg.header
    detections = msg.detections
    for detection in detections:
        obj = TFObject()        
        obj.br.x = max(detection.corners[0].x, detection.corners[1].x, detection.corners[2].x, detection.corners[3].x)
        obj.br.y = max(detection.corners[0].y, detection.corners[1].y, detection.corners[2].y, detection.corners[3].y)
        obj.tl.x = min(detection.corners[0].x, detection.corners[1].x, detection.corners[2].x, detection.corners[3].x)
        obj.tl.y = min(detection.corners[0].y, detection.corners[1].y, detection.corners[2].y, detection.corners[3].y)
        obj.id = detection.id
        obj.confidence = 1
        obj.label = str(detection.id)
        TFdet.objects.append(obj)

    pub.publish(TFdet)

def main():
    global pub 
    sub_topic = "/cuda_tag_detections"
    pub_topic = "obj_detection_msg"
    rospy.init_node('check_depth', anonymous=True)

    sub = rospy.Subscriber(sub_topic, AprilTagDetectionArray, depth_check_cb)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=3)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
