#! /usr/bin/env python3

# Ordering imports by length is cool
import os
import rospy
import rospkg
from baseYOLO import YOLO900
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from field_obj.msg import TFDetection, TFObject

bridge = CvBridge()
pub, pub_debug = None, None
min_confidence = 0.1
global OBJ_DET_SRC_DIR, DETECTRON

rospack = rospkg.RosPack()
OBJ_DET_SRC_DIR = os.path.join(rospack.get_path('tf_object_detection'), 'src/')

'''
Use this to capture images which don't have tags in them
(good for finding images in which known objects aren't detected correctly)
last_write_time = 0
SECONDS_PER_WRITE = 5
FILE_PREFIX = '/home/ubuntu/tags/tags_1080p_36h11_'
'''

frame_counter = 0
# // all caps to show its important
DETECTRON: YOLO900 = None

def run_inference_for_single_image(msg):
    DETECTRON.t.start("callback")
    global min_confidence
    # rospy.logwarn("Callback recived!")
    debug = False
    if pub_debug.get_num_connections() > 0:
        debug = True
    
    DETECTRON.t.start("imgmsg_to_cv2")
    ori = bridge.imgmsg_to_cv2(msg, "bgr8")
    DETECTRON.t.end("imgmsg_to_cv2")
    
    # type hinting is really nice here
    # also looks so much nicer having the logic somewhere else from the ros code
    detections = DETECTRON.gpu_preprocess(ori, debug=debug).infer()

    if debug:
        rospy.logwarn_throttle(3, "Publishing debug image for object detect")
        debug_image = DETECTRON.draw_bboxes()    
        pub_debug.publish(bridge.cv2_to_imgmsg(debug_image, encoding="bgr8"))

    DETECTRON.t.start("pub")
    detection = TFDetection()
    detection.header = msg.header

    d_bboxes, d_scores, d_labels = detections.bboxes, detections.scores, detections.labels  
    for (bbox, score, label) in zip(d_bboxes, d_scores, d_labels):
        if score <= min_confidence:
            continue

        bbox = bbox.round().int().tolist() # [x1, y1, x2, y2] where x1 y1 is top left corner and x2 y2 bottom right
        cls_id = int(label)
        obj = TFObject()
        obj.confidence = score
        height, width = ori.shape[:2]
        obj.tl.x = max(bbox[0], 0)
        obj.tl.y = max(bbox[1], 0) 
        obj.br.x = min(bbox[2], width)
        obj.br.y = min(bbox[3], height)
        obj.id = cls_id # number
        obj.label = DETECTRON.name_from_cls_id(cls_id) # string
        detection.objects.append(obj)

    pub.publish(detection)
    DETECTRON.t.end("pub")
    global frame_counter
    frame_counter += 1
    if frame_counter >= 3600:
        print(f'{DETECTRON.t}')
        frame_counter = 0

    DETECTRON.t.end("callback")
    

def main():
    global pub, pub_debug, min_confidence, DETECTRON

    os.chdir(OBJ_DET_SRC_DIR)
    DETECTRON = YOLO900(use_timings=False)

    sub_topic = "/obj_detection/c920/rect_image"
    pub_topic = "obj_detection_msg"
    rospy.init_node('tf_object_detection', anonymous=True)
    min_confidence = 0.1

    if rospy.has_param('~min_confidence'):
        min_confidence = rospy.get_param('~min_confidence')
        rospy.loginfo("Min confidence of " + str(min_confidence) + " loaded from config")
    else:
        rospy.logwarn("Unable to get min confidence, defaulting to 0.1")

    if rospy.has_param('~image_topic'):
        sub_topic = rospy.get_param('~image_topic')
        rospy.loginfo("Image topic of " + str(sub_topic) + " loaded from config")
    else:
        rospy.logwarn("Unable to get image topic, defaulting to c920/rect_image")

    sub = rospy.Subscriber(sub_topic, Image, run_inference_for_single_image, queue_size=1)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=1)
    pub_debug = rospy.Publisher("debug_image", Image, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()