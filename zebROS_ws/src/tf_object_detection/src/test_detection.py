#!/usr/bin/env python

import cv2
import numpy as np
from os.path import join
import rospkg
import tensorflow as tf

import rospy
from sensor_msgs.msg import Image
from field_obj.msg import TFDetection, TFObject
from cv_bridge import CvBridge, CvBridgeError

import uuid

bridge = CvBridge()

category_index, detection_graph, sess, pub, pub_debug, vis = None, None, None, None, None, None
min_confidence = 0.1

# This is needed since the modules are in a subdir of
# the python script
# These are 'borrowed' from the tensorflow models object
# detection directory
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
#from object_detection.utils import visualization_utils as vis_util
from visualization import BBoxVisualization

# Takes a image, and using the tensorflow session and graph
# provided, runs inference on the image. This returns a list
# of detections - each includes the object bounding box, type
# and confidence
def run_inference_for_single_image(msg):
  image_np = bridge.imgmsg_to_cv2(msg, "rgb8")
  try:
    image = np.expand_dims(image_np, axis=0)
  except CvBridgeError as e:
    print(e)

  with detection_graph.as_default():
    # Get handles to input and output tensors
    ops = tf.compat.v1.get_default_graph().get_operations()
    all_tensor_names = {output.name for op in ops for output in op.outputs}
    tensor_dict = {}
    for key in [
        'num_detections', 'detection_boxes', 'detection_scores',
        'detection_classes', 'detection_masks'
    ]:
      tensor_name = key + ':0'
      if tensor_name in all_tensor_names:
        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
            tensor_name)
    if 'detection_masks' in tensor_dict:
      # The following processing is only for single image
      detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
      detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
      # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
      real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
      detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
      detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
      detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
          detection_masks, detection_boxes, image.shape[1], image.shape[2])
      detection_masks_reframed = tf.cast(
          tf.greater(detection_masks_reframed, 0.5), tf.uint8)
      # Follow the convention by adding back the batch dimension
      tensor_dict['detection_masks'] = tf.expand_dims(
          detection_masks_reframed, 0)
    image_tensor = tf.compat.v1.get_default_graph().get_tensor_by_name('image_tensor:0')

    # Run inference
    output_dict = sess.run(tensor_dict,
                           feed_dict={image_tensor: image})

    # all outputs are float32 numpy arrays, so convert types as appropriate
    output_dict['num_detections'] = int(output_dict['num_detections'][0])
    output_dict['detection_classes'] = output_dict[
        'detection_classes'][0].astype(np.int64)
    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
    output_dict['detection_scores'] = output_dict['detection_scores'][0]
    if 'detection_masks' in output_dict:
      output_dict['detection_masks'] = output_dict['detection_masks'][0]

    detection = TFDetection()
    detection.header = msg.header
    for i in range(output_dict['num_detections']):
        obj = TFObject()
        obj.confidence = output_dict['detection_scores'][i]
        if obj.confidence < min_confidence:
            continue
        obj.tl.x = output_dict['detection_boxes'][i][1] * image.shape[2]
        obj.tl.y = output_dict['detection_boxes'][i][0] * image.shape[1]
        obj.br.x = output_dict['detection_boxes'][i][3] * image.shape[2]
        obj.br.y = output_dict['detection_boxes'][i][2] * image.shape[1]
        obj.id = output_dict['detection_classes'][i]
        obj.label = str(category_index.get(output_dict['detection_classes'][i])['name'])
        detection.objects.append(obj)

    pub.publish(detection)

    #hard_neg_mine(output_dict, image_np)
    #mine_undetected_power_cells(output_dict, image_np)

    visualize(output_dict, image_np)

def visualize(output_dict, image_np):
    if pub_debug.get_num_connections() > 0:
        '''
        Much slower version using tf vis_util
        vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                output_dict['detection_boxes'],
                output_dict['detection_classes'],
                output_dict['detection_scores'],
                category_index,
                instance_masks=output_dict.get('detection_masks'),
                use_normalized_coordinates=True,
                line_thickness=4,
                max_boxes_to_draw=50,
                min_score_thresh=min_confidence,
                groundtruth_box_visualization_color='yellow')
        '''
        num_detections = output_dict['num_detections']
        vis.draw_bboxes(image_np,
                        output_dict['detection_boxes'][:num_detections],
                        output_dict['detection_scores'][:num_detections],
                        output_dict['detection_classes'][:num_detections],
                        min_confidence)
        pub_debug.publish(bridge.cv2_to_imgmsg(image_np, encoding="rgb8"))


# In cases where a known number of power cells are visible in an image
# we can auto-detect cases where the wrong number of cells are detected
# save those images to use as examples for the next round of training
def mine_undetected_power_cells(output_dict, image_np):
    save_file = False
    if output_dict['num_detections'] == 0:
        save_file = True
    else:
        for i in range(output_dict['num_detections']):
            if str(category_index.get(output_dict['detection_classes'][i])['name']) != "power_cell":
                if float(output_dict['detection_scores'][i]) > min_confidence:
                    rospy.loginfo(str(category_index.get(output_dict['detection_classes'][i])['name']))
                    rospy.loginfo("Hard negative")
                    save_file = True
                    break
 
            elif float(output_dict['detection_scores'][i]) < min_confidence:
                rospy.loginfo("Missing power cell")
                save_file = True
                break

    if save_file:
        filename = 'powercell_' + str(uuid.uuid4()) + '.png'
        rospy.loginfo("saving " + filename)
        cv2.imwrite(filename, cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR))


# If objects are detected in videos which have none of the objects
# actually showing up, the detection code failed. Save those images
# to feed back into the next round of training
def hard_neg_mine(output_dict, image_np):
    for i in range(output_dict['num_detections']):
        obj = TFObject()
        obj.confidence = output_dict['detection_scores'][i]
        if obj.confidence >= min_confidence:
            filename = 'hard_neg_' + str(uuid.uuid4()) + '.png'
            rospy.loginfo("saving " + filename)
            cv2.imwrite(filename, cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR))
            continue


def main():
    global detection_graph, sess, pub, category_index, pub_debug, min_confidence, vis

    sub_topic = "/c920/rect_image"
    pub_topic = "obj_detection_msg"

    rospy.init_node('tf_object_detection', anonymous = True)

    if rospy.has_param('min_confidence'):
        min_confidence = rospy.get_param('min_confidence')

    if rospy.has_param('image_topic'):
        sub_topic = rospy.get_param('image_topic')

    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    # This shouldn't need to change
    rospack = rospkg.RosPack()
    THIS_DIR = join(rospack.get_path('tf_object_detection'), 'src/')
    PATH_TO_FROZEN_GRAPH = join(THIS_DIR, 'frozen_inference_graph.pb')
    #PATH_TO_FROZEN_GRAPH = join(THIS_DIR, 'trt_ssd_mobilenet_v2.pb')
    rospy.logwarn("Loading graph from " + str(PATH_TO_FROZEN_GRAPH))

    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = join(THIS_DIR, '2020Game_label_map.pbtxt')

    # Init TF detection graph and session
    detection_graph = tf.Graph()
    with detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    with detection_graph.as_default():
        config = tf.compat.v1.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(graph=detection_graph, config=config)
    print("Past Session")
    category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
    category_dict = {0: 'background'}
    for k in category_index.keys():
        category_dict[k] = category_index[k]['name']
    vis = BBoxVisualization(category_dict)

    sub = rospy.Subscriber(sub_topic, Image, run_inference_for_single_image)
    pub = rospy.Publisher(pub_topic, TFDetection, queue_size=2)
    pub_debug = rospy.Publisher("debug_image", Image, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
