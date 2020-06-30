'''
Script to generate TRT output graph files from a given checkpoint

TensorRT generates optimized code for running networks on GPUs, both for
desktop and especially for the Jetson.  The optimization process takes a bit
of time, so it is possible to preprocess and save the output. 
That's the goal of this script.

It will first look for frozen_inference_graph.pb, and if found,
create a corresponding optimized TRT file.

If the frozen graph is not found, it will try to create it using
checkpoint files. This mode is not likely to be necessary since we don't
put checkpoint files in the 2020RobotCode repo.  It is left over from
the stand-alone script in the tensorflow workspace, but the purposes
of this script and that one have diverged.

'''
# Dir where model.ckpt* files are being generated
import sys
import os.path
SAVED_MODEL_DIR='/home/ubuntu/2020RobotCode/zebROS_ws/src/tf_object_detection/src'
CHECKPOINT_NUMBER='118209'
sys.path.append(os.path.join(SAVED_MODEL_DIR, 'modules'))
sys.path.append(os.path.join(os.path.join(SAVED_MODEL_DIR, 'modules'), 'slim'))

import tensorflow as tf
import tensorflow.contrib.tensorrt as trt
from object_detection.protos import pipeline_pb2
from object_detection import exporter
from google.protobuf import text_format
import subprocess

from graph_utils import force_nms_cpu as f_force_nms_cpu
from graph_utils import replace_relu6 as f_replace_relu6
from graph_utils import remove_assert as f_remove_assert

from file_changed import file_changed

# Output file name
TRT_OUTPUT_GRAPH = 'trt_graph.pb'

# Network config
CONFIG_FILE=os.path.join(SAVED_MODEL_DIR, 'model/ssd_mobilenet_v2_coco.config')

INPUT_NAME='image_tensor'
BOXES_NAME='detection_boxes'
CLASSES_NAME='detection_classes'
SCORES_NAME='detection_scores'
MASKS_NAME='detection_masks'
NUM_DETECTIONS_NAME='num_detections'
MODEL_CHECKPOINT_PREFIX='model.ckpt-'
FROZEN_GRAPH_NAME='frozen_inference_graph.pb'

def load_frozen_graph(frozen_graph_name,
        force_nms_cpu=True,
        replace_relu6=True,
        remove_assert=True):
    # read frozen graph from file
    frozen_graph = tf.GraphDef()
    with open(frozen_graph_name, 'rb') as f:
        frozen_graph.ParseFromString(f.read())

    # apply graph modifications
    if force_nms_cpu:
        frozen_graph = f_force_nms_cpu(frozen_graph)
    if replace_relu6:
        frozen_graph = f_replace_relu6(frozen_graph)
    if remove_assert:
        frozen_graph = f_remove_assert(frozen_graph)

    # get input names
    # TODO: handle mask_rcnn 
    input_names = [INPUT_NAME]
    output_names = [BOXES_NAME, CLASSES_NAME, SCORES_NAME, NUM_DETECTIONS_NAME]

    return frozen_graph, input_names, output_names


# from tf_trt models dir
def build_frozen_graph(frozen_graph_name, config, checkpoint,
        batch_size=1,
        score_threshold=None,
        input_shape=None,
        output_dir='.generated_model'):
    """Builds a frozen graph for a pre-trained object detection model"""
    
    config_path = config
    checkpoint_path = checkpoint

    # parse config from file
    config = pipeline_pb2.TrainEvalPipelineConfig()
    with open(config_path, 'r') as f:
        text_format.Merge(f.read(), config, allow_unknown_extension=True)

    # override some config parameters
    if config.model.HasField('ssd'):
        config.model.ssd.feature_extractor.override_base_feature_extractor_hyperparams = True
        if score_threshold is not None:
            config.model.ssd.post_processing.batch_non_max_suppression.score_threshold = score_threshold    
        if input_shape is not None:
            config.model.ssd.image_resizer.fixed_shape_resizer.height = input_shape[0]
            config.model.ssd.image_resizer.fixed_shape_resizer.width = input_shape[1]
    elif config.model.HasField('faster_rcnn'):
        if score_threshold is not None:
            config.model.faster_rcnn.second_stage_post_processing.score_threshold = score_threshold
        if input_shape is not None:
            config.model.faster_rcnn.image_resizer.fixed_shape_resizer.height = input_shape[0]
            config.model.faster_rcnn.image_resizer.fixed_shape_resizer.width = input_shape[1]

    if os.path.isdir(output_dir):
        subprocess.call(['rm', '-rf', output_dir])

    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True

    # export inference graph to file (initial)
    with tf.Session(config=tf_config) as tf_sess:
        with tf.Graph().as_default() as tf_graph:
            exporter.export_inference_graph(
                'image_tensor', 
                config, 
                checkpoint_path, 
                output_dir, 
                input_shape=[batch_size, None, None, 3]
            )

    # remove temporary directory after saving frozen graph output
    os.rename(os.path.join(output_dir, FROZEN_GRAPH_NAME), frozen_graph_name)
    subprocess.call(['rm', '-rf', output_dir])


# TODO - turn this into a function called from
# the main ROS TRT object detection script at startup
# It will look for the trt saved graph and if not found create it
def main():
    if not file_changed(os.path.join(SAVED_MODEL_DIR, FROZEN_GRAPH_NAME)):
        print("Frozen graph not changed, not rebuilding")
        return
    
    # What model to run from - should be the directory name of an exported trained model
    # Change me to the directory checkpoint files are saved in
    frozen_graph_name = os.path.join(SAVED_MODEL_DIR, FROZEN_GRAPH_NAME)
    if not os.path.isfile(frozen_graph_name):
        print("Frozen graph not found, building...")
        build_frozen_graph(
            config=CONFIG_FILE,
            checkpoint=os.path.join(SAVED_MODEL_DIR, MODEL_CHECKPOINT_PREFIX+CHECKPOINT_NUMBER),
            score_threshold=0.2,
            batch_size=1
        )
    else:
        print("Frozen graph found, not rebuilding...")

    # read frozen graph from file
    frozen_graph, input_names, output_names = load_frozen_graph(frozen_graph_name)
    trt_graph = trt.create_inference_graph(
        input_graph_def=frozen_graph,
        outputs=output_names,
        max_batch_size=1,
        max_workspace_size_bytes=1 << 25,
        precision_mode='FP16', # TODO - FP16 or INT8 for Jetson
        minimum_segment_size=50
    )

    with open(os.path.join(SAVED_MODEL_DIR, TRT_OUTPUT_GRAPH), 'wb') as f:
        f.write(trt_graph.SerializeToString())

if __name__ == '__main__':
    main()
