#!/usr/bin/env python3

"""
Run object detection on a set of images
Create / update .xml label files for the objects detected in those images
Used to automatically create an initial set of labels for new training images

"""
import argparse
import cv2
from ultralytics import YOLO

from pathlib import Path

import os
import glob
from pascal import PascalVOC, PascalObject, BndBox, size_block
from xmlformatter import Formatter

def box_to_rect(box):
  x_min = int(box[0].item())
  y_min = int(box[1].item())
  x_max = int(box[2].item())
  y_max = int(box[3].item())
  return [x_min, y_min, x_max, y_max]

def check_iou(detected_rect, previous_labels, threshold):
  #print(f"check_iou : {detected_rect}")
  for label in previous_labels:
    new_rect = []
    new_rect.append(min(label.bndbox.xmin, label.bndbox.xmax))
    new_rect.append(min(label.bndbox.ymin, label.bndbox.ymax))
    new_rect.append(max(label.bndbox.xmin, label.bndbox.xmax))
    new_rect.append(max(label.bndbox.ymin, label.bndbox.ymax))
    #print(f"\tnew_rect = {new_rect}")
    if (bb_intersection_over_union(detected_rect, new_rect) > threshold):
      return False
  return True

# From https://pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/
def bb_intersection_over_union(boxA, boxB):
    # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
        # compute the area of both the prediction and ground-truth
        # rectangles
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)
        # return the intersection over union value
        return iou

def main(args: argparse.Namespace) -> None:
    formatter = Formatter(indent="1", indent_char="\t", eof_newline=True)
    model = YOLO(args.model)

    TEST_IMAGE_PATHS = sorted(glob.glob(args.input_files))
    print(f"TEST_IMAGE_PATHS = {TEST_IMAGE_PATHS}")
    for image_path in TEST_IMAGE_PATHS:
      print(f"image_path = {image_path}")
      image = cv2.imread(image_path)            
      results = model(image)
      result = results[0].cpu()

      xml_path = Path(image_path).with_suffix('.xml')
 
      try:
        voc = PascalVOC.from_xml(xml_path)
      except:
        voc = PascalVOC(image_path, path=os.path.abspath(image_path), size=size_block(image.shape[1], image.shape[0], 3), objects=[])

      # valid_labels are a list of string labels which are valid for the new model.  If running a model with the 
      # same labels as the desired new labels, use this.
      # TODO: If we're bootstrapping new images for a new model by running an old model,
      # this will have to be hand-coded with the labels from the new model, 
      # perhaps by reading them from the FRC202x.yaml file?
      valid_labels = [result.names[n] for n in result.names]

      # previous_labels is a map of obj.name -> list of previous labels of that type
      # read from the existing xml.
      # This is used to check against duplicating new labels on top of existing ones
      previous_labels = {}
      for cls_id in result.names:
          previous_labels[result.names[int(cls_id)]] = []
      print(f"valid label['names'] = {valid_labels}")
      for obj in voc.objects:
         if obj.name not in valid_labels:
            continue
         previous_labels[obj.name].append(obj)

      print(previous_labels)
      added_labels = False
      boxes = result.boxes
      for box, conf, cls in zip(boxes.xyxy, boxes.conf, boxes.cls):
          if conf < 0.2:
              continue
          cls = int(cls)
          print(f"cls = {cls}, result.names = {result.names}")
          if cls not in result.names:
              continue

          label = result.names[cls]
          print(f"label = {label}")

          rect = box_to_rect(box)
          if not check_iou(rect, previous_labels[label], 0.1):
              print(f"label {label} failed IoU check")
              continue

          # Generate a list of all AprilTags from the original XML.  Use this to
          # do an additional check of IoU if the current label is also an apriltag
          # This is to prevent a previously hand-labeled apriltag from being overwritten
          # by an auto-labeling of the wrong ID
          if 'april_tag' in label:
            all_apriltags = []
            for p in previous_labels:
              all_apriltags.extend(previous_labels[p])

            if not check_iou(rect, all_apriltags, 0.1):
                print(f"label {label} failed combined AprilTag IoU check")
                continue

          print(f"Adding new {label} at {rect}")
          voc.objects.append(PascalObject(label, "Unspecified", truncated=False, difficult=False, bndbox=BndBox(rect[0], rect[1], rect[2], rect[3])))
          added_labels = True

      if added_labels:
        voc.save(xml_path)
        formatted_xml_str = formatter.format_file(xml_path) 
        formatter.enc_output(xml_path, formatted_xml_str)

      if args.show:
        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(0) & 0xFF == ord("q"):
            break


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='FRC2024m.pt', help='Model .pt file')
    parser.add_argument('--input-files', type=str, help='Input files to process')
    parser.add_argument('--show',
                        action='store_true',
                        help='Show the detection results')
    parser.add_argument('--device',
                        type=str,
                        default='cuda:0',
                        help='TensorRT infer device')
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    main(args)
