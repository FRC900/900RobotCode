#!/usr/bin/env python3
import argparse
import cv2
from ultralytics import YOLO

def main(args: argparse.Namespace) -> None:
    # Load the YOLOv8 model
    model = YOLO(args.model)

    # Open the video file
    image = cv2.imread(args.input_image)

    # Run YOLOv8 inference on the frame
    # Add agnostic_nms=True to remove overlapping
    # detections of different classes for a single
    # object (useful for apriltags)
    results = model(image)

    # Visualize the results on the image
    annotated_image = results[0].plot()

    # Display the annotated image
    cv2.imshow("YOLOv8 Inference", annotated_image)

    # Break the loop if 'q' is pressed
    cv2.waitKey(0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, help='Model file')
    parser.add_argument('--input-image', type=str, help='Input video')
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    main(args)