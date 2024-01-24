#!/usr/bin/env python3

import argparse
import cv2
from baseYOLO import YOLO900
import timing
import numpy as np
import time
def add_transparent_rect(img):
    # First we crop the sub-rect from the image
    x, y, w, h = 0, 0, img.shape[1], 80
    sub_img = img[y:y+h, x:x+w]
    white_rect = np.ones(sub_img.shape, dtype=np.uint8) * 255

    res = cv2.addWeighted(sub_img, 0.5, white_rect, 0.5, 1.0)

    # Putting the image back to its position
    img[y:y+h, x:x+w] = res
    text = 'GPU PREPROC - IMAGINARY LETTERBOX'
    
    # font
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # org
    org = (00, 50)
    
    # fontScale
    fontScale = 1
    
    # Red color in BGR
    color = (0, 0, 255)
    
    # Line thickness of 2 px
    thickness = 2
    
    # Using cv2.putText() method
    img = cv2.putText(img, text, org, font, fontScale, 
                    color, thickness, cv2.LINE_AA, False)
    return img

pixel_shift = 800

def main(args: argparse.Namespace) -> None:
    global pixel_shift
    regen_trt = not args.skip_engine_regen
    DETECTRON = YOLO900(engine_path=args.engine, device_str=args.device, use_timings=True, regen_trt=regen_trt)

    t = timing.Timings()
    DETECTRON.t = t

    t.start('imread')
    original_bgr = cv2.imread(args.input_image)
     
    #bgr = cv2.flip(bgr, 0)
    M = np.float32([
	[1, 0, 0],
	[0, 1, pixel_shift]
    ])
    bgr = cv2.warpAffine(original_bgr, M, (original_bgr.shape[1], original_bgr.shape[0]))
    bgr = add_transparent_rect(bgr)
    
    bgr = original_bgr
    t.end('imread')

    #bgr = image_data

    while True:
        t.start('frame')
        M = np.float32([
        [1, 0, 0],
        [0, 1, pixel_shift]
        ])
         
        bgr = cv2.warpAffine(original_bgr, M, (original_bgr.shape[1], original_bgr.shape[0]))
        bgr = add_transparent_rect(bgr)
        pixel_shift -= 1
        
        if pixel_shift == -100:
            pixel_shift = 800
        

        if not args.cpu_preprocess:
            detections = DETECTRON.gpu_preprocess(original_bgr, debug=args.show).infer() 
        else:
            detections = DETECTRON.cpu_preprocess(original_bgr, debug=False).infer()  


        t.start("viz")
        if args.show:
            cv2.imshow('result', DETECTRON.draw_bboxes())
            t.end('viz')
            key = cv2.waitKey(0) & 0x000000ff
            if key == 27:
                break
            if key == 99: # 'c'
                args.cpu_preprocess = not args.cpu_preprocess
                print(f'Toggled cpu_preprocess to {args.cpu_preprocess}')
        else:
            t.end('viz')

        t.end('frame')
        time.sleep(0.001)
        break


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--engine', type=str, help='Engine file')
    parser.add_argument('--input-image', type=str, help='Input image path')
    parser.add_argument('--show',
                        action='store_true',
                        help='Show the detection results')
    parser.add_argument('--cpu-preprocess',
                        action='store_true',
                        help='Do image preprocessing on the CPU (slower)')
    parser.add_argument('--skip-engine-regen',
                        action='store_true',
                        help='Don\'t regenerate TensorRT engine file if .onnx file changes')
    parser.add_argument('--device',
                        type=str,
                        default='cuda:0',
                        help='TensorRT infer device')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)