#!/usr/bin/env python3
'''
TODO - set up a command line arg with options for 1st time training, or transfer learning
       or perhaps set it automatically based on the presence of a yolo model .pt file in the same directory
1st time training:
    - Use the default YOLOv8 args for patience, optimizer, lr0/lr (these are overridden by auto), warmup_bias_lr

Trainsfer learning (tune these)
       optimizer='SGD',
        lr0=0.002,
        warmup_bias_lr=0.001

Note, the final learning rate is lr0 * lrf. Default lrf is 0.01, meaning the final learning rate is 1% of
lr0. That's true in both cases, so leave this at the default for now.
'''
import argparse
from ultralytics import YOLO
from os import rename

def train_yolo(args: argparse.Namespace) -> None:
    if args.postprocess_model is None:
        model = YOLO(args.yolo_model)

        model.train(data=args.config,
                    epochs=args.epochs,
                    imgsz=args.input_size,
                    batch=args.batch_size,
                    patience=args.patience,
                    optimizer='SGD',
                    lr0=0.00125,
                    warmup_bias_lr=args.warmup_bias_lr,
                    augment=True, #Pretty sure this is a no-op
                    fliplr=False,
                    flipud=False,
                    degrees=20.0)
        pt_file_path = model.trainer.best
    else:
        pt_file_path = args.postprocess_model
    print(f'Best pt weights = {pt_file_path}')

    # Now convert from pytorch .pt format to a .onnx file
    # This is an intermediate step - the onnx file format is generic,
    # and in this case provides a way to translate from .pt to
    # an optimized TensorRT engine file

    from pathlib import Path
    # Need to import this after caling YOLO or training fails?
    from onnx_to_tensorrt import onnx_to_tensorrt
    import subprocess
    export_det_args = [
        'python3',
        '/home/ubuntu/YOLOv8-TensorRT/export-det.py',
        '--weights', pt_file_path,
        '--iou-thres', '0.65',
        '--conf-thres', '0.25',
        '--topk', '100',
        '--opset', '11',
        '--sim',
        '--input-shape', '1',  '3', f'{args.input_size}', f'{args.input_size}', 
        '--device', 'cuda:0',
    ]
    subprocess.run(export_det_args)


    # The output will be a file with .onnx as the extension
    onnx_path = Path(pt_file_path).with_suffix('.onnx')

    # Create a tensorrt .engine file. This is a model optimized for the specific
    # hardware we're currently running on. That will be useful for testing
    # the model locally.
    # Additionally, it will create a calibration file useful for optimizing
    # int8 models on other platforms.  
    tensorrt_path = onnx_path.with_name(args.output_stem + '_int8').with_suffix('.engine')
    calibration_path = onnx_path.with_name('calib_' + args.output_stem + '.bin')

    onnx_to_tensorrt(onnx_path,
                     tensorrt_path,
                     int8=True,
                     fp16=True,
                     dataset_path='/home/ubuntu/tensorflow_workspace/2024Game/datasets/FRC2024/images/train',
                     calibration_file=calibration_path)
    new_pt_file_path = Path(pt_file_path).with_stem(args.output_stem)
    new_onnx_path = onnx_path.with_stem(args.output_stem)
    rename(pt_file_path, new_pt_file_path)
    rename(onnx_path, new_onnx_path)
    print(f"Training finished, generated {new_pt_file_path}, {new_onnx_path}, {tensorrt_path} and {calibration_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--yolo-model',
                        type=str,
                        default='/home/ubuntu/2023RobotCode/zebROS_ws/src/tf_object_detection/src/FRC2024m.pt',
                        help='Engine file')
    parser.add_argument('--config',
                        type=str,
                        default='/home/ubuntu/2023RobotCode/zebROS_ws/src/tf_object_detection/src/FRC2024.yaml',
                        help='Config file')
    parser.add_argument('--output_stem',
                        type=str,
                        default='best',
                        help='File name stem for pt, onnx, engine and calibration file')
    parser.add_argument('--epochs',
                        type=int,
                        default=150,
                        help='Number of epochs to train')
    parser.add_argument('--patience',
                        type=int,
                        default=50,
                        help='Stop training early if this many epochs pass without improvement')
    # When doing reinforcment learning from a previously trained model, use a lower initial
    # learning rate to avoid jumping away from the previous model's weights
    parser.add_argument('--warmup-bias-lr',
                        type=float,
                        default=0.001,
                        help='Use a different learning rate for the first few epochs')
    parser.add_argument('--batch-size',
                        type=int,
                        default=12,
                        help='Number of images to batch')
    parser.add_argument('--postprocess-model',
                        type=str,
                        default=None,
                        help='Skip training, just run posprocessing on requested .pt file')
    parser.add_argument('--input-size',
                        type=int,
                        default=640,
                        help='Model input image size')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    train_yolo(args)
