from pathlib import Path
# Need to import this after caling YOLO or training fails?
from onnx_to_tensorrt import onnx_to_tensorrt
import subprocess
export_det_args = [
    'python3',
    '/home/ubuntu/YOLOv8-TensorRT/export-det.py',
    '--weights', 'FRC2024m.pt',
    '--iou-thres', '0.65',
    '--conf-thres', '0.25',
    '--topk', '100',
    '--opset', '11',
    '--sim',
    '--input-shape', '1',  '3', f'640', f'640', 
    '--device', 'cuda:0',
]
subprocess.run(export_det_args)