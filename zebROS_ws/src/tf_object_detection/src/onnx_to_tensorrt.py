#! /usr/bin/env python3
# Adapted from https://raw.githubusercontent.com/NVIDIA-AI-IOT/jetson_dla_tutorial/master/build.py
# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

# Takes an onnx model as input, outputs an optmized tensorrt engine
# This needs to be run on the target GPU (e.g. on the jetson) since
# the optimization is based on timing info obtained by running the 
# model on actual hardware
import argparse
import tensorrt as trt
import onnx

def onnx_to_tensorrt(onnx_model,
                     output, 
                     max_workspace_size=1<<25,
                     int8=False,
                     fp16=False,
                     dla_core=None,
                     batch_size=1,
                     batch_size_min=None,
                     batch_size_max=None,
                     input_width=None,
                     input_width_min=None,
                     input_width_max=None,
                     input_height=None,
                     input_height_min=None,
                     input_height_max=None,
                     gpu_fallback=False,
                     input_tensor='input',
                     dataset_path='datasets/FRC2023/images/train',
                     calibration_file='calib_FRC2023m.bin') -> None:
    import pycuda.autoinit # causes problems with multiple contexts if imported in global scope

    print(f"Onnx model {onnx_model}\nOutput {output}\nInt8 {int8}\nfp16 {fp16}\nDla core {dla_core}\nDataset {dataset_path}\nCalibration {calibration_file}")

    # Get model input size from the onnx model data
    model = onnx.load(onnx_model)
    input_shapes = [[d.dim_value for d in _input.type.tensor_type.shape.dim] for _input in model.graph.input]
    print(input_shapes)
    input_channels = input_shapes[0][1]
    #input_height = input_shapes[0][2]
    #input_width = input_shapes[0][3]
    if batch_size_min is None:
        batch_size_min = batch_size
    if batch_size_max is None:
        batch_size_max = batch_size

    if input_shapes[0][2] != 0:
        input_height = input_shapes[0][2]
        input_height_min = input_height
        input_height_max = input_height

    if input_shapes[0][3] != 0:
        input_width = input_shapes[0][3]
        input_width_min = input_width
        input_width_max = input_width

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    print(f'batch_size_max = {batch_size_max}')
    builder.max_batch_size = batch_size_max
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    onnx_parser = trt.OnnxParser(network, logger)

    with open(onnx_model, 'rb') as f:
        onnx_parser.parse(f.read())

    profile = builder.create_optimization_profile()

    profile.set_shape(
        input_tensor,
        (batch_size_min, input_channels, input_height_min, input_width_min), # min
        (batch_size, input_channels, input_height, input_width), # opt
        (batch_size_max, input_channels, input_height_max, input_width_max), # max
    )

    config = builder.create_builder_config()

    config.max_workspace_size = max_workspace_size

    if fp16:
        print("Setting FP16 flag")
        config.set_flag(trt.BuilderFlag.FP16)

    if int8:
        from calibrator import YOLOEntropyCalibrator
        config.set_flag(trt.BuilderFlag.INT8)
        config.int8_calibrator = YOLOEntropyCalibrator(dataset_path, 
                                                    (input_height, input_width),
                                                    calibration_file)
        config.set_calibration_profile(profile)
        print("Setting INT8 flag + calibrator")

    if gpu_fallback:
        print("Setting GPU_FALLBACK")
        config.set_flag(trt.BuilderFlag.GPU_FALLBACK)

    if dla_core is not None:
        config.default_device_type = trt.DeviceType.DLA
        config.DLA_core = dla_core
        config.set_flag(trt.BuilderFlag.STRICT_TYPES)
        print(f'Using DLA core {dla_core}')

    config.add_optimization_profile(profile)
    config.set_calibration_profile(profile)

    engine = builder.build_serialized_network(network, config)

    if output is not None:
        with open(output, 'wb') as f:
            f.write(engine)
    
    pycuda.autoinit._finish_up() # this sucks but since the pycuda context needs to go and the program isn't over yet I think its needed
    import atexit
    atexit.unregister(pycuda.autoinit._finish_up()) # literally does not matter but prevents an error on cleanup and is probably "correct" to do


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('onnx', type=str, help='Path to the ONNX model.')
    parser.add_argument('--output', type=str, default=None, help='Path to output the optimized TensorRT engine')
    parser.add_argument('--max-workspace-size', type=int, default=1<<25, help='Max workspace size for TensorRT engine.')
    parser.add_argument('--int8', action='store_true')
    parser.add_argument('--fp16', action='store_true')
    parser.add_argument('--dla-core', type=int, default=None)
    parser.add_argument('--batch-size', type=int, default=1)
    parser.add_argument('--batch-size-max', type=int, default=None)
    parser.add_argument('--batch-size-min', type=int, default=None)
    parser.add_argument('--input-width', type=int, default=None)
    parser.add_argument('--input-width-max', type=int, default=None)
    parser.add_argument('--input-width-min', type=int, default=None)
    parser.add_argument('--input-height', type=int, default=None)
    parser.add_argument('--input-height-max', type=int, default=None)
    parser.add_argument('--input-height-min', type=int, default=None)
    parser.add_argument('--gpu-fallback', action='store_true')
    parser.add_argument('--input-tensor', type=str, default='input')
    parser.add_argument('--dataset-path', type=str, default='datasets/FRC2023/images/train')
    parser.add_argument('--calibration-file', type=str, default='calib_FRC2023m.bin')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    onnx_to_tensorrt(args.onnx,
                     args.output,
                     args.max_workspace_size,
                     int8=args.int8,
                     fp16=args.fp16,
                     dla_core=args.dla_core,
                     batch_size=args.batch_size,
                     batch_size_min=args.batch_size_min,
                     batch_size_max=args.batch_size_max,
                     input_width=args.input_width,
                     input_width_min=args.input_width_min,
                     input_width_max=args.input_width_max,
                     input_height=args.input_height,
                     input_height_min=args.input_height_min,
                     input_height_max=args.input_height_max,
                     gpu_fallback=args.gpu_fallback,
                     input_tensor=args.input_tensor,
                     dataset_path=args.dataset_path,
                     calibration_file=args.calibration_file)