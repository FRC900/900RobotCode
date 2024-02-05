import cv2
import cupy
import math
import torch
from sys import path
path.append('/home/ubuntu/YOLOv8-TensorRT')
from models import TRTModule  # isort:skip
from models.torch_utils import det_postprocess
from models.utils import blob, letterbox
from cuda_event_timing import Timings
from pathlib import Path
import pytorch_pfn_extras as ppe
from collections import namedtuple
from onnx_to_tensorrt import onnx_to_tensorrt
from config_frc2024 import OBJECT_CLASSES, COLORS

# working nearest neighbor
'''
yolo_preprocess = cupy.RawKernel(
    r"""extern "C" __global__
    void yolo_preprocess(const unsigned char* input, float* output, int oWidth, int oHeight, int iWidth, int iHeight, int rowsToShiftDown) {
        const int oWindow = oHeight - (2 * rowsToShiftDown);
        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        const int y = blockIdx.y * blockDim.y + threadIdx.y;

        if (x >= oWidth || y >= oWindow) { // || y < rowsToShiftDown did not work 
            return;
        }

        const float new_x = float(x) / float(oWidth) * float(iWidth);
        const float new_y = float(y) / float(oWindow) * float(iHeight);

        int i;

        for (i = 0; i < 3; i++) {
            const int nearest_x = int(new_x + 0.5f);  // Nearest neighbor X coordinate
            const int nearest_y = int(new_y + 0.5f);  // Nearest neighbor Y coordinate

            const int clamped_x = max(0, min(nearest_x, iWidth - 1));  // Clamp X coordinate
            const int clamped_y = max(0, min(nearest_y, iHeight - 1));  // Clamp Y coordinate

            const float sample = (float)input[(clamped_y * iWidth + clamped_x) * 3 + i];

            const int rgb_offset = (oWidth * oHeight);
            int idx = rgb_offset * (2 - i) + (((y + rowsToShiftDown) * oWidth + x));

            output[idx] = sample / 255.0f;
        }
    }
""",
    "yolo_preprocess",
)
''' 

# @TODO make work on sideways images
yolo_preprocess = cupy.RawKernel(
    r"""
extern "C" __global__

// assumes that the letterbox will be drawn above and below the image rather than on the sides 
// should be fine given we want squares from long rectangles
// 1024x1024 resize still works with this which should be our max sqaure size

// input is bgr
// output is filled with color for letterbox, does bilinear interpolation with a shift to keep aspect ratio, scales 0-1, transposes to all reds, blues and greens
void yolo_preprocess(const unsigned char* input, float* output, int oWidth, int oHeight, int iWidth, int iHeight, int rowsToShiftDown) {
    const int oWindow = oHeight - (2 * rowsToShiftDown);
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    
	if( x >= oWidth || y >= (oWindow)) { 
        return;
    }
    
    
    const float new_x = float(x) / float(oWidth) * float(iWidth);
    const float new_y = float(y) / float(oWindow) * float(iHeight);

    int i;

    // loop for R G and B since no float3 :(
    for (i=0;i<3;i++) {
		const float bx = new_x - 0.5f;
		const float by = new_y - 0.5f;

		const float cx = bx < 0.0f ? 0.0f : bx;
		const float cy = by < 0.0f ? 0.0f : by;

		const int x1 = int(cx);
		const int y1 = int(cy);
			
		const int x2 = x1 >= iWidth - 1 ? x1 : x1 + 1;	// bounds check
		const int y2 = y1 >= iHeight - 1 ? y1 : y1 + 1;
		
        //const int x2 = min(x1 + 1, iWidth - 1);    // bounds check
        //const int y2 = min(y1 + 1, iHeight - 1);

		const float samples[4] = {
			(float)input[(y1 * iWidth + x1) * 3 + i],
			(float)input[(y1 * iWidth + x2) * 3 + i],
			(float)input[(y2 * iWidth + x1) * 3 + i],
			(float)input[(y2 * iWidth + x2) * 3 + i]};

		// compute bilinear weights
		const float x1d = cx - float(x1);
		const float y1d = cy - float(y1);

		const float x1f = 1.0f - x1d;
		const float y1f = 1.0f - y1d;

		const float x2f = 1.0f - x1f;
		const float y2f = 1.0f - y1f;

		const float x1y1f = x1f * y1f;
		const float x1y2f = x1f * y2f;
		const float x2y1f = x2f * y1f;
		const float x2y2f = x2f * y2f;

        // add to Y here to move the image down and add the letterbox part
        // 2 - i for bgr to rgb transform
        const int rgb_offset = (oWidth * oHeight); // should be safe from int division as  
        // initally used  + (2 - i) for bgr -> rbg, but for transposing now using rgb_offset * (2 - i)
        int idx = rgb_offset * (2 - i) + (((y + rowsToShiftDown) * oWidth + x));

        output[idx] = (samples[0] * x1y1f + samples[1] * x2y1f + samples[2] * x1y2f + samples[3] * x2y2f) / 255;
    }
}
""",
    "yolo_preprocess", )



Detections = namedtuple("Detections", ["bboxes", "scores", "labels"])

def iDivUp(a, b):
    if (a % b != 0):
        return a // (b + 1) # int division to replicate c++
    else:
        return a // b # should be an int


class YOLO900:


    def __init__(self, engine_path="FRC2024m.engine", device_str="cuda:0", use_timings=False, onnx_path="FRC2024m.onnx", regen_trt=True) -> None:
        self.device = torch.device(device_str)

        # @TODO check if this will ever not be 0's
        self.dwdh = torch.tensor([0, 0, 0, 0], device=self.device)
        self.ratio = None
        if regen_trt:
            self.check_and_regen_engine(onnx_path)
        self.Engine = TRTModule(engine_path, self.device)
        self.engine_H, self.engine_W = self.Engine.inp_info[0].shape[-2:]

        # set desired output names order
        self.Engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])
        self.debug_image = None
        self.input_tensor = None
        self.bboxes = None
        self.scores = None
        self.labels = None
        self.gpu_output_buffer = None
        self.last_preprocess_was_gpu = False
        self.pixels_to_shift_down = 0
        self.tensor = None
        self.t = Timings(self.Engine.stream)
        self.t.set_enabled(use_timings)
    
    def check_and_regen_engine(self, onnx_path):
        from file_changed import file_changed
        if not file_changed(onnx_path):
            return

        print("--------Regenerating engine file--------")
        # Create a tensorrt .engine file. This is a model optimized for the specific
        # hardware we're currently running on. That will be useful for testing
        # the model locally.
        # Additionally, it will create a calibration file useful for optimizing
        # int8 models on other platforms.  
        tensorrt_path = Path(onnx_path).with_suffix(".engine")
        tensorrt_path = tensorrt_path.with_name(tensorrt_path.name)        
        print(f"Tensorrt path {tensorrt_path}")
        calibration_path = tensorrt_path.with_name("calib_FRC2024m.bin")

        onnx_to_tensorrt(onnx_path,
                        tensorrt_path,
                        int8=True,
                        fp16=True,
                        dataset_path='/home/ubuntu/', # just give it a real path to get past checks for existing before using the .calib file
                        calibration_file=calibration_path)
        
        

    # assumes img is bgr
    def cpu_preprocess(self, img, debug=False):
        self.t.start("cpu_preproc")

        if debug:
            self.debug_image = img.copy()
        bgr, self.ratio, self.dwdh = letterbox(img, (self.engine_H, self.engine_W)) # resize while maintaining aspect ratio

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) # standard color conversion
        tensor = blob(rgb, return_seg=False) # convert to float, transpose, scale from 0.0->1.0
        self.tensor = tensor
        self.input_tensor = torch.asarray(tensor, device=self.device)
        
        self.t.end("cpu_preproc")
        
        self.dwdh = torch.asarray(self.dwdh * 2, dtype=torch.float32, device=self.device)
        self.last_preprocess_was_gpu = False
        #print(f"{tensor}")
        #print(type(tensor))
        #print("---------CPU ABOVE-----------")

        # probably too fancy but x.cpu_preprocess(img).infer() is really cool
        # also lets x.gpu_preprocess(img).infer() work 
        return self

    # assumes img is bgr
    def gpu_preprocess(self, img, debug=False):
        if debug:
            self.debug_image = img.copy()

        self.t.start("gpu_preproc")

        height, width = img.shape[:2]
        with ppe.cuda.stream(self.Engine.stream):
            if self.gpu_output_buffer is None:
                self.gpu_output_buffer = cupy.full((3, self.engine_H, self.engine_W), 114 / 255, dtype=cupy.float32)

            inital_gpu_image = cupy.asarray(img)
            block_size_1d = 256
            block_sqrt = int(math.sqrt(block_size_1d))
            block_size = (block_sqrt, block_sqrt)
            self.ratio = min(self.engine_W / width, self.engine_H / height)
            # Compute padding [width, height]
            new_unpad = int(round(width * self.ratio)), int(round(height * self.ratio))

            
            dw, dh = self.engine_W - new_unpad[0], self.engine_H - new_unpad[1]  # wh padding
            dw /= 2  # divide padding into 2 sides
            dh /= 2
            self.dwdh = torch.asarray((dw, dh) * 2, dtype=torch.float32, device=self.device)
            
            # will shift image down this much, and use to determine where to draw letterbox color
            self.pixels_to_shift_down = self.engine_H - new_unpad[1]
            self.pixels_to_shift_down //= 2
            #print(f"pixels {self.pixels_to_shift_down}")

            yolo_preprocess(                    # X                      # Y
                (iDivUp(self.engine_W, block_sqrt), iDivUp(self.engine_H, block_sqrt)), (block_size), 
                (inital_gpu_image, self.gpu_output_buffer, self.engine_W, self.engine_H, width, height, self.pixels_to_shift_down))

            self.input_tensor = torch.from_dlpack(self.gpu_output_buffer)
        
        self.t.end("gpu_preproc")
        
        self.last_preprocess_was_gpu = True
        
        #print(f"{self.gpu_output_buffer}")
        #print(type(self.gpu_output_buffer))
        #print("--------GPU ABOVE------------")

        return self
    
    
    def get_arrays(self):
        return cupy.asnumpy(self.gpu_output_buffer), self.tensor
    
    def infer(self) -> Detections:
        self.t.start("infer")
        
        data = self.Engine(self.input_tensor)
        
        self.t.end("infer")
        self.t.start("det_postprocess")
        self.bboxes, self.scores, self.labels = det_postprocess(data)
        self.bboxes -= self.dwdh
        self.bboxes /= self.ratio
        self.t.end("det_postprocess")

        return Detections(self.bboxes, self.scores, self.labels)

    def name_from_cls_id(self, cls_id) -> str:
        return OBJECT_CLASSES.get_name(cls_id)

    def draw_bboxes(self) -> cv2.Mat:
        if self.debug_image is None:
            print("Debug image is none, try calling the preprocessor with 'debug=True'")
            raise ValueError
        
        # just to make sure 
        d_bboxes = self.bboxes
        d_scores = self.scores
        d_labels = self.labels

        if d_bboxes.numel() != 0:
            for (bbox, score, label) in zip(d_bboxes, d_scores, d_labels):
                bbox = bbox.round().int().tolist()
                cls_id = int(label)
                cls = OBJECT_CLASSES.get_name(cls_id)
                cls = cls.replace("april_", "")
                color = COLORS[cls_id]
                #print(f"Color {color} cls_id {cls_id} cls {cls}")
                cv2.rectangle(self.debug_image, bbox[:2], bbox[2:], color, 2)
                cv2.putText(self.debug_image,
                            f'{cls}:{score:.3f}', (bbox[0], bbox[1] - 2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, [225, 255, 255],
                            thickness=2)
        return self.debug_image    
