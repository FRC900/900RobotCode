#ifndef CUDA_SSD_PREPROCESSING_H__
#define CUDA_SSD_PREPROCESSING_H__

#include "image_format.h"

/*
 * Downsample and apply pixel normalization, NCHW format
 */
cudaError_t cudaSSDPreprocessRGB(const void *input,
                                 const imageFormat format,
                                 const ushort2 &inputWidth,
                                 float *output,
                                 const ushort2 &outputSize,
                                 const float2 &range,
                                 cudaStream_t stream);
cudaError_t cudaSSDPreprocessBGR(const void *input,
                                 const imageFormat format,
                                 const ushort2 &inputWidth,
                                 float *output,
                                 const ushort2 &outputSize,
                                 const float2 &range,
                                 cudaStream_t stream);

#endif

