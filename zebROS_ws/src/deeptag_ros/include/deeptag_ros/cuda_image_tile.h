#ifndef CUDA_IMAGE_TILE_INC__
#define CUDA_IMAGE_TILE_INC__
#include <cstdint>

#include "device_types.h"
#include "image_format.h"

cudaError_t cudaImageTileRGB(const void *input,
							 const imageFormat format,
							 const ushort2& inputSize,
							 float *output,
							 const ushort2 &outputSize,
							 const ushort2 &shift,
							 const float2 &range,
							 cudaStream_t stream);

cudaError_t cudaImageTileBGR(const void *input,
							 const imageFormat format,
							 const ushort2& inputSize,
							 float *output,
							 const ushort2 &outputSize,
							 const ushort2 &shift,
							 const float2 &range,
							 cudaStream_t stream);

#endif

