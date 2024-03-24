#include "deeptag_ros/cuda_image_tile.h"
#include "deeptag_ros/cuda_utils.h"

// Kernel to copy an RoI from the input image into an output tensor
// Converts from BGR8/RGB8 to flat float32 in the process so it
//  is ready for input to engine inference
template <typename T, bool isBGR>
__global__ void gpuImageTile(const T *input,		 // RGB / BGR input image
							 const ushort2 iSize,	 // input image width and height
							 float *output,			 // output tensor
							 const ushort2 oSize,	 // desired output width and height
							 const ushort2 shift,	 // offset from origin of input image
							 const float multiplier, // used for scaling from uint8_t to float
							 const float min_value)	 // used for scaling from uint8_t to float
{
	// x and y are output x & y pixel coords
	const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
	const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x >= oSize.x || y >= oSize.y)
		return;

	// n is channel offset
	// m is x,y pixel offset into that particular channel
	const uint32_t n = static_cast<uint32_t>(oSize.x) * oSize.y;
	const uint32_t m = y * oSize.x + x;
	// Don't read past end of input image if it is smaller
	// that model input size
	if (((x + shift.x) < iSize.x) && ((y + shift.y) < iSize.y))
	{
		// Input pixel coords cald'c by shifting the output pixel coords
		const T px = input[(y + shift.y) * iSize.x + x + shift.x];

		// Basically free bgr->rgb if needed
		const float3 rgb = isBGR ? make_float3(px.z, px.y, px.x)
								 : make_float3(px.x, px.y, px.z);

		output[n * 0 + m] = rgb.x * multiplier + min_value;
		output[n * 1 + m] = rgb.y * multiplier + min_value;
		output[n * 2 + m] = rgb.z * multiplier + min_value;
	}
	else
	{
		output[n * 0 + m] = 0;
		output[n * 1 + m] = 0;
		output[n * 2 + m] = 0;
	}
}

template <bool isBGR>
static cudaError_t launchImageTile(const void *input,
								   const imageFormat format,
								   const ushort2 &inputSize,
								   float *output,
								   const ushort2 &outputSize,
								   const ushort2 &shift,
								   const float2 &range,
								   cudaStream_t stream)
{
	if (!input || !output)
		return cudaErrorInvalidDevicePointer;

	if (inputSize.x == 0 || outputSize.x == 0 || inputSize.y == 0 || outputSize.y == 0)
		return cudaErrorInvalidValue;

	// Check for invalid shift values
	if ((outputSize.x + shift.x) > inputSize.x)
		return cudaErrorInvalidValue;

	if ((outputSize.y + shift.y) > inputSize.y)
		return cudaErrorInvalidValue;

	const float multiplier = (range.y - range.x) / 255.0f;

	// launch kernel
	const dim3 blockDim(16, 16);
	const dim3 gridDim(iDivUp(outputSize.x, blockDim.x), iDivUp(outputSize.y, blockDim.y));

	//added BGR options
	if ((format == IMAGE_RGB8) || (format == IMAGE_BGR8))
		gpuImageTile<uchar3, isBGR><<<gridDim, blockDim, 0, stream>>>((uchar3 *)input, inputSize, output, outputSize, shift, multiplier, range.x);
	else if ((format == IMAGE_RGBA8) || (format == IMAGE_BGRA8))
		gpuImageTile<uchar4, isBGR><<<gridDim, blockDim, 0, stream>>>((uchar4 *)input, inputSize, output, outputSize, shift, multiplier, range.x);
	else if ((format == IMAGE_RGB32F) || (format == IMAGE_BGR32F))
		gpuImageTile<float3, isBGR><<<gridDim, blockDim, 0, stream>>>((float3 *)input, inputSize, output, outputSize, shift, multiplier, range.x);
	else if ((format == IMAGE_RGBA32F) || (format == IMAGE_BGRA32F))
		gpuImageTile<float4, isBGR><<<gridDim, blockDim, 0, stream>>>((float4 *)input, inputSize, output, outputSize, shift, multiplier, range.x);
	else
		return cudaErrorInvalidValue;

	return cudaGetLastError();
}

// cudaImageTileRGB
cudaError_t cudaImageTileRGB(const void *input,
							 const imageFormat format,
							 const ushort2& inputSize,
							 float *output,
							 const ushort2 &outputSize,
							 const ushort2 &shift,
							 const float2 &range,
							 cudaStream_t stream)
{
	return launchImageTile<false>(input,
								  format,
								  inputSize,
								  output,
								  outputSize,
								  shift,
								  range,
								  stream);
}

// cudaImageTileBGR
cudaError_t cudaImageTileBGR(const void *input,
							 const imageFormat format,
							 const ushort2& inputSize,
							 float *output,
							 const ushort2 &outputSize,
							 const ushort2 &shift,
							 const float2 &range,
							 cudaStream_t stream)
{
	return launchImageTile<true>(input,
								 format,
								 inputSize,
								 output,
								 outputSize,
								 shift,
								 range,
								 stream);
}
