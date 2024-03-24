#include "deeptag_ros/cuda_ssd_preprocess.h"
#include "deeptag_ros/cuda_utils.h"

// Kernel to preprocess SSD tensors

// gpuSSDPreprocess
template <typename T, bool isBGR>
__global__ void gpuSSDPreprocess(const float2 scale,
								 const T *input,
								 const ushort2 iSize,
								 float *output,
								 const ushort2 oSize,
								 float multiplier,
								 float min_value)
{
	const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
	const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x >= oSize.x || y >= oSize.y)
		return;

	// Resize coords
	const float dx = static_cast<float>(x) * scale.x;
	const float dy = static_cast<float>(y) * scale.y;

	// Bilinear interpolation
	const float cx = min(static_cast<float>(iSize.x - 1), max(dx, 0.0f));
	const float cy = min(static_cast<float>(iSize.y - 1), max(dy, 0.0f));

	const uint32_t x1 = static_cast<uint32_t>(cx);
	const uint32_t y1 = static_cast<uint32_t>(cy);

	const uint32_t x2 = min(x1 + 1, iSize.x - 1);    // bounds check
	const uint32_t y2 = min(y1 + 1, iSize.y - 1);

	// Input bounds check for cases where the input image is smaller than
	// the model input size
	// I don't know T{} is, but it has to be better than running past
	// the end of the input image data
	const T samples[4] = {
		((x1 < iSize.x) && (y1 < iSize.y)) ? input[y1 * iSize.x + x1] : T{},
		((x2 < iSize.x) && (y1 < iSize.y)) ? input[y1 * iSize.x + x2] : T{},
		((x1 < iSize.x) && (y2 < iSize.y)) ? input[y2 * iSize.x + x1] : T{},
		((x2 < iSize.x) && (y2 < iSize.y)) ? input[y2 * iSize.x + x2] : T{},
	};

	// compute bilinear weights
	const float x1d = cx - static_cast<float>(x1);
	const float y1d = cy - static_cast<float>(y1);

	const float x1f = 1.0f - x1d;
	const float y1f = 1.0f - y1d;

	const float x2f = 1.0f - x1f;
	const float y2f = 1.0f - y1f;

	const float x1y1f = x1f * y1f;
	const float x1y2f = x1f * y2f;
	const float x2y1f = x2f * y1f;
	const float x2y2f = x2f * y2f;

	float3 px{
		samples[0].x * x1y1f + samples[1].x * x2y1f + samples[2].x * x1y2f + samples[3].x * x2y2f,
		samples[0].y * x1y1f + samples[1].y * x2y1f + samples[2].y * x1y2f + samples[3].y * x2y2f,
		samples[0].z * x1y1f + samples[1].z * x2y1f + samples[2].z * x1y2f + samples[3].z * x2y2f
	};

	const float3 rgb = isBGR ? make_float3(px.z, px.y, px.x)
							 : make_float3(px.x, px.y, px.z);

	const uint32_t n = static_cast<uint32_t>(oSize.x) * oSize.y;
	const uint32_t m = y * oSize.x + x;
	output[n * 0 + m] = rgb.x * multiplier + min_value;
	output[n * 1 + m] = rgb.y * multiplier + min_value;
	output[n * 2 + m] = rgb.z * multiplier + min_value;
}

template <bool isBGR>
static cudaError_t launchSSDPreprocess(const void *input,
									   const imageFormat format,
									   const ushort2 &inputSize,
									   float *output,
									   const ushort2 &outputSize,
									   const float2 &range,
									   cudaStream_t stream)
{
	if (!input || !output)
		return cudaErrorInvalidDevicePointer;

	if (inputSize.x == 0 || outputSize.x == 0 || inputSize.y == 0 || outputSize.y == 0)
		return cudaErrorInvalidValue;

	const float2 scale = make_float2(float(inputSize.x) / float(outputSize.x),
									 float(inputSize.y) / float(outputSize.y));

	const float multiplier = (range.y - range.x) / 255.0f;

	// launch kernel
	const dim3 blockDim(16, 16);
	const dim3 gridDim(iDivUp(outputSize.x, blockDim.x), iDivUp(outputSize.y, blockDim.y));

	//added BGR options
	if ((format == IMAGE_RGB8) || (format == IMAGE_BGR8))
		gpuSSDPreprocess<uchar3, isBGR><<<gridDim, blockDim, 0, stream>>>(scale, (uchar3 *)input, inputSize, output, outputSize, multiplier, range.x);
	else if ((format == IMAGE_RGBA8) || (format == IMAGE_BGRA8))
		gpuSSDPreprocess<uchar4, isBGR><<<gridDim, blockDim, 0, stream>>>(scale, (uchar4 *)input, inputSize, output, outputSize, multiplier, range.x);
	else if ((format == IMAGE_RGB32F) || (format == IMAGE_BGR32F))
		gpuSSDPreprocess<float3, isBGR><<<gridDim, blockDim, 0, stream>>>(scale, (float3 *)input, inputSize, output, outputSize, multiplier, range.x);
	else if ((format == IMAGE_RGBA32F) || (format == IMAGE_BGRA32F))
		gpuSSDPreprocess<float4, isBGR><<<gridDim, blockDim, 0, stream>>>(scale, (float4 *)input, inputSize, output, outputSize, multiplier, range.x);
	else
		return cudaErrorInvalidValue;

	return cudaGetLastError();
}

// cudaSSDPreprocessRGB
cudaError_t cudaSSDPreprocessRGB(const void *input,
								 const imageFormat format,
								 const ushort2 &inputSize,
								 float *output,
								 const ushort2 &outputSize,
								 const float2 &range,
								 cudaStream_t stream)

{
	return launchSSDPreprocess<false>(input, format, inputSize, output, outputSize, range, stream);
}

// cudaSSDPreprocessBGR
cudaError_t cudaSSDPreprocessBGR(const void *input,
								 const imageFormat format,
								 const ushort2 &inputSize,
								 float *output,
								 const ushort2 &outputSize,
								 const float2 &range,
								 cudaStream_t stream)
{
	return launchSSDPreprocess<true>(input, format, inputSize, output, outputSize, range, stream);
}

