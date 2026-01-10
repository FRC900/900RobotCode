#include "deeptag_ros/decoder_preprocess.h"
#include "deeptag_ros/cuda_utils.h"
#include <iostream>

// Kernel to extract stage 2 tag images from full camera image into 3,256,256 tensors

template <typename T, bool isBGR, bool isGray=false, bool threeOutputChannels=true>
__global__ void gpuDecoderPreprocess(const float *H,
									 const T *input, uint32_t iWidth, uint32_t iHeight,
									 float *output, uint32_t oWidth, uint32_t oHeight,
									 float multiplier, float min_value)
{
	const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
	const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;

	if ((x >= oWidth) || (y >= oHeight))
		return;

	// Perspective warp
	const float denom = H[6] * x + H[7] * y + H[8];
	const float inputX = (H[0] * x + H[1] * y + H[2]) / denom;
	const float inputY = (H[3] * x + H[4] * y + H[5]) / denom;

	// Bilinear interpolation
	const float cx = min(static_cast<float>(iWidth - 1), max(inputX, 0.0f));
	const float cy = min(static_cast<float>(iHeight - 1), max(inputY, 0.0f));

	const int x1 = int(cx);
	const int y1 = int(cy);

	const int x2 = min(x1 + 1, iWidth - 1);    // bounds check
	const int y2 = min(y1 + 1, iHeight - 1);

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

	const T samples[4] = {
		input[y1 * iWidth + x1],
		input[y1 * iWidth + x2],
		input[y2 * iWidth + x1],
		input[y2 * iWidth + x2]};

	const uint32_t n = oWidth * oHeight; // size of one color channel
	const uint32_t m = y * oWidth + x;	 // pixel offset into channel

	if constexpr (!isGray)
	{
		float3 px{
			samples[0].x * x1y1f + samples[1].x * x2y1f + samples[2].x * x1y2f + samples[3].x * x2y2f,
			samples[0].y * x1y1f + samples[1].y * x2y1f + samples[2].y * x1y2f + samples[3].y * x2y2f,
			samples[0].z * x1y1f + samples[1].z * x2y1f + samples[2].z * x1y2f + samples[3].z * x2y2f
		};

		const float3 rgb = isBGR ? make_float3(px.z, px.y, px.x)
								 : make_float3(px.x, px.y, px.z);

		output[n * 0 + m] = rgb.x * multiplier + min_value;
		output[n * 1 + m] = rgb.y * multiplier + min_value;
		output[n * 2 + m] = rgb.z * multiplier + min_value;
	}
	else
	{
		const float gray = samples[0] * x1y1f + samples[1] * x2y1f + samples[2] * x1y2f + samples[3] * x2y2f;
		output[m] = gray * multiplier + min_value;
		if constexpr (threeOutputChannels)
		{
			output[n + m] = gray * multiplier + min_value;
			output[2 * n + m] = gray * multiplier + min_value;
		}
	}
}
DecoderPreprocess::DecoderPreprocess(void)
{
	cudaSafeCall(cudaMalloc(&m_dH, 9 * sizeof(float)));
}
DecoderPreprocess::~DecoderPreprocess(void)
{
	cudaSafeCall(cudaFree(m_dH));
}
template <bool isBGR, bool isGray, bool threeOutputChannels>
cudaError_t DecoderPreprocess::launchDecoderPreprocess(const float *hH, void *input, imageFormat format, size_t inputWidth, size_t inputHeight,
													   float *output, size_t outputWidth, size_t outputHeight,
													   const float2 &range, cudaStream_t stream)
{
	if (!input || !output)
	{
		return cudaErrorInvalidDevicePointer;
	}

	if (inputWidth == 0 || outputWidth == 0 || inputHeight == 0 || outputHeight == 0)
	{
		return cudaErrorInvalidValue;
	}

	const float multiplier = (range.y - range.x) / 255.0f;

	// launch kernel
	const dim3 blockDim(16, 16);
	const dim3 gridDim(iDivUp(outputWidth, blockDim.x), iDivUp(outputHeight, blockDim.y));

	cudaSafeCall(cudaMemcpyAsync(m_dH, hH, 9 * sizeof(float), cudaMemcpyHostToDevice, stream));
	//added BGR options
	if constexpr (!isGray)
	{
		if ((format == imageFormat::IMAGE_RGB8) || (format == imageFormat::IMAGE_BGR8))
			gpuDecoderPreprocess<uchar3, isBGR, isGray><<<gridDim, blockDim, 0, stream>>>(m_dH, (uchar3 *)input, inputWidth, inputHeight, output, outputWidth, outputHeight, multiplier, range.x);
		else if ((format == imageFormat::IMAGE_RGBA8) || (format == imageFormat::IMAGE_BGRA8))
			gpuDecoderPreprocess<uchar4, isBGR, isGray><<<gridDim, blockDim, 0, stream>>>(m_dH, (uchar4 *)input, inputWidth, inputHeight, output, outputWidth, outputHeight, multiplier, range.x);
		else if ((format == imageFormat::IMAGE_RGB32F) || (format == imageFormat::IMAGE_BGR32F))
			gpuDecoderPreprocess<float3, isBGR, isGray><<<gridDim, blockDim, 0, stream>>>(m_dH, (float3 *)input, inputWidth, inputHeight, output, outputWidth, outputHeight, multiplier, range.x);
		else if ((format == imageFormat::IMAGE_RGBA32F) || (format == imageFormat::IMAGE_BGRA32F))
			gpuDecoderPreprocess<float4, isBGR, isGray><<<gridDim, blockDim, 0, stream>>>(m_dH, (float4 *)input, inputWidth, inputHeight, output, outputWidth, outputHeight, multiplier, range.x);
		else
			return cudaErrorInvalidValue;
	} 
	else
	{
		// Mono8 option - 1 channel grayscale image, in either uint8_t or float format
		if (format == imageFormat::IMAGE_MONO8)
			gpuDecoderPreprocess<uint8_t, isBGR, isGray, threeOutputChannels><<<gridDim, blockDim, 0, stream>>>(m_dH, (uint8_t *)input, inputWidth, inputHeight, output, outputWidth, outputHeight, multiplier, range.x);
		else if (format == imageFormat::IMAGE_MONO32F)
			gpuDecoderPreprocess<float, isBGR, isGray, threeOutputChannels><<<gridDim, blockDim, 0, stream>>>(m_dH, (float *)input, inputWidth, inputHeight, output, outputWidth, outputHeight, multiplier, range.x);
		else
			return cudaErrorInvalidValue;
	}

	return cudaGetLastError();
}

// cudaSSDPreprocessRGB
cudaError_t DecoderPreprocess::decoderPreprocessRGB(const float *hH, void *input, imageFormat format, size_t inputWidth, size_t inputHeight,
													float *output, size_t outputWidth, size_t outputHeight,
													const float2 &range, cudaStream_t stream)
{
	return launchDecoderPreprocess<false, false, false>(hH, input, format, inputWidth, inputHeight, output, outputWidth, outputHeight, range, stream);
}

// cudaSSDPreprocessBGR
cudaError_t DecoderPreprocess::decoderPreprocessBGR(const float *hH, void *input, imageFormat format, size_t inputWidth, size_t inputHeight,
													float *output, size_t outputWidth, size_t outputHeight,
													const float2 &range, cudaStream_t stream)
{
	return launchDecoderPreprocess<true, false, false>(hH, input, format, inputWidth, inputHeight, output, outputWidth, outputHeight, range, stream);
}

cudaError_t DecoderPreprocess::decoderPreprocessGray(const float *hH, void *input, imageFormat format, size_t inputWidth, size_t inputHeight,
													 float *output, size_t outputWidth, size_t outputHeight,
													 const float2 &range, cudaStream_t stream)
{
	return launchDecoderPreprocess<false, true, false>(hH, input, format, inputWidth, inputHeight, output, outputWidth, outputHeight, range, stream);
}
cudaError_t DecoderPreprocess::decoderPreprocessGrayForThreeChannelOutput(const float *hH, void *input, imageFormat format, size_t inputWidth, size_t inputHeight,
													 float *output, size_t outputWidth, size_t outputHeight,
													 const float2 &range, cudaStream_t stream)
{
	return launchDecoderPreprocess<false, true, true>(hH, input, format, inputWidth, inputHeight, output, outputWidth, outputHeight, range, stream);
}