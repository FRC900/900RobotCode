#include "ros/console.h"
#include "base_trajectory/cuda_utils.h"

void cudaSafeCallWrapper(cudaError err, const char* file, const int line)
{
	if (cudaSuccess != err)
	{
		ROS_ERROR_STREAM("CUDA error : " <<
						 "\nFile: " << file <<
						 "\nLine Number: " << line <<
						 "\nReason:" << cudaGetErrorString(err));
		//cudaDeviceReset();
		//exit(EXIT_FAILURE);
	}
}

size_t numBlocks(size_t threads_per_block, size_t N)
{
	return (N + threads_per_block - 1) / threads_per_block;
}


bool hasCudaGPU(void)
{
	int deviceCount;
	cudaError_t cudaResultCode = cudaGetDeviceCount(&deviceCount);
	if (cudaResultCode != cudaSuccess)
		deviceCount = 0;
	/* machines with no GPUs can still report one emulation device */
	int gpuDeviceCount = 0;
	for (int device = 0; device < deviceCount; ++device) {
		struct cudaDeviceProp properties;
		cudaGetDeviceProperties(&properties, device);
		if (properties.major != 9999) /* 9999 means emulation only */
			++gpuDeviceCount;
	}

	return (gpuDeviceCount > 0);
}
