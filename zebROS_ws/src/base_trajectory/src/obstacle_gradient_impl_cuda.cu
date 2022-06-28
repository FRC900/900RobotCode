#include <limits>
#include "ros/console.h"
//#include "cuda_profiler_api.h"
#include "base_trajectory/cuda_utils.h"
#include "base_trajectory/obstacle_gradient_impl_cuda.h"

ObstacleGradientImplCuda::ObstacleGradientImplCuda(int xs, int ys, uint8_t threshold)
	: ObstacleGradientImpl(xs, ys, threshold)
{
	ns_ = 0;
	setSize(xs, ys);
	cudaSafeCall(cudaStreamCreateWithFlags(&cudaStreamH_, cudaStreamNonBlocking));
	cudaSafeCall(cudaStreamCreateWithFlags(&cudaStreamV_, cudaStreamNonBlocking));
	cudaSafeCall(cudaStreamCreateWithFlags(&cudaStreamForGraph_, cudaStreamNonBlocking));
	cudaSafeCall(cudaEventCreate(&cudaEventInputReady_));
	cudaSafeCall(cudaEventCreate(&cudaEventHKernel_));
	cudaSafeCall(cudaEventCreate(&cudaEventVKernel_));
}

ObstacleGradientImplCuda::~ObstacleGradientImplCuda()
{
	freeDeviceMemory();
	cudaSafeCall(cudaStreamDestroy(cudaStreamH_));
	cudaSafeCall(cudaStreamDestroy(cudaStreamV_));
	cudaSafeCall(cudaStreamDestroy(cudaStreamForGraph_));
	cudaSafeCall(cudaEventDestroy(cudaEventInputReady_));
	cudaSafeCall(cudaEventDestroy(cudaEventHKernel_));
	cudaSafeCall(cudaEventDestroy(cudaEventVKernel_));
	//cudaDeviceReset();
	//cudaProfilerStop();
}

__global__
void calcHorizontal(const int nx, int ny, const unsigned char* costs, const uint8_t threshold, const unsigned int maxUInt, unsigned int* hPotentials)
{
	// Each thread handles an individual row
	const int row = blockIdx.x * blockDim.x + threadIdx.x;
	if (row >= ny)
		return;
	const int startIdx = row * nx;
	for (int i = startIdx; i < startIdx + nx; i++)
	{
		if (costs[i] <= threshold)
		{
			hPotentials[i] = 0;
		}
		else
		{
			hPotentials[i] = maxUInt;
		}
	}
	// For this row iteratively propagate potentials from left and right of each cell
	for (int i = startIdx + 1; i < (startIdx + nx); i++)
	{
		hPotentials[i] = min(hPotentials[i-1] + 1, hPotentials[i]);
	}
	// And then repeat moving right to left
	for (int i = startIdx + nx - 2; i >= startIdx; i--)
	{
		hPotentials[i] = min(hPotentials[i+1] + 1, hPotentials[i]);
	}
}

__global__
void calcVertical(const int nx, const int ny, const unsigned char* costs, const uint8_t threshold, const unsigned int maxUInt, unsigned int* vPotentials)
{
	// Each thread handles an individual col
	const int col = blockIdx.x * blockDim.x + threadIdx.x;
	if (col >= nx)
		return;
	const int startIdx = col;
	for (int row = 0; row < ny; row++)
	{
		const auto i = startIdx + row * nx;
		if (costs[i] <= threshold)
		{
			vPotentials[i] = 0;
		}
		else
		{
			vPotentials[i] = maxUInt;
		}
	}

	for (int row = 1; row < ny; row++)
	{
		const auto i = startIdx + row * nx;
		vPotentials[i] = min(vPotentials[i-nx] + 1, vPotentials[i]);
	}
	for (int row = ny - 2; row >= 0; row--)
	{
		const auto i = startIdx + row * nx;
		vPotentials[i] = min(vPotentials[i+nx] + 1, vPotentials[i]);
	}
}

__global__
void sumHorizonalVertical(const int nx, const int ny, unsigned int* hPotentials, const unsigned int* vPotentials, const uint8_t *costs)
{
	// Each thread handles an individual row
	const int row = blockIdx.x * blockDim.x + threadIdx.x;
	if (row >= ny)
		return;

	const int startIdx = row * nx;
	for (int i = startIdx; i < startIdx + nx; i++)
	{
		hPotentials[i] += vPotentials[i];
		hPotentials[i] *= costs[i];
	}
}

// Given the grid map of size nx_, ny_ containing costs costs, create a potential
// map of the same space. Each grid potential is defined to be 0 if there is no
// obstacle at the grid location.  If there is an obstacle, the potential is the
// sum of the shortest horizontal and vertical distances to a grid location with
// no obstacle. This will tend to produce potential fields which look like pyramids,
// with a hopefully easy to follow gradient from obstacles into cells which are
// not obstructed.
void ObstacleGradientImplCuda::calculatePotentials(const unsigned char* costs, unsigned int* potentials)
{
	constexpr size_t THREADS_PER_BLOCK = 128;
	// memcpy h2d costs
	if (recordCudaGraph_)
	{
		cudaSafeCall(cudaStreamBeginCapture(cudaStreamH_, cudaStreamCaptureModeGlobal));
		cudaSafeCall(cudaMemcpyAsync(deviceCosts_, costs, ns_ * sizeof(*costs), cudaMemcpyHostToDevice, cudaStreamH_));
		cudaSafeCall(cudaEventRecord(cudaEventInputReady_, cudaStreamH_));
		// calc horizontal
		calcHorizontal<<<numBlocks(THREADS_PER_BLOCK, ny_), THREADS_PER_BLOCK, 0, cudaStreamH_>>>(nx_, ny_, deviceCosts_, threshold_, std::numeric_limits<unsigned int>::max(), deviceHPotentials_);
		cudaSafeCall(cudaEventRecord(cudaEventHKernel_, cudaStreamH_));
		// calc vertical
		cudaSafeCall(cudaStreamWaitEvent(cudaStreamV_, cudaEventInputReady_, 0));
		calcVertical<<<numBlocks(THREADS_PER_BLOCK, nx_), THREADS_PER_BLOCK, 0, cudaStreamV_>>>(nx_, ny_, deviceCosts_, threshold_, std::numeric_limits<unsigned int>::max(), deviceVPotentials_);
		cudaSafeCall(cudaEventRecord(cudaEventVKernel_, cudaStreamV_));

		// sum h+v into h once both the sub-sums are complete
		cudaSafeCall(cudaStreamWaitEvent(cudaStreamH_, cudaEventHKernel_, 0)); // redundant?
		cudaSafeCall(cudaStreamWaitEvent(cudaStreamH_, cudaEventVKernel_, 0));
		sumHorizonalVertical<<<numBlocks(THREADS_PER_BLOCK, ny_), THREADS_PER_BLOCK, 1, cudaStreamH_>>>(nx_, ny_, deviceHPotentials_, deviceVPotentials_, deviceCosts_);

		// memcpy async d2h device Hpotentials into host potentials
		cudaSafeCall(cudaMemcpyAsync(potentials, deviceHPotentials_, ns_ * sizeof(*potentials), cudaMemcpyDeviceToHost, cudaStreamH_));
		cudaSafeCall(cudaStreamEndCapture(cudaStreamH_, &cudaGraph_));
		cudaSafeCall(cudaGraphInstantiate(&cudaGraphExec_, cudaGraph_, 0, 0, 0));
		recordCudaGraph_ = false;
		ROS_INFO_STREAM("Graph created");
	}

	cudaSafeCall(cudaGraphLaunch(cudaGraphExec_, cudaStreamForGraph_));
	cudaSafeCall(cudaStreamSynchronize(cudaStreamForGraph_));
}

void ObstacleGradientImplCuda::setSize(int nx, int ny)
{
	const auto ns = ns_;
	ObstacleGradientImpl::setSize(nx, ny);

	if (ns != ns_)
	{
		freeDeviceMemory();
		cudaSafeCall(cudaMalloc(&deviceHPotentials_, ns_ * sizeof(*deviceHPotentials_)));
		cudaSafeCall(cudaMalloc(&deviceVPotentials_, ns_ * sizeof(*deviceVPotentials_)));
		cudaSafeCall(cudaMalloc(&deviceCosts_, ns_ * sizeof(*deviceCosts_)));
		recordCudaGraph_ = true;
	}
}

void ObstacleGradientImplCuda::freeDeviceMemory()
{
	if (deviceHPotentials_)
	{
		cudaSafeCall(cudaFree(deviceHPotentials_));
		deviceHPotentials_ = nullptr;
	}
	if (deviceVPotentials_)
	{
		cudaSafeCall(cudaFree(deviceVPotentials_));
		deviceVPotentials_ = nullptr;
	}
	if (deviceCosts_)
	{
		cudaSafeCall(cudaFree(deviceCosts_));
		deviceCosts_ = nullptr;
	}
}
