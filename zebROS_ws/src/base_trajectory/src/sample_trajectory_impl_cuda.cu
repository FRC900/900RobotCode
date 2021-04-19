// CUDA implementation of trajectory sampling code
// There are 2 areas which can be parallelized
// - the search for the time for each equally-spaced position along the arclength curve.  Each time being searched for is independent, so all of the searches for each for the equally-spaced times can be run in parallel
// - evaluating the x,y,theta state at each equally arclength-spaced time along the the curve.  Again, these are totally independent so the searches can be run in parallel.

#include "ros/console.h"
#include <functional>
#include <cuda_profiler_api.h>
#include <nvToolsExt.h>
#include "base_trajectory/cuda_mem_utils.cuh"
#include "base_trajectory/cuda_spline_utils.cuh"
#include "base_trajectory/cuda_utils.h"
#include "base_trajectory/sample_trajectory_impl_cuda.h"

// Helper class to hold equal arc length times and positions calculated
// by FindArclengthTime functor below.
template <typename T>
class TimeAndPosition
{
	public:
		__host__ __device__
		TimeAndPosition()
			: time_(-1.0f)
		    , position_(-1.0f)
		{
		}
		__host__ __device__
		TimeAndPosition(const T &time, const T &position)
			: time_(time)
		    , position_(position)
		{
		}
		T time_;
		T position_;
};

template <typename T>
__global__
void findArclengthTime(TimeAndPosition<T> *timeAndPosition,
					   const CudaSpan<T>   trajectoryIn,
					   const size_t        trajectoryElementsPerThread,
					   const T             distBetweenArcLength,
					   const T             distBetweenArcLengthEpsilon,
					   const size_t        N,
					   const T             endTime,
					   const T             endPosition)
{
	extern __shared__ float trajectory[];
	// Move trajectory info from global to shared mem to maybe speed up processing?
	for (size_t i = 0; i < trajectoryElementsPerThread; i++)
	{
		const auto index = i * blockDim.x + threadIdx.x;
		if (index < trajectoryIn.size_)
		{
			trajectory[index] = trajectoryIn.array_[index];
		}
	}
	const size_t tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= N)
		return;
	__syncthreads();
	auto &ret = timeAndPosition[tid];
	const T goalPosition = min(tid * distBetweenArcLength, endPosition);
	// Special case for the first and last position since we already know the answer
	if (tid == 0)
	{
		ret = TimeAndPosition<T>(0.0f, 0.0f);
		return;
	}
	else if (tid == (N - 1))
	{
		ret = TimeAndPosition<T>(endTime, endPosition);
		return;
	}

	// Binary search to get position within tolerance of desired cumulative distance
	// Return time and actual position at that time
	T start{0.0f};
	T end  {endTime};
	T mid  {end / 2.0f};
	T sampledPosition;
	while((end - mid) > 0.00001f) // Quit if time delta gets too small
	{
		sampleSplinePosition<ARCLENGTH_TRAJECTORY_ORDER>(trajectory, trajectoryIn.size_, mid, sampledPosition);

		const auto delta = goalPosition - sampledPosition;
		if (fabs(delta) < distBetweenArcLengthEpsilon)
		{
			ret = TimeAndPosition<T>(mid, sampledPosition);
			return; // If the sampled position is close enough save the time value
		}

		// Otherwise, continue the binary search in the correct direction
		if (delta > 0)
		{
			start = mid;
		}
		else
		{
			end = mid;
		}
		mid = (start + end) / 2.0f;
	}
}


// For the time specified in timeAndPosition, sample
// the x, y and theta splines to get their
// position, velocity and acceleration at that time
template <class T>
__global__
void sampleTrajectories(ThreeDState<T> *threeDState,
						const TimeAndPosition<T> *timeAndPosition,
						const CudaSpan<T>         xTrajectory,
						const CudaSpan<T>         yTrajectory,
						const CudaSpan<T>         tTrajectory,
						const float               endTime,
						const size_t              N)

{
	const size_t tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= N)
		return;
	const auto time = timeAndPosition[tid].time_;
	const auto seg  = arcLengthTimeToSegTime(time, endTime);
	auto &state     = threeDState[tid];

	sampleSplineSeg<5>(xTrajectory, seg, time, state.x_);
	sampleSplineSeg<5>(yTrajectory, seg, time, state.y_);
	sampleSplineSeg<5>(tTrajectory, seg, time, state.t_);
}

template <class T>
SampleTrajectoryImplCuda<T>::SampleTrajectoryImplCuda(double distBetweenArcLength,
								double distBetweenArcLengthEpsilon,
								double midTimeInflation)
	: SampleTrajectoryImpl<T>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation)
{
	cudaSafeCall(cudaStreamCreateWithFlags(&findArcLengthStream_, cudaStreamNonBlocking));
	cudaSafeCall(cudaStreamCreateWithFlags(&sampleXYTStream_, cudaStreamNonBlocking));
	cudaSafeCall(cudaStreamCreateWithFlags(&graphStream_, cudaStreamNonBlocking));
	cudaSafeCall(cudaEventCreate(&findArcLengthEvent_));
	cudaSafeCall(cudaEventCreate(&sampleD2HMemcpyEvent_));
	cudaSafeCall(cudaProfilerStart());
}

template <class T>
SampleTrajectoryImplCuda<T>::~SampleTrajectoryImplCuda()
{
	cudaSafeCall(cudaStreamDestroy(findArcLengthStream_));
	cudaSafeCall(cudaStreamDestroy(sampleXYTStream_));
	cudaSafeCall(cudaStreamDestroy(graphStream_));
	cudaSafeCall(cudaEventDestroy(findArcLengthEvent_));
	cudaSafeCall(cudaEventDestroy(sampleD2HMemcpyEvent_));
	cudaSafeCall(cudaProfilerStop());
	if (hTimeAndPositions_) cudaSafeCall(cudaFree(hTimeAndPositions_));
	if (dTimeAndPositions_) cudaSafeCall(cudaFree(dTimeAndPositions_));
	if (hArcLengthTrajectory_) cudaSafeCall(cudaFree(hArcLengthTrajectory_));
	if (dArcLengthTrajectory_) cudaSafeCall(cudaFree(dArcLengthTrajectory_));
	if (hXTrajectory_) cudaSafeCall(cudaFree(hXTrajectory_));
	if (dXTrajectory_) cudaSafeCall(cudaFree(dXTrajectory_));
	if (hYTrajectory_) cudaSafeCall(cudaFree(hYTrajectory_));
	if (dYTrajectory_) cudaSafeCall(cudaFree(dYTrajectory_));
	if (hTTrajectory_) cudaSafeCall(cudaFree(hTTrajectory_));
	if (dTTrajectory_) cudaSafeCall(cudaFree(dTTrajectory_));
	if (hThreeDStates_) cudaSafeCall(cudaFree(hThreeDStates_));
	if (dThreeDStates_) cudaSafeCall(cudaFree(dThreeDStates_));
}

#if 0

// Called after findArcLength and associated D2H memcpy are finished
template <class T>
void CUDART_CB postprocEqualArcLengthAndTime(void *,
											std::vector<T> &equalArcLengthTimes,
											std::vector<T> &equalArcLengthPositions)
{
	// Resize return buffers for equalArcLength time and position arrays
	nvtxRangePushA("::resizeEqualArcLengthTime");
	equalArcLengthTimes.resize(idealEqualPositionsSize);
	equalArcLengthPositions.resize(idealEqualPositionsSize);
	nvtxRangePop();

	// Since data is ready, copy from the host buffer into the function argument buffers
	nvtxRangePushA("::equalArcLengthTime");
	for (size_t i = 0; i < idealEqualPositionsSize; i++)
	{
		equalArcLengthTimes[i] = hTimeAndPositions_[i].time_;
		equalArcLengthPositions[i] = hTimeAndPositions_[i].position_;
	}
	nvtxRangePop();
}

// Called after sampleTrajectoryies and associated D2H memcpy are finished
template <class T>
void CUDART_CB postprocThreeDStates(void *,
									std::vector<SegmentState<T>> &xStates,
									std::vector<SegmentState<T>> &yStates,
									std::vector<SegmentState<T>> &tStates)
{
	// Convert the device-formatted sampleTrajectories results back to std::vectors
	// so they can be returned to the calling code
	nvtxRangePushA("::resizeThreeDStates_");
	xStates.resize(idealEqualPositionsSize);
	yStates.resize(idealEqualPositionsSize);
	tStates.resize(idealEqualPositionsSize);
	nvtxRangePop();

	nvtxRangePushA("::postprocThreeDStates_");
	for (size_t i = 0; i < idealEqualPositionsSize; i++)
	{
		const auto &s = hThreeDStates_[i];
		appendState(xStates[i], s.x_);
		appendState(yStates[i], s.y_);
		appendState(tStates[i], s.t_);
	}
	nvtxRangePop();
}
#endif

template <class T>
bool SampleTrajectoryImplCuda<T>::sample(std::vector<T> &equalArcLengthTimes,
										std::vector<T> &equalArcLengthPositions,
										std::vector<SegmentState<T>> &xStates,
										std::vector<SegmentState<T>> &yStates,
										std::vector<SegmentState<T>> &tStates,
										const XYTTrajectory<T> &trajectory,
										const ArcLengthTrajectory<T> &arcLengthTrajectory)
{
	nvtxRangePushA(__FUNCTION__); // add a top level function tag

	nvtxRangePushA("::allocDTimeAndPosition");
	// Sample arclength end point info
	const CudaT endTime = arcLengthTrajectory[0].back().endTime();
	T endPosition;
	trajectory_interface::samplePosition(arcLengthTrajectory[0], endTime, endPosition);

	const size_t idealEqualPositionsSize = ceil(endPosition / this->distBetweenArcLength_) + 1;

	if (reallocDeviceMemory(dTimeAndPositions_, dTimeAndPositionsCapacity_, sizeof(TimeAndPosition<CudaT>) * idealEqualPositionsSize))
	{
		recordCudaGraph_ = true;
	}
	nvtxRangePop();

	// Alloc space in device mem for output of sampleTrajectories
	nvtxRangePushA("::allocDThreeDStates");
	if (reallocDeviceMemory(dThreeDStates_, dThreeDStatesCapacity_, sizeof(ThreeDState<CudaT>) * idealEqualPositionsSize))
	{
		recordCudaGraph_ = true;
	}
	nvtxRangePop();

	nvtxRangePushA("::allocHTimeAndPosition");
	if (reallocHostPinnedMemory(hTimeAndPositions_, hTimeAndPositionsCapacity_, sizeof(TimeAndPosition<CudaT>) * idealEqualPositionsSize))
	{
		recordCudaGraph_ = true;
	}
	nvtxRangePop();

	// Allocate host buffer for sampleTrajectories result
	nvtxRangePushA("::allocHThreeDStates_");
	if (reallocHostPinnedMemory(hThreeDStates_, hThreeDStatesCapacity_, sizeof(ThreeDState<CudaT>) * idealEqualPositionsSize))
	{
		recordCudaGraph_ = true;
	}
	nvtxRangePop();

	nvtxRangePushA("::h2d_arclengthtrajectory");
	// Convert C++ nested vectors of vectors of vectors spline representation
	// into a single flat array for use in CUDA host code
	size_t arclengthLen;
	if (hostToDeviceTrajectory(hArcLengthTrajectory_, hArcLengthTrajectoryCapacity_,
							   dArcLengthTrajectory_, dArcLengthTrajectoryCapacity_,
							   arclengthLen, arcLengthTrajectory[0]))
	{
		recordCudaGraph_ = true;
	}
	nvtxRangePop();
	nvtxRangePushA("::h2d_xytstate");
	size_t trajectoryLen;
	// Prep x, y, theta trajectories for use in device code
	// Putting these here should run the host side prep
	// in parallel with the arc length kernel on the GPU, above
	if (hostToDeviceTrajectory(hXTrajectory_, hXTrajectoryCapacity_,
							   dXTrajectory_, dXTrajectoryCapacity_,
							   trajectoryLen, trajectory[0]))
	{
		recordCudaGraph_ = true;
	}
	if (hostToDeviceTrajectory(hYTrajectory_, hYTrajectoryCapacity_,
							   dYTrajectory_, dYTrajectoryCapacity_,
							   trajectoryLen, trajectory[1]))
	{
		recordCudaGraph_ = true;
	}
	if (hostToDeviceTrajectory(hTTrajectory_, hTTrajectoryCapacity_,
							   dTTrajectory_, dTTrajectoryCapacity_,
							   trajectoryLen, trajectory[2]))
	{
		recordCudaGraph_ = true;
	}
	nvtxRangePop();

	// start recording cudagraph here
	if (recordCudaGraph_)
	{
		cudaSafeCall(cudaStreamBeginCapture(findArcLengthStream_, cudaStreamCaptureModeGlobal));
		nvtxRangePushA("::memcpyArcLengthTrajectory");
		cudaSafeCall(cudaMemcpyAsync(dArcLengthTrajectory_,
									 hArcLengthTrajectory_,
									 arclengthLen * sizeof(CudaT),
									 cudaMemcpyHostToDevice,
									 findArcLengthStream_));
		nvtxRangePop();
		nvtxRangePushA("::submitFindArcLengthAndTime");
		// Run findArcLengthAndTime() on each element of dIdealEqualPositions,
		// store result in dTimeAndPositions vector
		constexpr size_t THREADS_PER_BLOCK = 128;
		const size_t dArcLengthTrajectoryLength = idealEqualPositionsSize * (arcLengthTrajectory[0][0].size() + 3);
		const size_t trajectoryElementsPerThread = (dArcLengthTrajectoryLength + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
		//ROS_INFO_STREAM("THREADS_PER_BLOCK = " << THREADS_PER_BLOCK << " numBlocks = " << numBlocks(THREADS_PER_BLOCK, idealEqualPositionsSize));
		//ROS_INFO_STREAM("dArcLengthTrajectory_ size = " << idealEqualPositionsSize * (arcLengthTrajectory[0][0].size() + 3));
		findArclengthTime<CudaT><<<numBlocks(THREADS_PER_BLOCK, idealEqualPositionsSize), THREADS_PER_BLOCK, sizeof(dArcLengthTrajectory_[0]) * dArcLengthTrajectoryLength, findArcLengthStream_>>>(
				dTimeAndPositions_,
				CudaSpan<CudaT>(dArcLengthTrajectory_, dArcLengthTrajectoryLength),
				trajectoryElementsPerThread,
				this->distBetweenArcLength_,
				this->distBetweenArcLengthEpsilon_,
				idealEqualPositionsSize,
				endTime,
				endPosition);
		// Record event to indicate findArcLengthTime kernel has completed.
		// Will be checked before running the sampleTrajectory kernels
		// to be sure that dTimeAndPosition is filled in correctly
		cudaSafeCall(cudaEventRecord(findArcLengthEvent_, findArcLengthStream_));
		nvtxRangePop();

		nvtxRangePushA("::h2d_xytstate");
		cudaSafeCall(cudaMemcpyAsync(dXTrajectory_,
									 hXTrajectory_,
									 sizeof(CudaT) * trajectoryLen,
									 cudaMemcpyDeviceToHost,
									 sampleXYTStream_));
		nvtxRangePop();
		nvtxRangePushA("::memcpyArcLengthTrajectory");
		cudaSafeCall(cudaMemcpyAsync(dYTrajectory_,
									 hYTrajectory_,
									 sizeof(CudaT) * trajectoryLen,
									 cudaMemcpyDeviceToHost,
									 sampleXYTStream_));
		nvtxRangePop();
		nvtxRangePushA("::memcpyArcLengthTrajectory");
		cudaSafeCall(cudaMemcpyAsync(dTTrajectory_,
									 hTTrajectory_,
									 sizeof(CudaT) * trajectoryLen,
									 cudaMemcpyDeviceToHost,
									 sampleXYTStream_));
		nvtxRangePop();

		// Extract device results from findArclengthAndTime into
		// correct format to return to caller
		// Queue up copy of results to host array
		nvtxRangePushA("::memcpyHTimeAndPosition");
		cudaSafeCall(cudaMemcpyAsync(hTimeAndPositions_,
									 dTimeAndPositions_,
									 sizeof(TimeAndPosition<CudaT>) * idealEqualPositionsSize,
									 cudaMemcpyDeviceToHost,
									 findArcLengthStream_));
		nvtxRangePop();

		// TODO - launch host function to copy from hTimeAndPosision into equalArchLength Time/Positions return vecs

		// Set a barrier on sampleXYTStream so the code pauses until findArcLength finishes
		// sample uses findArcLength data, so the latter must finish before the former can start
		cudaSafeCall(cudaStreamWaitEvent(sampleXYTStream_, findArcLengthEvent_, 0));

		// For each time in dTimeAndPositions, run sampleTrajectory
		// using that time and store the result in dThreeDState.
		// Run this on a second stream, the event wait from a few lines up will
		// make sure data from findArcLength is done before starting this kernel
		nvtxRangePushA("::submitSampleTrajectories");
		sampleTrajectories<<<numBlocks(THREADS_PER_BLOCK, idealEqualPositionsSize), THREADS_PER_BLOCK, 0, sampleXYTStream_>>>
			(dThreeDStates_, dTimeAndPositions_,
			 CudaSpan<CudaT>(dXTrajectory_, trajectory[0].size() * (trajectory[0][0].size() + 3)),
			 CudaSpan<CudaT>(dYTrajectory_, trajectory[1].size() * (trajectory[1][0].size() + 3)),
			 CudaSpan<CudaT>(dTTrajectory_, trajectory[2].size() * (trajectory[2][0].size() + 3)),
			 endTime, idealEqualPositionsSize);
		nvtxRangePop();

		// Queue up memcpy of results of sampleTrajectories result from device to host
		nvtxRangePushA("::memcpyThreeDStates_");
		cudaSafeCall(cudaMemcpyAsync(hThreeDStates_,
									 dThreeDStates_,
									 sizeof(ThreeDState<CudaT>) * idealEqualPositionsSize,
									 cudaMemcpyDeviceToHost,
									 sampleXYTStream_));
		cudaSafeCall(cudaEventRecord(sampleD2HMemcpyEvent_, sampleXYTStream_));
		cudaSafeCall(cudaStreamWaitEvent(findArcLengthStream_, sampleD2HMemcpyEvent_, 0));
		nvtxRangePop();

		// TODO - launch host function to copy from hThreeDStates into x,y,t states return vectors

		cudaSafeCall(cudaStreamEndCapture(findArcLengthStream_, &cudaGraph_));
		cudaSafeCall(cudaGraphInstantiate(&cudaGraphExec_, cudaGraph_, 0, 0, 0));
		recordCudaGraph_ = false;
		ROS_INFO_STREAM("Graph created");
	}

	cudaSafeCall(cudaGraphLaunch(cudaGraphExec_, graphStream_));

	// Resize return buffers for equalArcLength time and position arrays
	nvtxRangePushA("::resizeEqualArcLengthTime");
	equalArcLengthTimes.resize(idealEqualPositionsSize);
	equalArcLengthPositions.resize(idealEqualPositionsSize);
	nvtxRangePop();

	// Convert the device-formatted sampleTrajectories results back to std::vectors
	// so they can be returned to the calling code
	nvtxRangePushA("::resizeThreeDStates_");
	xStates.resize(idealEqualPositionsSize);
	yStates.resize(idealEqualPositionsSize);
	tStates.resize(idealEqualPositionsSize);
	nvtxRangePop();

	cudaSafeCall(cudaStreamSynchronize(graphStream_));

	// Since data is ready, copy from the host buffer into the function argument buffers
	nvtxRangePushA("::equalArcLengthTime");
	for (size_t i = 0; i < idealEqualPositionsSize; i++)
	{
		equalArcLengthTimes[i] = hTimeAndPositions_[i].time_;
		equalArcLengthPositions[i] = hTimeAndPositions_[i].position_;
	}
	nvtxRangePop();

	nvtxRangePushA("::postprocThreeDStates_");
	for (size_t i = 0; i < idealEqualPositionsSize; i++)
	{
		const auto &s = hThreeDStates_[i];
		appendState(xStates[i], s.x_);
		appendState(yStates[i], s.y_);
		appendState(tStates[i], s.t_);
	}
	nvtxRangePop();

	nvtxRangePop(); // top-level function

	return true;
}

template class SampleTrajectoryImplCuda<double>;
//template class SampleTrajectoryImplCuda<float>;
