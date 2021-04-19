// CUDA implementation of trajectory sampling code
// There are 2 areas which can be parallelized
// - the search for the time for each equally-spaced position along the arclength curve.  Each time being searched for is independent, so all of the searches for each for the equally-spaced times can be run in parallel
// - evaluating the x,y,theta state at each equally arclength-spaced time along the the curve.  Again, these are totally independent so the searches can be run in parallel.

#include "ros/console.h"
#include <cuda_profiler_api.h>
#include <nvToolsExt.h>
#include "base_trajectory/sample_trajectory_impl_hybrid.h"
#include "base_trajectory/cuda_mem_utils.cuh"
#include "base_trajectory/cuda_spline_utils.cuh"
#include "base_trajectory/cuda_utils.h"

// For the time specified in timeAndPosition, sample
// the x, y and theta splines to get their
// position, velocity and acceleration at that time
template <class T>
__global__
void sampleTrajectories(ThreeDState<T> *threeDState,
						const CudaT *times,
						const CudaSpan<T>         xTrajectory,
						const CudaSpan<T>         yTrajectory,
						const CudaSpan<T>         tTrajectory,
						const float               endTime,
						const size_t              N)

{
	const size_t tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= N)
		return;
	const auto time = times[tid];
	const auto seg  = arcLengthTimeToSegTime(time, endTime);
	auto &state     = threeDState[tid];

	sampleSplineSeg<5>(xTrajectory, seg, time, state.x_);
	sampleSplineSeg<5>(yTrajectory, seg, time, state.y_);
	sampleSplineSeg<5>(tTrajectory, seg, time, state.t_);
}

template <class T>
SampleTrajectoryImplHybrid<T>::SampleTrajectoryImplHybrid(double distBetweenArcLength,
								double distBetweenArcLengthEpsilon,
								double midTimeInflation)
	: SampleTrajectoryImplCpu<T>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation)
{
	cudaSafeCall(cudaStreamCreateWithFlags(&sampleXYTStream_, cudaStreamNonBlocking));
	cudaSafeCall(cudaProfilerStart());
}

template <class T>
SampleTrajectoryImplHybrid<T>::~SampleTrajectoryImplHybrid()
{
	cudaSafeCall(cudaProfilerStop());
	cudaSafeCall(cudaStreamDestroy(sampleXYTStream_));
	if (hTime_) cudaSafeCall(cudaFree(hTime_));
	if (dTime_) cudaSafeCall(cudaFree(dTime_));
	if (hXTrajectory_) cudaSafeCall(cudaFree(hXTrajectory_));
	if (dXTrajectory_) cudaSafeCall(cudaFree(dXTrajectory_));
	if (hYTrajectory_) cudaSafeCall(cudaFree(hYTrajectory_));
	if (dYTrajectory_) cudaSafeCall(cudaFree(dYTrajectory_));
	if (hTTrajectory_) cudaSafeCall(cudaFree(hTTrajectory_));
	if (dTTrajectory_) cudaSafeCall(cudaFree(dTTrajectory_));
	if (hThreeDStates_) cudaSafeCall(cudaFree(hThreeDStates_));
	if (dThreeDStates_) cudaSafeCall(cudaFree(dThreeDStates_));
}

template <class T>
bool SampleTrajectoryImplHybrid<T>::sample(std::vector<T> &equalArcLengthTimes,
										std::vector<T> &equalLengthPositions,
										std::vector<SegmentState<T>> &xStates,
										std::vector<SegmentState<T>> &yStates,
										std::vector<SegmentState<T>> &tStates,
										const XYTTrajectory<T> &trajectory,
										const ArcLengthTrajectory<T> &arcLengthTrajectory)
{
	nvtxRangePushA(__FUNCTION__);
	// Prep x, y, theta trajectories for use in device code
	// Putting this first will run the memcpy's in parallel
	// with the CPU subdivideLength call below
	nvtxRangePushA("::h2d_xytstate");
	size_t len;
	hostToDeviceTrajectory(hXTrajectory_, hXTrajectoryCapacity_,
						   dXTrajectory_, dXTrajectoryCapacity_,
						   len, trajectory[0]);
	hostToDeviceTrajectory(hYTrajectory_, hYTrajectoryCapacity_,
						   dYTrajectory_, dYTrajectoryCapacity_,
						   len, trajectory[1]);
	hostToDeviceTrajectory(hTTrajectory_, hTTrajectoryCapacity_,
						   dTTrajectory_, dTTrajectoryCapacity_,
						   len, trajectory[2]);
	nvtxRangePop();
	nvtxRangePushA("::memcpyXTrajectory");
	cudaSafeCall(cudaMemcpyAsync(dXTrajectory_,
								 hXTrajectory_,
								 sizeof(CudaT) * len,
								 cudaMemcpyDeviceToHost,
								 sampleXYTStream_));
	nvtxRangePop();
	nvtxRangePushA("::memcpyYTrajectory");
	cudaSafeCall(cudaMemcpyAsync(dYTrajectory_,
								 hYTrajectory_,
								 sizeof(CudaT) * len,
								 cudaMemcpyDeviceToHost,
								 sampleXYTStream_));
	nvtxRangePop();
	nvtxRangePushA("::memcpyTTrajectory");
	cudaSafeCall(cudaMemcpyAsync(dTTrajectory_,
								 hTTrajectory_,
								 sizeof(CudaT) * len,
								 cudaMemcpyDeviceToHost,
								 sampleXYTStream_));
	nvtxRangePop();

	nvtxRangePushA("::subdivideLength");
	if (!this->subdivideLength(equalArcLengthTimes, equalLengthPositions, arcLengthTrajectory))
		return false;
	nvtxRangePop();

	// Queue up copy of results to host array
	nvtxRangePushA("::reallocHTime");
	reallocHostPinnedMemory(hTime_, hTimeCapacity_, sizeof(CudaT) * equalArcLengthTimes.size());
	nvtxRangePop();
	nvtxRangePushA("::setHTime");
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		hTime_[i] = equalArcLengthTimes[i];
	}
	nvtxRangePop();
	nvtxRangePushA("::reallocDTime");
	reallocDeviceMemory(dTime_, dTimeCapacity_, sizeof(CudaT) * equalArcLengthTimes.size());
	nvtxRangePop();
	nvtxRangePushA("::memcpyDTime");
	cudaSafeCall(cudaMemcpyAsync(dTime_,
			hTime_,
			sizeof(CudaT) * equalArcLengthTimes.size(),
			cudaMemcpyHostToDevice,
			sampleXYTStream_));
	nvtxRangePop();
	nvtxRangePushA("::reallocDThreeDStates");
	reallocDeviceMemory(dThreeDStates_, dThreeDStatesCapacity_, sizeof(ThreeDState<CudaT>) * equalArcLengthTimes.size());
	nvtxRangePop();

	// For each time in dTimeAndPositions, run sampleTrajectory
	// using that time and store the result in dThreeDState.
	// Run this on a second stream, event wait for findarclengthandtime event
	constexpr size_t THREADS_PER_BLOCK = 128;
	sampleTrajectories<<<numBlocks(THREADS_PER_BLOCK, equalArcLengthTimes.size()), THREADS_PER_BLOCK, 0, sampleXYTStream_>>>
		(dThreeDStates_, dTime_,
		 CudaSpan<CudaT>(dXTrajectory_, trajectory[0].size() * (trajectory[0][0].size() + 3)),
		 CudaSpan<CudaT>(dYTrajectory_, trajectory[1].size() * (trajectory[1][0].size() + 3)),
		 CudaSpan<CudaT>(dTTrajectory_, trajectory[2].size() * (trajectory[2][0].size() + 3)),
		 equalArcLengthTimes.back(), equalArcLengthTimes.size());

	// Extract device results from findArclengthAndTime into
	// correct format to return to caller
	// TODO experiment with where to put this relative to calling sampleTrajectories

	nvtxRangePushA("::reallocHThreeDStates");
	reallocHostPinnedMemory(hThreeDStates_, hThreeDStatesCapacity_, sizeof(ThreeDState<CudaT>) * equalArcLengthTimes.size());
	nvtxRangePop();
	nvtxRangePushA("::memcpyHThreeDStates");
	cudaSafeCall(cudaMemcpyAsync(hThreeDStates_,
			dThreeDStates_,
			sizeof(ThreeDState<CudaT>) * equalArcLengthTimes.size(),
			cudaMemcpyDeviceToHost,
			sampleXYTStream_));
	nvtxRangePop();

	// Convert the device-formatted results back to std::vectors
	// so they can be returned to the calling code
	nvtxRangePushA("::resizeXYTStates");
	xStates.resize(equalArcLengthTimes.size());
	yStates.resize(equalArcLengthTimes.size());
	tStates.resize(equalArcLengthTimes.size());
	nvtxRangePop();
	cudaSafeCall(cudaStreamSynchronize(sampleXYTStream_));
	nvtxRangePushA("::copyXYTStates");
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		const auto &s = hThreeDStates_[i];
		appendState(xStates[i], s.x_);
		appendState(yStates[i], s.y_);
		appendState(tStates[i], s.t_);
	}
	nvtxRangePop();

	nvtxRangePop();

	return true;
}

template class SampleTrajectoryImplHybrid<double>;
//template class SampleTrajectoryImplHybrid<float>;
