#ifndef INC_SAMPLE_TRAJECTORY_IMPL_CUDA_
#define INC_SAMPLE_TRAJECTORY_IMPL_CUDA_

#include "cuda_runtime.h"
#include "base_trajectory/cudat_def.cuh"
#include "base_trajectory/sample_trajectory_impl.h"

template <class T> class SplineState;
template <class T> class TimeAndPosition;
template <class T> class ThreeDState;
template <class T>
class SampleTrajectoryImplCuda : public SampleTrajectoryImpl<T>
{
	public:
		SampleTrajectoryImplCuda(double distBetweenArcLength,
								double distBetweenArcLengthEpsilon,
								double midTimeInflation); // TODO - not used
		~SampleTrajectoryImplCuda();
		bool sample(std::vector<T> &equalArcLengthTimes,
					std::vector<T> &equalArcLengthPositions,
					std::vector<SegmentState<T>> &xStates,
					std::vector<SegmentState<T>> &yStates,
					std::vector<SegmentState<T>> &tStates,
					const XYTTrajectory<T> &trajectory,
					const ArcLengthTrajectory<T> &arcLengthTrajectory) override;
	private:
		cudaStream_t findArcLengthStream_;
		cudaStream_t sampleXYTStream_;
		cudaStream_t graphStream_;
		cudaEvent_t  findArcLengthEvent_;
		cudaEvent_t  sampleD2HMemcpyEvent_;

		cudaGraph_t     cudaGraph_;
		cudaGraphExec_t cudaGraphExec_;
		bool            recordCudaGraph_{true};

		TimeAndPosition<CudaT> *hTimeAndPositions_{nullptr};
		size_t                  hTimeAndPositionsCapacity_{0};
		TimeAndPosition<CudaT> *dTimeAndPositions_{nullptr};
		size_t                  dTimeAndPositionsCapacity_{0};

		CudaT *hArcLengthTrajectory_{nullptr};
		size_t hArcLengthTrajectoryCapacity_{0};
		CudaT *dArcLengthTrajectory_{nullptr};
		size_t dArcLengthTrajectoryCapacity_{0};

		CudaT *hXTrajectory_{nullptr};
		size_t hXTrajectoryCapacity_{0};
		CudaT *dXTrajectory_{nullptr};
		size_t dXTrajectoryCapacity_{0};

		CudaT *hYTrajectory_{nullptr};
		size_t hYTrajectoryCapacity_{0};
		CudaT *dYTrajectory_{nullptr};
		size_t dYTrajectoryCapacity_{0};

		CudaT *hTTrajectory_{nullptr};
		size_t hTTrajectoryCapacity_{0};
		CudaT *dTTrajectory_{nullptr};
		size_t dTTrajectoryCapacity_{0};

		ThreeDState<CudaT> *hThreeDStates_{nullptr};
		size_t              hThreeDStatesCapacity_{0};
		ThreeDState<CudaT> *dThreeDStates_{nullptr};
		size_t              dThreeDStatesCapacity_{0};
};
#endif
