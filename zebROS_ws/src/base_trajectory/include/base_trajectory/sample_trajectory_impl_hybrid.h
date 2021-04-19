#ifndef INC_SAMPLE_TRAJECTORY_IMPL_HYBRID_
#define INC_SAMPLE_TRAJECTORY_IMPL_HYBRID_

#include "cuda_runtime.h"
#include "base_trajectory/cudat_def.cuh"
#include "base_trajectory/sample_trajectory_impl_cpu.h"

template <class T> class ThreeDState;
template <class T>
class SampleTrajectoryImplHybrid : public SampleTrajectoryImplCpu<T>
{
	public:
		SampleTrajectoryImplHybrid(double distBetweenArcLength,
								double distBetweenArcLengthEpsilon,
								double midTimeInflation);
		~SampleTrajectoryImplHybrid();
		virtual bool sample(std::vector<T> &equalArcLengthTimes,
					std::vector<T> &equalArcLengthPositions,
					std::vector<SegmentState<T>> &xStates,
					std::vector<SegmentState<T>> &yStates,
					std::vector<SegmentState<T>> &tStates,
					const XYTTrajectory<T> &trajectory,
					const ArcLengthTrajectory<T> &arcLengthTrajectory) override;
	private:
		cudaStream_t sampleXYTStream_;

		CudaT *hTime_{nullptr};
		size_t hTimeCapacity_{0};
		CudaT *dTime_{nullptr};
		size_t dTimeCapacity_{0};

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
