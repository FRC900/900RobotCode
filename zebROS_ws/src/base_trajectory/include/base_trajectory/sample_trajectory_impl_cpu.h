#ifndef INC_SAMPLE_TRAJECTORY_IMPL_CPU_
#define INC_SAMPLE_TRAJECTORY_IMPL_CPU_

#include "base_trajectory/sample_trajectory_impl.h"

template <class T>
class SampleTrajectoryImplCpu : public SampleTrajectoryImpl<T>
{
	public:
		SampleTrajectoryImplCpu(double distBetweenArcLength,
								double distBetweenArcLengthEpsilon,
								double midTimeInflation);
		virtual bool sample(std::vector<T> &equalArcLengthTimes,
					std::vector<T> &equalArcLengthPositions,
					std::vector<SegmentState<T>> &xStates,
					std::vector<SegmentState<T>> &yStates,
					std::vector<SegmentState<T>> &tStates,
					const XYTTrajectory<T> &trajectory,
					const ArcLengthTrajectory<T> &arcLengthTrajectory) override;
	protected:
		bool subdivideLength(std::vector<T> &equalArcLengthTimes,
							 std::vector<T> &equalArcLengthPositions,
							 const ArcLengthTrajectory<T> &trajectory);
		bool sampleEqualArcLengths(std::vector<SegmentState<T>> &xStates,
								   std::vector<SegmentState<T>> &yStates,
								   std::vector<SegmentState<T>> &tStates, // thetaState == rotation state
								   const std::vector<T> &equalArcLengthTimes,
								   const XYTTrajectory<T> &trajectory);
};

#endif
