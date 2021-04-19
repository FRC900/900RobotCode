#ifndef INC_SAMPLE_TRAJECTORY_IMPL_THREADED_
#define INC_SAMPLE_TRAJECTORY_IMPL_THREADED_

#include "base_trajectory/sample_trajectory_impl.h"
#include "base_trajectory/thread_pool.h"

template <class T>
class SampleTrajectoryImplThreaded : public SampleTrajectoryImpl<T>
{
	public:
		SampleTrajectoryImplThreaded(double distBetweenArcLength,
								double distBetweenArcLengthEpsilon,
								double midTimeInflation);
		bool sample(std::vector<T> &equalArcLengthTimes,
					std::vector<T> &equalArcLengthPositions,
					std::vector<SegmentState<T>> &xStates,
					std::vector<SegmentState<T>> &yStates,
					std::vector<SegmentState<T>> &tStates,
					const XYTTrajectory<T> &trajectory,
					const ArcLengthTrajectory<T> &arcLengthTrajectory) override;
	private:
		bool subdivideLength(std::vector<T> &equalArcLengthTimes,
							 std::vector<T> &equalArcLengthPositions,
							 const ArcLengthTrajectory<T> &trajectory);
		bool sampleEqualArcLengths(std::vector<SegmentState<T>> &xStates,
								   std::vector<SegmentState<T>> &yStates,
								   std::vector<SegmentState<T>> &tStates, // thetaState == rotation state
								   const std::vector<T> &equalArcLengthTimes,
								   const XYTTrajectory<T> &trajectory);
		void subdivideLengthWorker(std::vector<T> &equalArcLengthTimes,
								   std::vector<T> &equalArcLengthPositions,
									const size_t startIdx,
									const size_t length,
									const T      endTime,
									const T      endPosition,
									const ArcLengthTrajectory<T> &trajectory) const;

		ThreadPool threadPool_;
};

#endif
