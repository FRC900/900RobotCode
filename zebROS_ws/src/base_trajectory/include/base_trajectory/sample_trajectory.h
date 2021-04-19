#ifndef INC_SAMPLE_TRAJECTORY_
#define INC_SAMPLE_TRAJECTORY_

#include "spline_util/spline_util.h"

enum class SampleTrajectoryType
{
	CPU,
	THREADED,
	CUDA,
	HYBRID
};

template <class T>
class SampleTrajectoryImpl;
template <class T>
class SampleTrajectory
{
	public:
		SampleTrajectory(double distBetweenArcLength,
						 double distBetweenArcLengthEpsilon,
						 double midTimeInflation,
						 SampleTrajectoryType type = SampleTrajectoryType::CPU);
		~SampleTrajectory();

		bool sample(std::vector<T> &equalArcLengthTimes,
					std::vector<T> &equalArcLengthPositions,
					std::vector<SegmentState<T>> &xStates,
					std::vector<SegmentState<T>> &yStates,
					std::vector<SegmentState<T>> &thetaStates,
					const XYTTrajectory<T> &trajectory,
					const ArcLengthTrajectory<T> &arcLengthTrajectory);


		void setDistBetweenArcLengths(double distBetweenArcLength);
		void setDistBetweenArcLengthEpsilon(double distBetweenArcLengthEpsilon);
		void setMidTimeInflation(double midTimeInflation);
		double getDistBetweenArcLengths() const;
		double getDistBetweenArcLengthEpsilon() const;
		double getMidTimeInflation() const;
	private:
		std::unique_ptr<SampleTrajectoryImpl<T>> impl_;
};

#endif
