#ifndef INC_SAMPLE_TRAJECTORY_IMPL_
#define INC_SAMPLE_TRAJECTORY_IMPL_

#include "spline_util/spline_util.h"

template <class T>
class SampleTrajectoryImpl
{
	public:
		SampleTrajectoryImpl(double distBetweenArcLength, double distBetweenArcLengthEpsilon, double midTimeInflation);
		virtual ~SampleTrajectoryImpl() = default;
		virtual bool sample(std::vector<T> &equalArcLengthTimes,
				std::vector<T> &equalArcLengthPositions,
				std::vector<SegmentState<T>> &xStates,
				std::vector<SegmentState<T>> &yStates,
				std::vector<SegmentState<T>> &thetaStates,
				const XYTTrajectory<T> &trajectory,
				const ArcLengthTrajectory<T> &arcLengthTrajectory) = 0;

		virtual void setDistBetweenArcLengths(double distBetweenArcLength);
		virtual void setDistBetweenArcLengthEpsilon(double distBetweenArcLengthEpsilon);
		virtual void setMidTimeInflation(double midTimeInflation);
		virtual double getDistBetweenArcLengths() const;
		virtual double getDistBetweenArcLengthEpsilon() const;
		virtual double getMidTimeInflation() const;

	protected:
		double distBetweenArcLength_;
		double distBetweenArcLengthEpsilon_;
		double midTimeInflation_;
};

#endif


