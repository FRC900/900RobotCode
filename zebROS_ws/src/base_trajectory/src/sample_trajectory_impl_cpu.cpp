#include "base_trajectory/arclengthtime_to_segtime.h"
#include "base_trajectory/sample_trajectory_impl_cpu.h"

template <class T>
SampleTrajectoryImplCpu<T>::SampleTrajectoryImplCpu(double distBetweenArcLength,
													double distBetweenArcLengthEpsilon,
													double midTimeInflation)
	: SampleTrajectoryImpl<T>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation)
{
}

template <class T>
bool SampleTrajectoryImplCpu<T>::sample(std::vector<T> &equalArcLengthTimes,
										std::vector<T> &equalArcLengthPositions,
										std::vector<SegmentState<T>> &xStates,
										std::vector<SegmentState<T>> &yStates,
										std::vector<SegmentState<T>> &tStates,
										const XYTTrajectory<T> &trajectory,
										const ArcLengthTrajectory<T> &arcLengthTrajectory)
{
	return subdivideLength(equalArcLengthTimes, equalArcLengthPositions, arcLengthTrajectory) &&
		sampleEqualArcLengths(xStates, yStates, tStates, equalArcLengthTimes, trajectory);
}

template <class T>
bool SampleTrajectoryImplCpu<T>::subdivideLength(std::vector<T> &equalArcLengthTimes,
												 std::vector<T> &equalArcLengthPositions,
												 const ArcLengthTrajectory<T> &trajectory)
{
	// For each cumulative distance
	// start = prev found time, end = last time
	// Binary search to get sample[] within tolerance of desired cumulative distance
	// Push that result onto equalArcLengthTimes
	T start{static_cast<T>(0.0)};

	// since we're looking for more or less monotonincally increasing values
	// keep track of the jump to get from one location to another. Use
	// this as a starting guess for the next length increment
	T prevStart = start;
	const auto endTime = trajectory[0].back().endTime();
	T prevTimeDelta = endTime / static_cast<T>(2.0);

	T totalLength;
	if (trajectory_interface::samplePosition(trajectory[0], endTime, totalLength) == trajectory[0].cend())
	{
		ROS_ERROR_STREAM(__FUNCTION__ << " : error sampling endTime (" << endTime << ") of arc length trajectory");
		return false;
	}

	const size_t outSize = ceil(totalLength / this->distBetweenArcLength_) + 1;
	equalArcLengthTimes.resize(outSize);
	equalArcLengthPositions.resize(outSize);
	equalArcLengthTimes[0] = 0; // start at t==0
	equalArcLengthPositions[0] = 0; // start at distace==0

	size_t iterCount = 0;
	size_t outIdx = 1;
	for (T currDistance = this->distBetweenArcLength_; currDistance <= totalLength; currDistance += this->distBetweenArcLength_)
	{
		auto end = endTime;
		auto mid = start + prevTimeDelta;
		if (mid >= end)
			mid = (start + end) / static_cast<T>(2.0);
		T currPosition{0};
		while((end - mid) > static_cast<T>(0.00001)) // Quit if time delta gets too small
		{
			const auto trajIt = trajectory_interface::samplePosition(trajectory[0], mid, currPosition);
			if (trajIt == trajectory[0].cend())
			{
				ROS_ERROR_STREAM("sample_trajectory_cpu: could not sample mid state at time " << mid);
				return false;
			}
			iterCount += 1;
			const T delta = currDistance - currPosition;
			//ROS_INFO_STREAM("currDistance = " << currDistance << " start=" << start << " mid=" << mid << " end=" << end << " position[0]=" << currPosition << " delta=" << delta);
			if (fabs(delta) < this->distBetweenArcLengthEpsilon_)
			{
				break;
			}

			if (delta > 0)
			{
				start = mid;
			}
			else
			{
				end = mid;
			}
			mid = (start + end) / 2.0;
		}

		equalArcLengthTimes[outIdx] = mid;
		equalArcLengthPositions[outIdx] = currPosition;
		outIdx += 1;
		start = mid;
		// Use starting "midpoint" guess of of start time plus
		// the previous time jump, plus a little bit extra to
		// make sure we don't undershoot. Undershooting would require a
		// binary search of basically the entire distance between
		// mid and end, and since mid is very close to the start,
		// it is basically a binary search of the entire range.
		prevTimeDelta = (start - prevStart) * this->midTimeInflation_;
		prevStart = start;
	}

	if (equalArcLengthTimes[outIdx - 1] != endTime)
	{
		equalArcLengthTimes[outIdx] = endTime;
		equalArcLengthPositions[outIdx] = totalLength;
	}
	// TODO fix me!!! ROS_INFO_STREAM_FILTER(&messageFilter, "iterCount = " << iterCount);
	return true;
}

template <class T>
bool SampleTrajectoryImplCpu<T>::sampleEqualArcLengths(std::vector<SegmentState<T>> &xStates,
													   std::vector<SegmentState<T>> &yStates,
													   std::vector<SegmentState<T>> &tStates, // thetaState == rotation state
													   const std::vector<T> &equalArcLengthTimes,
													   const XYTTrajectory<T> &trajectory)
{
	xStates.resize(equalArcLengthTimes.size());
	yStates.resize(equalArcLengthTimes.size());
	tStates.resize(equalArcLengthTimes.size());
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		const auto t   = equalArcLengthTimes[i];
		const auto seg = arcLengthTimeToSegTime(equalArcLengthTimes, i);

		// Since seg is set to the current spline segment, use this to
		// index into each trajectory. This saves time compared to searching
		// through the range of times in each trajectory array
		auto xIt = trajectory[0].cbegin() + seg;
		if (xIt >= trajectory[0].cend())
		{
			ROS_ERROR_STREAM("base_trajectory : evaluateTrajectory could not sample xState at time " << t << ", seg = " << seg << ", trajectory[0].size() = " << trajectory[0].size());
			return false;
		}
		xIt->sample(t, xStates[i]);

		auto yIt = trajectory[1].cbegin() + seg;
		if (yIt >= trajectory[1].cend())
		{
			ROS_ERROR_STREAM("base_trajectory : evaluateTrajectory could not sample yState at time " << t << ", seg = " << seg << ", trajectory[1].size() = " << trajectory[1].size());
			return false;
		}
		yIt->sample(t, yStates[i]);

		auto tIt = trajectory[2].cbegin() + seg;
		if (tIt >= trajectory[2].cend())
		{
			ROS_ERROR_STREAM("base_trajectory : evaluateTrajectory could not sample tState at time " << t << ", seg = " << seg << ", trajectory[2].size() = " << trajectory[2].size());
			return false;
		}
		tIt->sample(t, tStates[i]);
	}
	return true;
}

template class SampleTrajectoryImplCpu<double>;
//template class SampleTrajectoryImplCpu<float>;
