#include "base_trajectory/arclengthtime_to_segtime.h"
#include "base_trajectory/sample_trajectory_impl_threaded.h"

//#include <iostream> // for debugging printouts only

template <class T>
SampleTrajectoryImplThreaded<T>::SampleTrajectoryImplThreaded(double distBetweenArcLength,
								double distBetweenArcLengthEpsilon,
								double midTimeInflation)
	: SampleTrajectoryImpl<T>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation)
	, threadPool_(4)
{
	threadPool_.wakeup();
}

template <class T>
bool SampleTrajectoryImplThreaded<T>::sample(std::vector<T> &equalArcLengthTimes,
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
bool SampleTrajectoryImplThreaded<T>::subdivideLength(std::vector<T> &equalArcLengthTimes,
												 std::vector<T> &equalArcLengthPositions,
												 const ArcLengthTrajectory<T> &trajectory)
{
	//threadPool_.wakeup();
	// Generate array of equally-spaced positions to search for along the arc length spline
	const T endTime = trajectory[0].back().endTime();
	SegmentState<T> state;
	trajectory_interface::sample(trajectory[0], endTime, state);
	const T endPosition = state.position;

	const size_t outSize = ceil(endPosition / this->distBetweenArcLength_);
	// Start at t=0, distance=0
	equalArcLengthTimes.resize(outSize, 0);
	equalArcLengthPositions.resize(outSize, 0);

	//ROS_INFO_STREAM("idealEqualPositions.size() = " << idealEqualPositions.size());
	const size_t countPerThread = outSize / threadPool_.getThreadCount() + 1;
	for (size_t i = 0; i < (outSize - 1); i += countPerThread)
	{
		const size_t thisCount = std::min(countPerThread, outSize - i);
		//ROS_INFO_STREAM("queuing " << i + 1 << " for length " << thisCount);
		threadPool_.enqueue(std::bind(&SampleTrajectoryImplThreaded<T>::subdivideLengthWorker, this, std::ref(equalArcLengthTimes), std::ref(equalArcLengthPositions),
				i+1, thisCount,
				endTime, endPosition,
				std::cref(trajectory)));
	}
	threadPool_.waitFinished();
	//threadPool_.sleep();

	if (equalArcLengthTimes.back() != endTime)
	{
		equalArcLengthTimes.push_back(endTime);
		equalArcLengthPositions.push_back(endPosition);
	}
#if 0
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		ROS_INFO_STREAM("i = " << i << " t = " << equalArcLengthTimes[i] << " p = " << equalArcLengthPositions[i]);
	}
#endif
	return true;
}


template <class T>
void SampleTrajectoryImplThreaded<T>::subdivideLengthWorker(std::vector<T> &equalArcLengthTimes,
															std::vector<T> &equalArcLengthPositions,
															const size_t startIdx,
															const size_t length,
															const T      endTime,
															const T      endPosition,
															const ArcLengthTrajectory<T> &trajectory) const
{
	//ROS_INFO_STREAM("startIdx = " << startIdx << " length = " << length << " endTime = " << endTime << "endPosition = " << endPosition);

	// For each cumulative distance
	// start = prev found time, end = last time
	// Binary search to get sample[] within tolerance of desired cumulative distance
	// Push that result onto equalArcLengthTimes
	T start = 0.0;

	// since we're looking for more or less monotonincally increasing values
	// keep track of the jump to get from one location to another. Use
	// this as a starting guess for the next length increment
	T prevStart = 0.0;
	T prevTimeDelta = endTime / static_cast<T>(2.0);

	SegmentState<T> state;
	trajectory_interface::sample(trajectory[0], endTime, state);

	size_t iterCount = 0;
	for (size_t i = startIdx; i < (startIdx + length); i++)
	{
		T end = endTime;
		T mid = start + prevTimeDelta;
		const auto currDistance = std::min(static_cast<T>(i * this->distBetweenArcLength_), endPosition);
		//ROS_INFO_STREAM("i = " << i << " currDistance = " << currDistance
			//<< " start=" << start << " mid=" << mid << " end=" << end << " prevStart = " << prevStart << " prevTimeDelta = " << prevTimeDelta);
		while((end - mid) > 0.00001) // Quit if time delta gets too small
		{
			const auto trajIt = trajectory_interface::sample(trajectory[0], mid, state);
			if (trajIt == trajectory[0].cend())
			{
				ROS_ERROR_STREAM("base_trajectory : could not sample mid state at time " << mid);

				//result->set_value(-1);
				return;
			}
			iterCount += 1;
			const auto delta = currDistance - state.position;
			//ROS_INFO_STREAM("currDistance = " << currDistance << " start=" << start << " mid=" << mid << " end=" << end << " position=" << state.position << " delta=" << delta);
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

		if (mid > endTime)
		{
			equalArcLengthTimes[i] = endTime;
			equalArcLengthPositions[i] = endPosition;
			break;
		}

		equalArcLengthTimes[i] = mid;
		equalArcLengthPositions[i] = state.position;
		start = mid;
		// Use starting "midpoint" guess of of start time plus
		// the previous time jump, plus a little bit extra to
		// make sure we don't undershoot. Undershooting would require a
		// binary search of basically the entire distance between
		// mid and end, and since mid is very close to the start,
		// it is basically a binary search of the entire range.
		if (prevStart == 0)
		{
			prevTimeDelta = (endTime - start) / static_cast<T>(2.0);
		}
		else
		{
			prevTimeDelta = (start - prevStart) * this->midTimeInflation_;
		}
		prevStart = start;
	}
	//result->set_value(iterCount);

	// TODO fix me!!! ROS_INFO_STREAM_FILTER(&messageFilter, "iterCount = " << iterCount);
}

template <class T>
bool SampleTrajectoryImplThreaded<T>::sampleEqualArcLengths(std::vector<SegmentState<T>> &xStates,
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

template class SampleTrajectoryImplThreaded<double>;
//template class SampleTrajectoryImplThreaded<float>;
