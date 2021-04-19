#include "base_trajectory/sample_trajectory.h"
#include "base_trajectory/sample_trajectory_impl_cpu.h"
#include "base_trajectory/sample_trajectory_impl_cuda.h"
#include "base_trajectory/sample_trajectory_impl_hybrid.h"
#include "base_trajectory/sample_trajectory_impl_threaded.h"

template <class T>
SampleTrajectory<T>::SampleTrajectory(double distBetweenArcLength, double distBetweenArcLengthEpsilon, double midTimeInflation, SampleTrajectoryType type)
{
	if (type == SampleTrajectoryType::CPU)
		impl_ = std::make_unique<SampleTrajectoryImplCpu<T>>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation);
	else if (type == SampleTrajectoryType::THREADED)
		impl_ = std::make_unique<SampleTrajectoryImplThreaded<T>>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation);
	else if (type == SampleTrajectoryType::CUDA)
		impl_ = std::make_unique<SampleTrajectoryImplCuda<T>>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation);
	else if (type == SampleTrajectoryType::HYBRID)
		impl_ = std::make_unique<SampleTrajectoryImplHybrid<T>>(distBetweenArcLength, distBetweenArcLengthEpsilon, midTimeInflation);
	else
		ROS_INFO_STREAM("Invalid SampleTrajectoryType : " << static_cast<int>(type));
}

template <class T>
SampleTrajectory<T>::~SampleTrajectory<T>() = default;

template <class T>
bool SampleTrajectory<T>::sample(std::vector<T> &equalArcLengthTimes,
		std::vector<T> &equalArcLengthPositions,
		std::vector<SegmentState<T>> &xStates,
		std::vector<SegmentState<T>> &yStates,
		std::vector<SegmentState<T>> &thetaStates,
		const XYTTrajectory<T> &trajectory,
		const ArcLengthTrajectory<T> &arcLengthTrajectory)
{
	return impl_->sample(equalArcLengthTimes, equalArcLengthPositions,
						 xStates, yStates, thetaStates,
						 trajectory, arcLengthTrajectory);
}

template <class T>
void SampleTrajectory<T>::setDistBetweenArcLengths(double distBetweenArcLength)
{
	impl_->setDistBetweenArcLengths(distBetweenArcLength);
}

template <class T>
void SampleTrajectory<T>::setDistBetweenArcLengthEpsilon(double distBetweenArcLengthEpsilon)
{
	impl_->setDistBetweenArcLengthEpsilon(distBetweenArcLengthEpsilon);
}

template <class T>
void SampleTrajectory<T>::setMidTimeInflation(double midTimeInflation)
{
	impl_->setMidTimeInflation(midTimeInflation);
}

template <class T>
double SampleTrajectory<T>::getDistBetweenArcLengths() const
{
	return impl_->getDistBetweenArcLengths();
}

template <class T>
double SampleTrajectory<T>::getDistBetweenArcLengthEpsilon() const
{
	return impl_->getDistBetweenArcLengthEpsilon();
}

template <class T>
double SampleTrajectory<T>::getMidTimeInflation() const
{
	return impl_->getMidTimeInflation();
}

template class SampleTrajectory<double>;
//template class SampleTrajectory<float>;
