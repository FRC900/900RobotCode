#include "base_trajectory/sample_trajectory_impl.h"

template<class T>
SampleTrajectoryImpl<T>::SampleTrajectoryImpl(double distBetweenArcLength, double distBetweenArcLengthEpsilon, double midTimeInflation)
	: distBetweenArcLength_(distBetweenArcLength)
	, distBetweenArcLengthEpsilon_(distBetweenArcLengthEpsilon)
	, midTimeInflation_(midTimeInflation)
{
}
template<class T>
void SampleTrajectoryImpl<T>::setDistBetweenArcLengths(double distBetweenArcLength)
{
	distBetweenArcLength_ = distBetweenArcLength;
}
template<class T>
void SampleTrajectoryImpl<T>::setDistBetweenArcLengthEpsilon(double distBetweenArcLengthEpsilon)
{
	distBetweenArcLengthEpsilon_ = distBetweenArcLengthEpsilon;
}
template<class T>
void SampleTrajectoryImpl<T>::setMidTimeInflation(double midTimeInflation)
{
	midTimeInflation_ = midTimeInflation;
}
template<class T>
double SampleTrajectoryImpl<T>::getDistBetweenArcLengths() const
{
	return distBetweenArcLength_;
}
template<class T>
double SampleTrajectoryImpl<T>::getDistBetweenArcLengthEpsilon() const
{
	return distBetweenArcLengthEpsilon_;
}
template<class T>
double SampleTrajectoryImpl<T>::getMidTimeInflation() const
{
	return midTimeInflation_;
}

template class SampleTrajectoryImpl<double>;
//template class SampleTrajectoryImpl<float>;
