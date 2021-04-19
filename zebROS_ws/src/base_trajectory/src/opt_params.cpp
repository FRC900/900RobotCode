#include <limits>
#include "base_trajectory/opt_params.h"

// Optimization parameters - these are deltas added to
// the original guess for the spline generation used
// to improve the overall cost of following the spline
template <class T>
OptParams<T>::OptParams(T minOffX, T maxOffX, T minOffY, T maxOffY)
	: minOffX_(minOffX)
	, maxOffX_(maxOffX)
	, minOffY_(minOffY)
	, maxOffY_(maxOffY)
{
}

// Use to enable optimization of endpoint velocity vector
template <class T>
void OptParams<T>::setDeltaV(T minMagnitude, T maxMagnitude, T minDirection, T maxDirection)
{
	minDeltaVMagnitude_ = minMagnitude;
	maxDeltaVMagnitude_ = maxMagnitude;
	minDeltaVDirection_ = minDirection;
	maxDeltaVDirection_ = maxDirection;
}

// Force length var ranges to be +/- 0 from
// the current value. Used to disable optimization
// of these vars for start and end points since they
// don't apply
template <class T>
void OptParams<T>::clearLengthLimits(void)
{
	minLength_ = length_;
	maxLength_ = length_;
	minLengthScale_ = lengthScale_;
	maxLengthScale_ = lengthScale_;
	clearRotationLengthLimits();
}

template <class T>
void OptParams<T>::clearRotationLengthLimits(void)
{
	minRotLength_ = rotLength_;
	maxRotLength_ = rotLength_;
	minRotLengthScale_ = rotLengthScale_;
	maxRotLengthScale_ = rotLengthScale_;
}

template <class T>
T &OptParams<T>::operator[] (size_t index)
{
	if (index == 0) // posX_
	{
		return posX_;
	}
	if (index == 1) // posX_
	{
		return posY_;
	}
	if (index == 2) // length_
	{
		return length_;
	}
	if (index == 3) // lengthScale_
	{
		return lengthScale_;
	}
	if (index == 4) // deltaVMagnitude_
	{
		return deltaVMagnitude_;
	}
	if (index == 5) // deltaVDirection_
	{
		return deltaVDirection_;
	}
	if (index == 6) // rotLength_
	{
		return rotLength_;
	}
	if (index == 7) // rotLengthScale_
	{
		return rotLengthScale_;
	}

	throw std::out_of_range ("out of range in OptParams operator[]");
	return posX_;
}

template <class T>
const T& OptParams<T>::operator[] (size_t index) const
{
	if (index == 0) // posX_
	{
		return posX_;
	}
	if (index == 1) // posX_
	{
		return posY_;
	}
	if (index == 2) // length_
	{
		return length_;
	}
	if (index == 3) // lengthScale_
	{
		return lengthScale_;
	}
	if (index == 4) // deltaVMagnitude_
	{
		return deltaVMagnitude_;
	}
	if (index == 5) // deltaVDirection_
	{
		return deltaVDirection_;
	}
	if (index == 6) // rotLength_
	{
		return rotLength_;
	}
	if (index == 7) // rotLengthScale_
	{
		return rotLengthScale_;
	}
	throw std::out_of_range ("out of range in OptParams operator[]");
	return posX_;
}

template <class T>
bool OptParams<T>::isAtMax(size_t index) const
{
	if (index == 0) // posX_
	{
		return posX_ >= maxOffX_;
	}
	if (index == 1) // posX_
	{
		return posY_ >= maxOffY_;
	}
	if (index == 2) // length_
	{
		return length_ >= maxLength_;
	}
	if (index == 3) // lengthScale_
	{
		return lengthScale_ >= maxLengthScale_;
	}
	if (index == 4) // deltaVMagnitude_
	{
		return deltaVMagnitude_ >= maxDeltaVMagnitude_;
	}
	if (index == 5) // deltaVDirection_
	{
		return deltaVDirection_ >= maxDeltaVDirection_;
	}
	if (index == 6) // rotLength_
	{
		return rotLength_ >= maxRotLength_;
	}
	if (index == 7) // rotLengthScale_
	{
		return rotLengthScale_ >= maxRotLengthScale_;
	}

	throw std::out_of_range ("out of range in OptParams isAtMax");
	return false;
}

template <class T>
bool OptParams<T>::doNotOptimize(size_t index) const
{
	if (index == 0) // posX_
	{
		return minOffX_ >= maxOffX_;
	}
	if (index == 1) // posX_
	{
		return minOffY_ >= maxOffY_;
	}
	if (index == 2) // length_
	{
		return minLength_ >= maxLength_;
	}
	if (index == 3) // lengthScale_
	{
		return true; // just use 1 length var for now
		return minLengthScale_ >= maxLengthScale_;
	}
	if (index == 4) // deltaVMagnitude_
	{
		return minDeltaVMagnitude_ >= maxDeltaVMagnitude_;
	}
	if (index == 5) // deltaVDirection_
	{
		return minDeltaVDirection_ >= maxDeltaVDirection_;
	}
	if (index == 6) // rotLength_
	{
		return minRotLength_ >= maxRotLength_;
	}
	if (index == 7) // rotLengthScale_
	{
		return true; // just use 1 length var for now
		return minRotLengthScale_ >= maxRotLengthScale_;
	}

	throw std::out_of_range ("out of range in OptParams doNotOptimize");
	return false;
}

template <class T>
T OptParams<T>::getLowerLimit(size_t index) const
{
	if (index == 0) // posX_
	{
		return minOffX_;
	}
	if (index == 1) // posX_
	{
		return minOffY_;
	}
	if (index == 2) // length_
	{
		return minLength_;
	}
	if (index == 3) // lengthScale_
	{
		return minLengthScale_;
	}
	if (index == 4) // deltaVMagnitude_
	{
		return minDeltaVMagnitude_;
	}
	if (index == 5) // deltaVDirection_
	{
		return minDeltaVDirection_;
	}
	if (index == 6) // rotLength_
	{
		return minRotLength_;
	}
	if (index == 7) // rotLengthScale_
	{
		return minRotLengthScale_;
	}

	throw std::out_of_range ("out of range in OptParams getLowerLimit");
	return std::numeric_limits<T>::quiet_NaN();
}

template <class T>
T OptParams<T>::getUpperLimit(size_t index) const
{
	if (index == 0) // posX_
	{
		return maxOffX_;
	}
	if (index == 1) // posX_
	{
		return maxOffY_;
	}
	if (index == 2) // length_
	{
		return maxLength_;
	}
	if (index == 3) // lengthScale_
	{
		return maxLengthScale_;
	}
	if (index == 4) // deltaVMagnitude_
	{
		return maxDeltaVMagnitude_;
	}
	if (index == 5) // deltaVDirection_
	{
		return maxDeltaVDirection_;
	}
	if (index == 6) // rotLength_
	{
		return maxRotLength_;
	}
	if (index == 7) // rotLengthScale_
	{
		return maxRotLengthScale_;
	}

	throw std::out_of_range ("out of range in OptParams getUpperLimit");
	return std::numeric_limits<T>::quiet_NaN();
}


template <class T>
bool OptParams<T>::incrementVariable(size_t index, T value)
{
	//ROS_INFO_STREAM("IncrementVariable, index=" << index << " value=" << value);
	if (index == 0) // posX_
	{
		return incrementHelper(posX_, value, minOffX_, maxOffX_);
	}
	if (index == 1) // posY_
	{
		return incrementHelper(posY_, value, minOffY_, maxOffY_);
	}
	if (index == 2) // length_
	{
		return incrementHelper(length_, value, minLength_, maxLength_);
	}
	if (index == 3) // lengthScale_
	{
		return incrementHelper(lengthScale_, value, minLengthScale_, maxLengthScale_);
	}
	if (index == 4) // deltaVMagnitude
	{
		return incrementHelper(deltaVMagnitude_, value, minDeltaVMagnitude_, maxDeltaVMagnitude_);
	}
	if (index == 5) // deltaVDirection
	{
		return incrementHelper(deltaVDirection_, value, minDeltaVDirection_, maxDeltaVDirection_);
	}
	if (index == 6) // rotLength_
	{
		return incrementHelper(rotLength_, value, minRotLength_, maxRotLength_);
	}
	if (index == 7) // rotLengthScale_
	{
		return incrementHelper(rotLengthScale_, value, minRotLengthScale_, maxRotLengthScale_);
	}
	throw std::out_of_range ("out of range in OptParams IncrementVariable");
	return false;
}

template <class T>
T OptParams<T>::initialDParamMultiplier(size_t index) const
{
	if ((index == 0) || (index == 1))
		return 5;
	return 1;
}

template <class T>
T OptParams<T>::initialDParamGuess(size_t index, T value) const
{
	//ROS_INFO_STREAM("IncrementVariable, index=" << index << " value=" << value);
	if (index == 0) // posX_
	{
		return initialDParamGuessHelper(index, posX_, value, minOffX_, maxOffX_);
	}
	if (index == 1) // posY_
	{
		return initialDParamGuessHelper(index, posY_, value, minOffY_, maxOffY_);
	}
	if (index == 2) // length_
	{
		return initialDParamGuessHelper(index, length_, value, minLength_, maxLength_);
	}
	if (index == 3) // lengthScale_
	{
		return initialDParamGuessHelper(index, lengthScale_, value, minLengthScale_, maxLengthScale_);
	}
	if (index == 4) // deltaVMagnitude
	{
		return initialDParamGuessHelper(index, deltaVMagnitude_, value, minDeltaVMagnitude_, maxDeltaVMagnitude_);
	}
	if (index == 5) // deltaVDirection
	{
		return initialDParamGuessHelper(index, deltaVDirection_, value, minDeltaVDirection_, maxDeltaVDirection_);
	}
	if (index == 6) // rotLength_
	{
		return initialDParamGuessHelper(index, rotLength_, value, minRotLength_, maxRotLength_);
	}
	if (index == 7) // rotLengthScale_
	{
		return initialDParamGuessHelper(index, rotLengthScale_, value, minRotLengthScale_, maxRotLengthScale_);
	}
	throw std::out_of_range ("out of range in OptParams IncrementVariable");
	return value;
}

template <class T>
T OptParams<T>::initialDParamGuessHelper(size_t index, T var, T value, T minOff, T maxOff) const
{
	if (var > 0)
	{
		return std::min(initialDParamMultiplier(index) * value, maxOff);
	}
	return std::max(initialDParamMultiplier(index) * value, minOff);
}

// Syntax to let callers use the values in this
// object as if they were an array. Since the optimizer
// loops over all member vars, this turns the code
// in there into a simple for() loop
template <class T>
size_t OptParams<T>::size(void) const
{
	return 8;
}

template <class T>
T OptParams<T>::regularizationCost(T lambda) const
{
	T cost{0};

	cost += (posX_ - initialPosX_) * (posX_ - initialPosX_);
	cost += (posY_ - initialPosY_) * (posY_ - initialPosY_);
	cost += (length_ - initialLength_) * (length_ - initialLength_);
	cost += (lengthScale_ - initialLengthScale_) * (lengthScale_ - initialLengthScale_);
	cost += (rotLength_ - initialRotLength_) * (rotLength_ - initialRotLength_);
	cost += (rotLengthScale_ - initialRotLengthScale_) * (rotLengthScale_ - initialRotLengthScale_);
	cost += (deltaVMagnitude_ - initialDeltaVMagnitude_) * (deltaVMagnitude_ - initialDeltaVMagnitude_);
	cost += (deltaVDirection_ - initialDeltaVDirection_) * (deltaVDirection_ - initialDeltaVDirection_);
	cost *= static_cast<T>(size()) * lambda;

	return cost;
}

template <class T>
bool OptParams<T>::incrementHelper(T &var, T value, T minOff, T maxOff)
{
	//ROS_INFO_STREAM("  var=" << var << " value=" << value << " minOff=" << minOff << " maxOff=" << maxOff);
	if ((value > 0) && (var >= maxOff))
	{
		//ROS_INFO_STREAM("At maximum");
		return false;
	}
	if ((value < 0) && (var <= minOff))
	{
		//ROS_INFO_STREAM("At minimum");
		return false;
	}

	if ((var + value) >= maxOff)
	{
		//ROS_INFO_STREAM("hit max");
		var = maxOff;
	}
	else if ((var + value) <= minOff)
	{
		//ROS_INFO_STREAM("hit min");
		var = minOff;
	}
	else
		var += value;
	//ROS_INFO_STREAM("new var = " << var);
	return true;
}

template class OptParams<double>;
template class OptParams<float>;

template <class T>
std::ostream& operator<< (std::ostream& stream, const OptParams<T> &optParams)
{
	stream << "posX_:" << optParams.posX_;
	stream << " (" << optParams.minOffX_;
	stream << ", " << optParams.maxOffX_;
	stream << ") ";
	stream << " posY_:" << optParams.posY_;
	stream << " (" << optParams.minOffY_;
	stream << ", " << optParams.maxOffY_;
	stream << ") ";
	stream << " length_:" << optParams.length_;
	stream << " (" << optParams.maxLength_;
	stream << ", " << optParams.minLength_;
	stream << ") ";
	stream << " lengthScale_:" << optParams.lengthScale_;
	stream << " (" << optParams.maxLengthScale_;
	stream << ", " << optParams.minLengthScale_;
	stream << ") ";
	stream << " rotLength_:" << optParams.rotLength_;
	stream << " (" << optParams.maxRotLength_;
	stream << ", " << optParams.minRotLength_;
	stream << ") ";
	stream << " rotLengthScale_:" << optParams.rotLengthScale_;
	stream << " (" << optParams.maxRotLengthScale_;
	stream << ", " << optParams.minRotLengthScale_;
	stream << ") ";
	stream << " deltaVMagnitude_:" << optParams.deltaVMagnitude_;
	stream << " (" << optParams.maxDeltaVMagnitude_;
	stream << ", " << optParams.minDeltaVMagnitude_;
	stream << ") ";
	stream << " deltaVDirection_:" << optParams.deltaVDirection_;
	stream << " (" << optParams.maxDeltaVDirection_;
	stream << ", " << optParams.minDeltaVDirection_;
	stream << ") ";
	return stream;
}

template std::ostream& operator<< (std::ostream& stream, const OptParams<double> &optParams);
//template std::ostream& operator<< (std::ostream& stream, const OptParams<float> &optParams);
