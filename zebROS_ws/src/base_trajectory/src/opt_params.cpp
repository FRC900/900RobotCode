#include "base_trajectory/opt_params.h"
// Optimization parameters - these are deltas added to
// the original guess for the spline generation used
// to improve the overall cost of following the spline
OptParams::OptParams(double minOffX, double maxOffX, double minOffY, double maxOffY)
	: minOffX_(minOffX)
	, maxOffX_(maxOffX)
	, minOffY_(minOffY)
	, maxOffY_(maxOffY)
{
}

// Use to enable optimization of endpoint velocity vector
void OptParams::setDeltaV(double minMagnitude, double maxMagnitude, double minDirection, double maxDirection)
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
void OptParams::clearLengthLimits(void)
{
	minLength_ = length_;
	maxLength_ = length_;
	minLengthScale_ = lengthScale_;
	maxLengthScale_ = lengthScale_;
	clearRotationLengthLimits();
}

void OptParams::clearRotationLengthLimits(void)
{
	minRotLength_ = rotLength_;
	maxRotLength_ = rotLength_;
	minRotLengthScale_ = rotLengthScale_;
	maxRotLengthScale_ = rotLengthScale_;
}

bool OptParams::IsAtMax(size_t index) const
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
	if (index == 6) // length_
	{
		return rotLength_ >= maxRotLength_;
	}
	if (index == 7) // lengthScale_
	{
		return rotLengthScale_ >= maxRotLengthScale_;
	}

	throw std::out_of_range ("out of range in OptParams IncrementVariable");
	return false;
}

bool OptParams::IncrementVariable(size_t index, double value)
{
	//ROS_INFO_STREAM("IncrementVariable, index=" << index << " value=" << value);
	if (index == 0) // posX_
	{
		return IncrementHelper(posX_, value, minOffX_, maxOffX_);
	}
	if (index == 1) // posY_
	{
		return IncrementHelper(posY_, value, minOffY_, maxOffY_);
	}
	if (index == 2) // length_
	{
		return IncrementHelper(length_, value, minLength_, maxLength_);
	}
	if (index == 3) // lengthScale_
	{
		return IncrementHelper(lengthScale_, value, minLengthScale_, maxLengthScale_);
	}
	if (index == 4) // deltaVMagnitude
	{
		return IncrementHelper(deltaVMagnitude_, value, minDeltaVMagnitude_, maxDeltaVMagnitude_);
	}
	if (index == 5) // deltaVDirection
	{
		return IncrementHelper(deltaVDirection_, value, minDeltaVDirection_, maxDeltaVDirection_);
	}
	if (index == 6) // rotLength_
	{
		return IncrementHelper(rotLength_, value, minRotLength_, maxRotLength_);
	}
	if (index == 7) // rotLengthScale_
	{
		return IncrementHelper(rotLengthScale_, value, minRotLengthScale_, maxRotLengthScale_);
	}
	throw std::out_of_range ("out of range in OptParams IncrementVariable");
	return false;
}

// Syntax to let callers use the values in this
// object as if they were an array. Since the optimizer
// loops over all member vars, this turns the code
// in there into a simple for() loop
size_t OptParams::size(void) const
{
	return 8;
}

bool OptParams::IncrementHelper(double &var, double value, double minOff, double maxOff)
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

std::ostream& operator<< (std::ostream& stream, const OptParams &optParams)
{
	stream << "posX_:" << optParams.posX_;
	stream << " posY_:" << optParams.posY_;
	stream << " minOffX_:" << optParams.minOffX_;
	stream << " maxOffX_:" << optParams.maxOffX_;
	stream << " minOffY_:" << optParams.minOffY_;
	stream << " maxOffY_:" << optParams.maxOffY_;
	stream << " length_:" << optParams.length_;
	stream << " maxLength_:" << optParams.maxLength_;
	stream << " minLength_:" << optParams.minLength_;
	stream << " lengthScale_:" << optParams.lengthScale_;
	stream << " maxLengthScale_:" << optParams.maxLengthScale_;
	stream << " minLengthScale_:" << optParams.minLengthScale_;
	stream << " rotLength_:" << optParams.length_;
	stream << " maxRotLength_:" << optParams.maxRotLength_;
	stream << " minRotLength_:" << optParams.minRotLength_;
	stream << " rotLengthScale_:" << optParams.lengthScale_;
	stream << " maxRotLengthScale_:" << optParams.maxRotLengthScale_;
	stream << " minRotLengthScale_:" << optParams.minRotLengthScale_;
	stream << " deltaVMagnitude_:" << optParams.deltaVMagnitude_;
	stream << " maxDeltaVMagnitude_:" << optParams.maxDeltaVMagnitude_;
	stream << " minDeltaVMagnitude_:" << optParams.minDeltaVMagnitude_;
	stream << " deltaVDirection_:" << optParams.deltaVDirection_;
	stream << " maxDeltaVDirection_:" << optParams.maxDeltaVDirection_;
	stream << " minDeltaVDirection_:" << optParams.minDeltaVDirection_;
	return stream;
}

