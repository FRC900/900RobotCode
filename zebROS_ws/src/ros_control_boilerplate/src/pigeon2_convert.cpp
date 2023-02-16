#include "ros_control_boilerplate/pigeon2_convert.h"

namespace pigeon2_convert
{
bool Pigeon2Convert::axisDirection(hardware_interface::pigeon2::AxisDirection input,
		ctre::phoenix::sensors::AxisDirection &output) const
{
	switch (input)
	{
	case hardware_interface::pigeon2::AxisDirection::PositiveZ:
		output = ctre::phoenix::sensors::AxisDirection::PositiveZ;
		break;
	case hardware_interface::pigeon2::AxisDirection::PositiveY:
		output = ctre::phoenix::sensors::AxisDirection::PositiveY;
		break;
	case hardware_interface::pigeon2::AxisDirection::PositiveX:
		output = ctre::phoenix::sensors::AxisDirection::PositiveX;
		break;
	case hardware_interface::pigeon2::AxisDirection::NegativeZ:
		output = ctre::phoenix::sensors::AxisDirection::NegativeZ;
		break;
	case hardware_interface::pigeon2::AxisDirection::NegativeY:
		output = ctre::phoenix::sensors::AxisDirection::NegativeY;
		break;
	case hardware_interface::pigeon2::AxisDirection::NegativeX:
		output = ctre::phoenix::sensors::AxisDirection::NegativeX;
		break;
	default:
		ROS_ERROR("Invalid input in Pigeon2Convert::axisDirection");
		return false;
	}
	return true;
}

} // namespace pigeon2_convert
