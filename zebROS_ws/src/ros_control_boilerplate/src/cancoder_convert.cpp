#include "ros_control_boilerplate/cancoder_convert.h"

namespace cancoder_convert
{
bool CANCoderConvert::magnetFieldStrength(ctre::phoenix::sensors::MagnetFieldStrength input,
		hardware_interface::cancoder::MagnetFieldStrength &output) const
{
	switch (input)
	{
		case ctre::phoenix::sensors::MagnetFieldStrength::Invalid_Unknown:
			output = hardware_interface::cancoder::MagnetFieldStrength::Invalid_Unknown;
			break;
		case ctre::phoenix::sensors::MagnetFieldStrength::BadRange_RedLED:
			output = hardware_interface::cancoder::MagnetFieldStrength::BadRange_RedLED;
			break;
		case ctre::phoenix::sensors::MagnetFieldStrength::Adequate_OrangeLED:
			output = hardware_interface::cancoder::MagnetFieldStrength::Adequate_OrangeLED;
			break;
		case ctre::phoenix::sensors::MagnetFieldStrength::Good_GreenLED:
			output = hardware_interface::cancoder::MagnetFieldStrength::Good_GreenLED;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANCoderMagnetFieldStrength");
			return false;
	}
	return true;
}

bool CANCoderConvert::velocityMeasPeriod(hardware_interface::cancoder::SensorVelocityMeasPeriod input,
		ctre::phoenix::sensors::SensorVelocityMeasPeriod &output) const
{
	switch (input)
	{
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_1Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_1Ms;
			break;
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_2Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_2Ms;
			break;
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_5Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_5Ms;
			break;
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_10Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_10Ms;
			break;
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_20Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_20Ms;
			break;
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_25Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_25Ms;
			break;
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_50Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms;
			break;
		case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_100Ms:
			output = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_100Ms;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANCoderVelocityMeasPeriod");
			return false;
	}
	return true;
}

bool CANCoderConvert::absoluteSensorRange(hardware_interface::cancoder::AbsoluteSensorRange input,
		ctre::phoenix::sensors::AbsoluteSensorRange &output) const
{
	switch (input)
	{
		case hardware_interface::cancoder::AbsoluteSensorRange::Unsigned_0_to_360:
			output = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
			break;
		case hardware_interface::cancoder::AbsoluteSensorRange::Signed_PlusMinus180:
			output = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANCoderAbsoluteSensorRange");
			return false;
	}
	return true;
}

bool CANCoderConvert::initializationStrategy(hardware_interface::cancoder::SensorInitializationStrategy input,
		ctre::phoenix::sensors::SensorInitializationStrategy &output) const
{
	switch (input)
	{
		case hardware_interface::cancoder::SensorInitializationStrategy::BootToZero:
			output = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;
			break;
		case hardware_interface::cancoder::SensorInitializationStrategy::BootToAbsolutePosition:
			output = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANCoderInitializationStrategy");
			return false;
	}
	return true;
}
bool CANCoderConvert::timeBase(hardware_interface::cancoder::SensorTimeBase input,
		ctre::phoenix::sensors::SensorTimeBase &output) const
{
	switch (input)
	{
		case hardware_interface::cancoder::SensorTimeBase::Per100Ms_Legacy:
			output = ctre::phoenix::sensors::SensorTimeBase::Per100Ms_Legacy;
			break;
		case hardware_interface::cancoder::SensorTimeBase::PerSecond:
			output = ctre::phoenix::sensors::SensorTimeBase::PerSecond;
			break;
		case hardware_interface::cancoder::SensorTimeBase::PerMinute:
			output = ctre::phoenix::sensors::SensorTimeBase::PerMinute;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANCoderTimeBase");
			return false;
	}
	return true;
}

} // namespace cancoder_convert

