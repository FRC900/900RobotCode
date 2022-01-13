#include "ros_control_boilerplate/rev_convert.h"

namespace rev_convert
{
bool RevConvert::motorType(const hardware_interface::MotorType input,
		rev::CANSparkMaxLowLevel::MotorType &output) const
{
	switch (input)
	{
		case hardware_interface::kBrushed:
			output = rev::CANSparkMaxLowLevel::MotorType::kBrushed;
			break;
		case hardware_interface::kBrushless:
			output = rev::CANSparkMaxLowLevel::MotorType::kBrushless;
			break;
		default:
			ROS_ERROR("Invalid input in convertRevMotorType");
			return false;
	}
	return true;
}
bool RevConvert::limitSwitchPolarity(const hardware_interface::LimitSwitchPolarity input,
		rev::CANDigitalInput::LimitSwitchPolarity &output) const
{
	switch (input)
	{
		case hardware_interface::kNormallyOpen:
			output = rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen;
			break;
		case hardware_interface::kNormallyClosed:
			output = rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed;
			break;
		default:
			ROS_ERROR("Invalid input in convertRevLimitSwitchPolarity");
			return false;
	}
	return true;
}

bool RevConvert::encoderType(const hardware_interface::SensorType input,
		rev::CANEncoder::EncoderType &output) const
{
	switch (input)
	{
		case hardware_interface::kNoSensor:
			output = rev::CANEncoder::EncoderType::kNoSensor;
			break;
		case hardware_interface::kHallSensor:
			output = rev::CANEncoder::EncoderType::kHallSensor;
			break;
		case hardware_interface::kQuadrature:
			output = rev::CANEncoder::EncoderType::kQuadrature;
			break;
		default:
			ROS_ERROR("Invalid input in convertRevEncoderType");
			return false;
	}
	return true;
}

bool RevConvert::controlType(const hardware_interface::ControlType input,
		rev::ControlType &output) const
{
	switch (input)
	{
		case hardware_interface::kDutyCycle:
			output = rev::ControlType::kDutyCycle;
			break;
		case hardware_interface::kVelocity:
			output = rev::ControlType::kVelocity;
			break;
		case hardware_interface::kVoltage:
			output = rev::ControlType::kVoltage;
			break;
		case hardware_interface::kPosition:
			output = rev::ControlType::kPosition;
			break;
		case hardware_interface::kSmartMotion:
			output = rev::ControlType::kSmartMotion;
			break;
		case hardware_interface::kCurrent:
			output = rev::ControlType::kCurrent;
			break;
		case hardware_interface::kSmartVelocity:
			output = rev::ControlType::kSmartVelocity;
			break;

		default:
			ROS_ERROR("Invalid input in convertRevControlType");
			return false;
	}
	return true;
}

bool RevConvert::arbFFUnits(const hardware_interface::ArbFFUnits input,
		rev::CANPIDController::ArbFFUnits &output) const
{
	switch (input)
	{
		case hardware_interface::ArbFFUnits::kVoltage:
			output = rev::CANPIDController::ArbFFUnits::kVoltage;
			break;
		case hardware_interface::ArbFFUnits::kPercentOut:
			output = rev::CANPIDController::ArbFFUnits::kPercentOut;
			break;

		default:
			ROS_ERROR("Invalid input in convertRevControlType");
			return false;
	}
	return true;
}

bool RevConvert::idleMode(const hardware_interface::IdleMode input,
		rev::CANSparkMax::IdleMode &output) const
{
	switch (input)
	{
		case hardware_interface::IdleMode::kCoast:
			output = rev::CANSparkMax::IdleMode::kCoast;
			break;
		case hardware_interface::IdleMode::kBrake:
			output = rev::CANSparkMax::IdleMode::kBrake;
			break;

		default:
			ROS_ERROR("Invalid input in convertRevIdleMode");
			return false;
	}
	return true;
}

bool RevConvert::externalFollower(const hardware_interface::ExternalFollower input,
		rev::CANSparkMax::ExternalFollower &output) const
{
	switch (input)
	{
		case hardware_interface::ExternalFollower::kFollowerDisabled:
			output = rev::CANSparkMax::kFollowerDisabled;
			break;
		case hardware_interface::ExternalFollower::kFollowerSparkMax:
			output = rev::CANSparkMax::kFollowerSparkMax;
			break;
		case hardware_interface::ExternalFollower::kFollowerPhoenix:
			output = rev::CANSparkMax::kFollowerPhoenix;
			break;

		default:
			ROS_ERROR("Invalid input in convertRevExternalFollower");
			return false;
	}
	return true;
}
	
} // namespace rev_convert

