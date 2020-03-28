#include "ros_control_boilerplate/canifier_convert.h"

namespace canifier_convert
{
bool CANifierConvert::generalPin(const hardware_interface::canifier::GeneralPin input,
		ctre::phoenix::CANifier::GeneralPin &output) const
{
	switch (input)
	{
		case hardware_interface::canifier::GeneralPin::QUAD_IDX:
			output = ctre::phoenix::CANifier::GeneralPin::QUAD_IDX;
			break;
		case hardware_interface::canifier::GeneralPin::QUAD_B:
			output = ctre::phoenix::CANifier::GeneralPin::QUAD_B;
			break;
		case hardware_interface::canifier::GeneralPin::QUAD_A:
			output = ctre::phoenix::CANifier::GeneralPin::QUAD_A;
			break;
		case hardware_interface::canifier::GeneralPin::LIMR:
			output = ctre::phoenix::CANifier::GeneralPin::LIMR;
			break;
		case hardware_interface::canifier::GeneralPin::LIMF:
			output = ctre::phoenix::CANifier::GeneralPin::LIMF;
			break;
		case hardware_interface::canifier::GeneralPin::SDA:
			output = ctre::phoenix::CANifier::GeneralPin::SDA;
			break;
		case hardware_interface::canifier::GeneralPin::SCL:
			output = ctre::phoenix::CANifier::GeneralPin::SCL;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_CS:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_CS;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_MISO_PWM2P:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_MISO_PWM2P;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_MOSI_PWM1P:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_MOSI_PWM1P;
			break;
		case hardware_interface::canifier::GeneralPin::SPI_CLK_PWM0P:
			output = ctre::phoenix::CANifier::GeneralPin::SPI_CLK_PWM0P;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierGeneralPin");
			return false;
	}
	return true;
}
bool CANifierConvert::PWMChannel(hardware_interface::canifier::PWMChannel input,
		ctre::phoenix::CANifier::PWMChannel &output) const
{
	switch (input)
	{
		case hardware_interface::canifier::PWMChannel::PWMChannel0:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel0;
			break;
		case hardware_interface::canifier::PWMChannel::PWMChannel1:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel1;
			break;
		case hardware_interface::canifier::PWMChannel::PWMChannel2:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel2;
			break;
		case hardware_interface::canifier::PWMChannel::PWMChannel3:
			output = ctre::phoenix::CANifier::PWMChannel::PWMChannel3;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierPWMChannel");
			return false;
	}
	return true;
}

bool CANifierConvert::LEDChannel(hardware_interface::canifier::LEDChannel input,
		ctre::phoenix::CANifier::LEDChannel &output) const
{
	switch (input)
	{
		case hardware_interface::canifier::LEDChannel::LEDChannelA:
			output = ctre::phoenix::CANifier::LEDChannel::LEDChannelA;
			break;
		case hardware_interface::canifier::LEDChannel::LEDChannelB:
			output = ctre::phoenix::CANifier::LEDChannel::LEDChannelB;
			break;
		case hardware_interface::canifier::LEDChannel::LEDChannelC:
			output = ctre::phoenix::CANifier::LEDChannel::LEDChannelC;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierLEDChannel");
			return false;
	}
	return true;
}

bool CANifierConvert::velocityMeasurementPeriod(hardware_interface::canifier::CANifierVelocityMeasPeriod input,
		ctre::phoenix::CANifierVelocityMeasPeriod &output) const
{
	switch (input)
	{
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_1Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_1Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_2Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_2Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_5Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_5Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_10Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_10Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_20Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_20Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_25Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_25Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_50Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_50Ms;
			break;
		case hardware_interface::canifier::CANifierVelocityMeasPeriod::Period_100Ms:
			output = ctre::phoenix::CANifierVelocityMeasPeriod::Period_100Ms;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierVelocityMeasurementPeriod");
			return false;
	}
	return true;
}

bool CANifierConvert::statusFrame(hardware_interface::canifier::CANifierStatusFrame input,
		ctre::phoenix::CANifierStatusFrame &output) const
{
	switch (input)
	{
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_1_General:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_1_General;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_2_General:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_2_General;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3;
			break;
		case hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc:
			output = ctre::phoenix::CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierStatusFrame");
			return false;
	}
	return true;
}

bool CANifierConvert::controlFrame(hardware_interface::canifier::CANifierControlFrame input,
		ctre::phoenix::CANifierControlFrame &output) const
{
	switch (input)
	{
		case hardware_interface::canifier::CANifierControlFrame::CANifier_Control_1_General:
			output = ctre::phoenix::CANifierControlFrame::CANifier_Control_1_General;
			break;
		case hardware_interface::canifier::CANifierControlFrame::CANifier_Control_2_PwmOutput:
			output = ctre::phoenix::CANifierControlFrame::CANifier_Control_2_PwmOutput;
			break;
		default:
			ROS_ERROR("Invalid input in convertCANifierControlFrame");
			return false;
	}
	return true;
}

} // namespace canifier_convert
