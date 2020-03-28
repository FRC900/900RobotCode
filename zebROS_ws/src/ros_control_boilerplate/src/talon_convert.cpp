#include "ros_control_boilerplate/talon_convert.h"

namespace talon_convert
{
bool TalonConvert::controlMode(
		const hardware_interface::TalonMode input_mode,
		ctre::phoenix::motorcontrol::ControlMode &output_mode) const
{
	switch (input_mode)
	{
		case hardware_interface::TalonMode_PercentOutput:
			output_mode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
			break;
		case hardware_interface::TalonMode_Position:      // CloseLoop
			output_mode = ctre::phoenix::motorcontrol::ControlMode::Position;
			break;
		case hardware_interface::TalonMode_Velocity:      // CloseLoop
			output_mode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
			break;
		case hardware_interface::TalonMode_Current:       // CloseLoop
			output_mode = ctre::phoenix::motorcontrol::ControlMode::Current;
			break;
		case hardware_interface::TalonMode_Follower:
			output_mode = ctre::phoenix::motorcontrol::ControlMode::Follower;
			break;
		case hardware_interface::TalonMode_MotionProfile:
			output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionProfile;
			break;
		case hardware_interface::TalonMode_MotionMagic:
			output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
			break;
		case hardware_interface::TalonMode_MotionProfileArc:
			output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionProfileArc;
			break;
		case hardware_interface::TalonMode_Disabled:
			output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
			break;
		default:
			output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
			ROS_WARN("Unknown mode seen in HW interface");
			return false;
	}
	return true;
}

bool TalonConvert::demand1Type(
		const hardware_interface::DemandType input,
		ctre::phoenix::motorcontrol::DemandType &output) const
{
	switch(input)
	{
		case hardware_interface::DemandType::DemandType_Neutral:
			output = ctre::phoenix::motorcontrol::DemandType::DemandType_Neutral;
			break;
		case hardware_interface::DemandType::DemandType_AuxPID:
			output = ctre::phoenix::motorcontrol::DemandType::DemandType_AuxPID;
			break;
		case hardware_interface::DemandType::DemandType_ArbitraryFeedForward:
			output = ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward;
			break;
		default:
			output = ctre::phoenix::motorcontrol::DemandType::DemandType_Neutral;
			ROS_WARN("Unknown demand1 type seen in HW interface");
			return false;
	}
	return true;
}

bool TalonConvert::neutralMode(
		const hardware_interface::NeutralMode input_mode,
		ctre::phoenix::motorcontrol::NeutralMode &output_mode) const
{
	switch (input_mode)
	{
		case hardware_interface::NeutralMode_EEPROM_Setting:
			output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
			break;
		case hardware_interface::NeutralMode_Coast:
			output_mode = ctre::phoenix::motorcontrol::Coast;
			break;
		case hardware_interface::NeutralMode_Brake:
			output_mode = ctre::phoenix::motorcontrol::Brake;
			break;
		default:
			output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
			ROS_WARN("Unknown neutral mode seen in HW interface");
			return false;
	}

	return true;
}

bool TalonConvert::feedbackDevice(
		const hardware_interface::FeedbackDevice input_fd,
		ctre::phoenix::motorcontrol::FeedbackDevice &output_fd) const
{
	switch (input_fd)
	{
		case hardware_interface::FeedbackDevice_QuadEncoder:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder;
			break;
		case hardware_interface::FeedbackDevice_IntegratedSensor:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
			break;
		case hardware_interface::FeedbackDevice_Analog:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::Analog;
			break;
		case hardware_interface::FeedbackDevice_Tachometer:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::Tachometer;
			break;
		case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::PulseWidthEncodedPosition;
			break;
		case hardware_interface::FeedbackDevice_SensorSum:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::SensorSum;
			break;
		case hardware_interface::FeedbackDevice_SensorDifference:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::SensorDifference;
			break;
		case hardware_interface::FeedbackDevice_RemoteSensor0:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
			break;
		case hardware_interface::FeedbackDevice_RemoteSensor1:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor1;
			break;
		case hardware_interface::FeedbackDevice_None:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::None;
			break;
		case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
			output_fd = ctre::phoenix::motorcontrol::FeedbackDevice::SoftwareEmulatedSensor;
			break;
		default:
			ROS_WARN("Unknown feedback device seen in HW interface");
			return false;
	}
	return true;
}

bool TalonConvert::remoteFeedbackDevice(
		const hardware_interface::RemoteFeedbackDevice input_fd,
		ctre::phoenix::motorcontrol::RemoteFeedbackDevice &output_fd) const
{
	switch (input_fd)
	{
		case hardware_interface::RemoteFeedbackDevice_SensorSum:
			output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::SensorSum;
			break;
		case hardware_interface::RemoteFeedbackDevice_SensorDifference:
			output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::SensorDifference;
			break;
		case hardware_interface::RemoteFeedbackDevice_RemoteSensor0:
			output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor0;
			break;
		case hardware_interface::RemoteFeedbackDevice_RemoteSensor1:
			output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::RemoteSensor1;
			break;
		case hardware_interface::RemoteFeedbackDevice_None:
			output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::None;
			break;
		case hardware_interface::RemoteFeedbackDevice_SoftwareEmulatedSensor:
			output_fd = ctre::phoenix::motorcontrol::RemoteFeedbackDevice::SoftwareEmulatedSensor;
			break;
		default:
			ROS_WARN("Unknown remote feedback device seen in HW interface");
			return false;
	}

	return true;
}

bool TalonConvert::remoteSensorSource(
		const hardware_interface::RemoteSensorSource input_rss,
		ctre::phoenix::motorcontrol::RemoteSensorSource &output_rss) const
{
	switch (input_rss)
	{
		case hardware_interface::RemoteSensorSource_Off:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Off;
			break;
		case hardware_interface::RemoteSensorSource_TalonSRX_SelectedSensor:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor;
			break;
		case hardware_interface::RemoteSensorSource_Pigeon_Yaw:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw;
			break;
		case hardware_interface::RemoteSensorSource_Pigeon_Pitch:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch;
			break;
		case hardware_interface::RemoteSensorSource_Pigeon_Roll:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Pigeon_Roll;
			break;
		case hardware_interface::RemoteSensorSource_CANifier_Quadrature:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature;
			break;
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput0:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0;
			break;
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput1:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1;
			break;
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput2:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2;
			break;
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput3:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3;
			break;
		case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Yaw:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw;
			break;
		case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Pitch:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch;
			break;
		case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Roll:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll;
			break;
		case hardware_interface::RemoteSensorSource_CANCoder:
			output_rss = ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_CANCoder;
			break;

		default:
			ROS_WARN("Unknown remote sensor source seen in HW interface");
			return false;
	}

	return true;
}

bool TalonConvert::limitSwitchSource(
		const hardware_interface::LimitSwitchSource input_ls,
		ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls) const
{
	switch (input_ls)
	{
		case hardware_interface::LimitSwitchSource_FeedbackConnector:
			output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
			break;
		case hardware_interface::LimitSwitchSource_RemoteTalonSRX:
			output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
			break;
		case hardware_interface::LimitSwitchSource_RemoteCANifier:
			output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteCANifier;
			break;
		case hardware_interface::LimitSwitchSource_Deactivated:
			output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
			break;
		default:
			ROS_WARN("Unknown limit switch source seen in HW interface");
			return false;
	}
	return true;
}

bool TalonConvert::remoteLimitSwitchSource(
		const hardware_interface::RemoteLimitSwitchSource input_ls,
		ctre::phoenix::motorcontrol::RemoteLimitSwitchSource &output_ls) const
{
	switch (input_ls)
	{
		case hardware_interface::RemoteLimitSwitchSource_RemoteTalonSRX:
			output_ls = ctre::phoenix::motorcontrol::RemoteLimitSwitchSource_RemoteTalonSRX;
			break;
		case hardware_interface::RemoteLimitSwitchSource_RemoteCANifier:
			output_ls = ctre::phoenix::motorcontrol::RemoteLimitSwitchSource_RemoteCANifier;
			break;
		case hardware_interface::RemoteLimitSwitchSource_Deactivated:
			output_ls = ctre::phoenix::motorcontrol::RemoteLimitSwitchSource_Deactivated;
			break;
		default:
			ROS_WARN("Unknown remote limit switch source seen in HW interface");
			return false;
	}
	return true;
}

bool TalonConvert::limitSwitchNormal(
		const hardware_interface::LimitSwitchNormal input_ls,
		ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls) const
{
	switch (input_ls)
	{
		case hardware_interface::LimitSwitchNormal_NormallyOpen:
			output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
			break;
		case hardware_interface::LimitSwitchNormal_NormallyClosed:
			output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
			break;
		case hardware_interface::LimitSwitchNormal_Disabled:
			output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
			break;
		default:
			ROS_WARN("Unknown limit switch normal seen in HW interface");
			return false;
	}
	return true;

}

bool TalonConvert::velocityMeasurementPeriod(const hardware_interface::VelocityMeasurementPeriod input_v_m_p,
		ctre::phoenix::motorcontrol::VelocityMeasPeriod &output_v_m_period) const
{
	switch(input_v_m_p)
	{
		case hardware_interface::VelocityMeasurementPeriod::Period_1Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_1Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_2Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_2Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_5Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_10Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_10Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_20Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_20Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_25Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_25Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_50Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_50Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_100Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_100Ms;
			break;
		default:
			ROS_WARN("Unknown velocity measurement period seen in HW interface");
			return false;
	}
	return true;
}

bool TalonConvert::statusFrame(const hardware_interface::StatusFrame input,
		ctre::phoenix::motorcontrol::StatusFrameEnhanced &output) const
{
	switch (input)
	{
		case hardware_interface::Status_1_General:
			output = ctre::phoenix::motorcontrol::Status_1_General;
			break;
		case hardware_interface::Status_2_Feedback0:
			output = ctre::phoenix::motorcontrol::Status_2_Feedback0;
			break;
		case hardware_interface::Status_3_Quadrature:
			output = ctre::phoenix::motorcontrol::Status_3_Quadrature;
			break;
		case hardware_interface::Status_4_AinTempVbat:
			output = ctre::phoenix::motorcontrol::Status_4_AinTempVbat;
			break;
		case hardware_interface::Status_6_Misc:
			output = ctre::phoenix::motorcontrol::Status_6_Misc;
			break;
		case hardware_interface::Status_7_CommStatus:
			output = ctre::phoenix::motorcontrol::Status_7_CommStatus;
			break;
		case hardware_interface::Status_8_PulseWidth:
			output = ctre::phoenix::motorcontrol::Status_8_PulseWidth;
			break;
		case hardware_interface::Status_9_MotProfBuffer:
			output = ctre::phoenix::motorcontrol::Status_9_MotProfBuffer;
			break;
		case hardware_interface::Status_10_MotionMagic:
			output = ctre::phoenix::motorcontrol::Status_10_MotionMagic;
			break;
		case hardware_interface::Status_11_UartGadgeteer:
			output = ctre::phoenix::motorcontrol::Status_11_UartGadgeteer;
			break;
		case hardware_interface::Status_12_Feedback1:
			output = ctre::phoenix::motorcontrol::Status_12_Feedback1;
			break;
		case hardware_interface::Status_13_Base_PIDF0:
			output = ctre::phoenix::motorcontrol::Status_13_Base_PIDF0;
			break;
		case hardware_interface::Status_14_Turn_PIDF1:
			output = ctre::phoenix::motorcontrol::Status_14_Turn_PIDF1;
			break;
		case hardware_interface::Status_15_FirmwareApiStatus:
			output = ctre::phoenix::motorcontrol::Status_15_FirmareApiStatus;
			break;
		case hardware_interface::Status_17_Targets1:
			output = ctre::phoenix::motorcontrol::Status_17_Targets1;
			break;
		case hardware_interface::Status_Brushless_Current:
			output = ctre::phoenix::motorcontrol::Status_Brushless_Current;
			break;
		default:
			ROS_ERROR("Invalid input in convertStatusFrame");
			return false;
	}
	return true;
}

bool TalonConvert::controlFrame(const hardware_interface::ControlFrame input,
		ctre::phoenix::motorcontrol::ControlFrame &output) const
{
	switch (input)
	{
		case hardware_interface::Control_3_General:
			output = ctre::phoenix::motorcontrol::Control_3_General;
			break;
		case hardware_interface::Control_4_Advanced:
			output = ctre::phoenix::motorcontrol::Control_4_Advanced;
			break;
#if 0 // There's no SetControlFramePeriod which takes an enhanced ControlFrame, so this is out for now
		case hardware_interface::Control_5_FeedbackOutputOverride:
			output = ctre::phoenix::motorcontrol::Control_5_FeedbackOutputOverride_;
			break;
#endif
		case hardware_interface::Control_6_MotProfAddTrajPoint:
			output = ctre::phoenix::motorcontrol::Control_6_MotProfAddTrajPoint;
			break;
		default:
			ROS_ERROR("Invalid input in convertControlFrame");
			return false;
	}
	return true;
}

bool TalonConvert::motorCommutation(const hardware_interface::MotorCommutation input,
		ctre::phoenix::motorcontrol::MotorCommutation &output) const
{
	switch (input)
	{
		case hardware_interface::MotorCommutation::Trapezoidal:
			output = ctre::phoenix::motorcontrol::MotorCommutation::Trapezoidal;
			break;
		default:
			ROS_ERROR("Invalid input in convertMotorCommutation");
			return false;
	}
	return true;
}

bool TalonConvert::absoluteSensorRange(const hardware_interface::AbsoluteSensorRange input,
		ctre::phoenix::sensors::AbsoluteSensorRange &output) const
{
	switch (input)
	{
		case hardware_interface::Unsigned_0_to_360:
			output = ctre::phoenix::sensors::Unsigned_0_to_360;
			break;
		case hardware_interface::Signed_PlusMinus180:
			output = ctre::phoenix::sensors::Signed_PlusMinus180;
			break;
		default:
			ROS_ERROR("Invalid input in convertAbsoluteSensorRange");
			return false;
	}
	return true;
}

bool TalonConvert::sensorInitializationStrategy(const hardware_interface::SensorInitializationStrategy input,
		ctre::phoenix::sensors::SensorInitializationStrategy &output) const
{
	switch (input)
	{
		case hardware_interface::BootToZero:
			output = ctre::phoenix::sensors::BootToZero;
			break;
		case hardware_interface::BootToAbsolutePosition:
			output = ctre::phoenix::sensors::BootToAbsolutePosition;
			break;
		default:
			ROS_ERROR("Invalid input in convertSensorInitializationStrategy");
			return false;
	}
	return true;
}


} // namespace talon_convert
