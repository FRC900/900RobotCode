#include "talon_interface/canifier_state_interface.h"

namespace hardware_interface
{
namespace canifier
{
	CANifierHWState::CANifierHWState(int can_id)
		: can_id_(can_id)
		, quadrature_position_{0}
		, quadrature_velocity_{0}
		, velocity_measurement_period_{CANifierVelocityMeasPeriod::Period_100Ms}
		, velocity_measurement_window_{64}
		, clear_position_on_limit_f_{false}
		, clear_position_on_limit_r_{false}
		, clear_position_on_quad_idx_{false}
		, bus_voltage_{0.0}
		, faults_{0}
		, sticky_faults_{0}
		, encoder_ticks_per_rotation_{4096}
		, conversion_factor_{1.0}
	{
		for (auto &lo : led_output_)
			lo = 0;

		for (auto &gpo : general_pin_output_)
			gpo = false;
		for (auto &gpoe : general_pin_output_enable_)
			gpoe = false;
		for (auto &gpi : general_pin_input_)
			gpi = false;

		for (auto &po : pwm_output_)
			po = 0.0;
		for (auto &pe : pwm_output_enable_)
			pe = false;
		for (auto &pi : pwm_input_)
			pi = std::array<double, 2>{0.0, 0.0};

		status_frame_period_[CANifierStatusFrame_Status_1_General] = canifierstatusframe_status_1_general_default;
		status_frame_period_[CANifierStatusFrame_Status_2_General] = canifierstatusframe_status_2_general_default;
		status_frame_period_[CANifierStatusFrame_Status_3_PwmInputs0] = canifierstatusframe_status_3_pwminputs0_default;
		status_frame_period_[CANifierStatusFrame_Status_4_PwmInputs1] = canifierstatusframe_status_4_pwminputs1_default;
		status_frame_period_[CANifierStatusFrame_Status_5_PwmInputs2] = canifierstatusframe_status_5_pwminputs2_default;
		status_frame_period_[CANifierStatusFrame_Status_6_PwmInputs3] = canifierstatusframe_status_6_pwminputs3_default;
		status_frame_period_[CANifierStatusFrame_Status_8_Misc] = canifierstatusframe_status_8_misc_default;

		control_frame_period_[CANifier_Control_1_General] = canifier_control_1_general_default;
		control_frame_period_[CANifier_Control_2_PwmOutput] = canifier_control_2_pwmoutput_default;
	}
	int CANifierHWState::getCANId(void) const
	{
		return can_id_;
	}
	void CANifierHWState::setLEDOutput(LEDChannel led_channel, double percentOutput)
	{
		if ((led_channel <= LEDChannel::LEDChannelFirst) || (led_channel >= LEDChannel::LEDChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : led channel out of range");
			return;
		}
			led_output_[led_channel] = percentOutput;
	}
	double CANifierHWState::getLEDOutput(LEDChannel led_channel) const
	{
		if ((led_channel <= LEDChannel::LEDChannelFirst) || (led_channel >= LEDChannel::LEDChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : led channel out of range");
			return -std::numeric_limits<double>::max();
		}
		return led_output_[led_channel];
	}

	void CANifierHWState::setGeneralPinOutput(GeneralPin pin, bool value)
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return;
		}
		general_pin_output_[pin] = value;
	}
	bool CANifierHWState::getGeneralPinOutput(GeneralPin pin) const
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return false;
		}
		return general_pin_output_[pin];
	}
	void CANifierHWState::setGeneralPinOutputEnable(GeneralPin pin, bool output_enable)
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return;
		}
		general_pin_output_enable_[pin] = output_enable;
	}
	bool CANifierHWState::getGeneralPinOutputEnable(GeneralPin pin) const
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return false;
		}
		return general_pin_output_enable_[pin];
	}
	void CANifierHWState::setGeneralPinInput(GeneralPin pin, bool value)
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return;
		}
		general_pin_input_[pin] = value;
	}
	bool CANifierHWState::getGeneralPinInput(GeneralPin pin) const
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return false;
		}
		return general_pin_input_[pin];
	}

	void CANifierHWState::setQuadraturePosition(double position)
	{
		quadrature_position_ = position;
	}
	double CANifierHWState::getQuadraturePosition(void) const
	{
		return quadrature_position_;
	}

	void CANifierHWState::setQuadratureVelocity(double velocity)
	{
		quadrature_velocity_ = velocity;
	}
	double CANifierHWState::getQuadratureVelocity(void) const
	{
		return quadrature_velocity_;
	}

	void CANifierHWState::setVelocityMeasurementPeriod(CANifierVelocityMeasPeriod period)
	{
		velocity_measurement_period_ = period;
	}
	CANifierVelocityMeasPeriod CANifierHWState::getVelocityMeasurementPeriod(void) const
	{
		return velocity_measurement_period_;
	}

	void CANifierHWState::setVelocityMeasurementWindow(int window)
	{
		velocity_measurement_window_ = window;
	}
	int CANifierHWState::getVelocityMeasurementWindow(void) const
	{
		return velocity_measurement_window_;
	}

	void CANifierHWState::setClearPositionOnLimitF(bool value)
	{
		clear_position_on_limit_f_ = value;
	}
	bool CANifierHWState::getClearPositionOnLimitF(void) const
	{
		return clear_position_on_limit_f_;
	}

	void CANifierHWState::setClearPositionOnLimitR(bool value)
	{
		clear_position_on_limit_r_ = value;
	}
	bool CANifierHWState::getClearPositionOnLimitR(void) const
	{
		return clear_position_on_limit_r_;
	}

	void CANifierHWState::setClearPositionOnQuadIdx(bool value)
	{
		clear_position_on_quad_idx_ = value;
	}
	bool CANifierHWState::getClearPositionOnQuadIdx(void) const
	{
		return clear_position_on_quad_idx_;
	}

	void CANifierHWState::setBusVoltage(double bus_voltage)
	{
		bus_voltage_ = bus_voltage;
	}
	double CANifierHWState::getBusVoltage(void) const
	{
		return bus_voltage_;
	}

	void CANifierHWState::setPWMOutput(PWMChannel channel, double value)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return;
		}
		pwm_output_[channel] = value;
	}
	double CANifierHWState::getPWMOutput(PWMChannel channel) const
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return -std::numeric_limits<double>::max();
		}
		return pwm_output_[channel];
	}

	void CANifierHWState::setPWMOutputEnable(PWMChannel channel, bool value)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return;
		}
		pwm_output_enable_[channel] = value;

	}
	bool CANifierHWState::getPWMOutputEnable(PWMChannel channel) const
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return false;
		}
		return pwm_output_enable_[channel];
	}

	void CANifierHWState::setPWMInput(PWMChannel channel, const std::array<double, 2> &value)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return;
		}
		pwm_input_[channel] = value;
	}
	std::array<double, 2> CANifierHWState::getPWMInput(PWMChannel channel) const
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return std::array<double, 2>{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
		}
		return pwm_input_[channel];
	}

	void CANifierHWState::setStatusFramePeriod(CANifierStatusFrame frame_id, int period)
	{
		if ((frame_id <= CANifierStatusFrame::CANifierStatusFrame_First) ||
			(frame_id >= CANifierStatusFrame::CANifierStatusFrame_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM frame_id out of range");
			return;
		}
		status_frame_period_[frame_id] = period;
	}
	int CANifierHWState::getStatusFramePeriod(CANifierStatusFrame frame_id) const
	{
		if ((frame_id <= CANifierStatusFrame::CANifierStatusFrame_First) ||
			(frame_id >= CANifierStatusFrame::CANifierStatusFrame_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM frame_id out of range");
			return 0;
		}
		return status_frame_period_[frame_id];
	}

	void CANifierHWState::setControlFramePeriod(CANifierControlFrame frame_id, int period)
	{
		if ((frame_id <= CANifierControlFrame::CANifier_Control_First) ||
			(frame_id >= CANifierControlFrame::CANifier_Control_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM frame_id out of range");
			return;
		}
		control_frame_period_[frame_id] = period;
	}
	int CANifierHWState::getControlFramePeriod(CANifierControlFrame frame_id) const
	{
		if ((frame_id <= CANifierControlFrame::CANifier_Control_First) ||
			(frame_id >= CANifierControlFrame::CANifier_Control_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM frame_id out of range");
			return 0;
		}
		return control_frame_period_[frame_id];
	}

	void CANifierHWState::setFaults(unsigned int faults)
	{
		faults_ = faults;
	}
	unsigned int CANifierHWState::getFaults(void) const
	{
		return faults_;
	}
	void CANifierHWState::setStickyFaults(unsigned int sticky_faults)
	{
		sticky_faults_ = sticky_faults;
	}
	unsigned int CANifierHWState::getStickyFaults(void) const
	{
		return sticky_faults_;
	}

	void CANifierHWState::setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation)
	{
		encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
	}
	unsigned int CANifierHWState::getEncoderTicksPerRotation(void) const
	{
		return encoder_ticks_per_rotation_;
	}

	void CANifierHWState::setConversionFactor(double conversion_factor)
	{
		conversion_factor_ = conversion_factor;
	}
	double CANifierHWState::getConversionFactor(void) const
	{
		return conversion_factor_;
	}

} // namespace canifier
} // namespace hardware_interface
