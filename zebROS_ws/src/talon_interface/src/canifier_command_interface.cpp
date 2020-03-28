#include "talon_interface/canifier_command_interface.h"

namespace hardware_interface
{
namespace canifier
{
	CANifierHWCommand::CANifierHWCommand(void)
		: quadrature_position_{0}
		, quadrature_position_changed_{false}
		, velocity_measurement_period_{CANifierVelocityMeasPeriod::Period_100Ms}
		, velocity_measurement_period_changed_{false}
		, velocity_measurement_window_{64}
		, velocity_measurement_window_changed_{false}
		, clear_position_on_limit_f_{false}
		, clear_position_on_limit_f_changed_{false}
		, clear_position_on_limit_r_{false}
		, clear_position_on_limit_r_changed_{false}
		, clear_position_on_quad_idx_{false}
		, clear_position_on_quad_idx_changed_{false}
		, clear_sticky_faults_{false}
		, encoder_ticks_per_rotation_{4096}
		, conversion_factor_{1.0}
	{
		// Disable pwm output by defalt
		for (auto &lo : led_output_)
			lo = 0;
		for (auto &loc : led_output_changed_)
			loc = true;

		for (auto &gp : general_pin_output_)
			gp = false;
		for (auto &gpoe : general_pin_output_enable_)
			gpoe = false;
		for (auto &gpc : general_pin_changed_)
			gpc = false;

		for (auto &po : pwm_output_)
			po = 0.0;
		for (auto &poc : pwm_output_changed_)
			poc = false;

		// Disable pwm output by defalt
		for (auto &pe : pwm_output_enable_)
			pe = false;
		for (auto &pec : pwm_output_enable_changed_)
			pec = true;

		status_frame_period_[CANifierStatusFrame_Status_1_General] = canifierstatusframe_status_1_general_default;
		status_frame_period_[CANifierStatusFrame_Status_2_General] = canifierstatusframe_status_2_general_default;
		status_frame_period_[CANifierStatusFrame_Status_3_PwmInputs0] = canifierstatusframe_status_3_pwminputs0_default;
		status_frame_period_[CANifierStatusFrame_Status_4_PwmInputs1] = canifierstatusframe_status_4_pwminputs1_default;
		status_frame_period_[CANifierStatusFrame_Status_5_PwmInputs2] = canifierstatusframe_status_5_pwminputs2_default;
		status_frame_period_[CANifierStatusFrame_Status_6_PwmInputs3] = canifierstatusframe_status_6_pwminputs3_default;
		status_frame_period_[CANifierStatusFrame_Status_8_Misc] = canifierstatusframe_status_8_misc_default;
		for (auto &sfpc : status_frame_period_changed_)
			sfpc = true;

		control_frame_period_[CANifier_Control_1_General] = canifier_control_1_general_default;
		control_frame_period_[CANifier_Control_2_PwmOutput] = canifier_control_2_pwmoutput_default;
		for (auto &cfpc : control_frame_period_changed_)
			cfpc = false;
	}
	void CANifierHWCommand::setLEDOutput(LEDChannel led_channel, double percentOutput)
	{
		if ((led_channel <= LEDChannel::LEDChannelFirst) || (led_channel >= LEDChannel::LEDChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : led channel out of range");
			return;
		}
		if (fabs(percentOutput - led_output_[led_channel]) > 0.001)
		{
			led_output_[led_channel] = percentOutput;
			led_output_changed_[led_channel] = true;
		}
	}
	double CANifierHWCommand::getLEDOutput(LEDChannel led_channel) const
	{
		if ((led_channel <= LEDChannel::LEDChannelFirst) || (led_channel >= LEDChannel::LEDChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : led channel out of range");
			return -std::numeric_limits<double>::max();
		}
		return led_output_[led_channel];
	}
	bool CANifierHWCommand::ledOutputChanged(LEDChannel led_channel, double &percentOutput)
	{
		if ((led_channel <= LEDChannel::LEDChannelFirst) || (led_channel >= LEDChannel::LEDChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : led channel out of range");
			return false;
		}
		auto ret = led_output_changed_[led_channel];
		led_output_changed_[led_channel] = false;
		percentOutput = led_output_[led_channel];
		return ret;
	}
	void CANifierHWCommand::resetLEDOutput(LEDChannel led_channel)
	{
		if ((led_channel <= LEDChannel::LEDChannelFirst) || (led_channel >= LEDChannel::LEDChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : led channel out of range");
			return;
		}
		led_output_changed_[led_channel] = true;
	}

	void CANifierHWCommand::setGeneralPinOutput(GeneralPin pin, bool value)
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return;
		}
		if (general_pin_output_[pin] != value)
		{
			general_pin_output_[pin] = value;
			general_pin_changed_[pin] = true;
		}
	}
	bool CANifierHWCommand::getGeneralPinOutput(GeneralPin pin) const
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return false;
		}
		return general_pin_output_[pin];
	}
	void CANifierHWCommand::setGeneralPinOutputEnable(GeneralPin pin, bool output_enable)
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return;
		}
		if (general_pin_output_enable_[pin] != output_enable)
		{
			general_pin_output_enable_[pin] = output_enable;
			general_pin_changed_[pin] = true;
		}
	}
	bool CANifierHWCommand::getGeneralPinOutputEnable(GeneralPin pin) const
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return false;
		}
		return general_pin_output_enable_[pin];
	}
	bool CANifierHWCommand::generalPinOutputChanged(GeneralPin pin, bool &value, bool &output_enable)
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return false;
		}
		auto ret = general_pin_changed_[pin];
		general_pin_changed_[pin] = false;
		value = general_pin_output_[pin];
		output_enable = general_pin_output_enable_[pin];
		return ret;
	}
	void CANifierHWCommand::resetGeneralPinOutput(GeneralPin pin)
	{
		if ((pin <= GeneralPin::GeneralPin_FIRST) || (pin >= GeneralPin::GeneralPin_LAST))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : general channel pin out of range");
			return;
		}
		general_pin_changed_[pin] = true;
	}
	void CANifierHWCommand::setQuadraturePosition(double position)
	{
		if (fabs(quadrature_position_ - position) > 0.0001)
		{
			quadrature_position_ = position;
			quadrature_position_changed_ = true;
		}
	}
	double CANifierHWCommand::getQuadraturePosition(void) const
	{
		return quadrature_position_;
	}
	bool CANifierHWCommand::quadraturePositionChanged(double &position)
	{
		auto ret = quadrature_position_changed_;
		quadrature_position_changed_ = false;
		position = quadrature_position_;
		return ret;
	}
	void CANifierHWCommand::resetQuadraturePosition(void)
	{
		quadrature_position_changed_ = true;
	}

	void CANifierHWCommand::setVelocityMeasurementPeriod(CANifierVelocityMeasPeriod period)
	{
		if (velocity_measurement_period_ != period)
		{
			velocity_measurement_period_ = period;
			velocity_measurement_period_changed_ = true;
		}
	}
	CANifierVelocityMeasPeriod CANifierHWCommand::getVelocityMeasurementPeriod(void) const
	{
		return velocity_measurement_period_;
	}
	bool CANifierHWCommand::velocityMeasurementPeriodChanged(CANifierVelocityMeasPeriod &period)
	{
		auto ret = velocity_measurement_period_changed_;
		velocity_measurement_period_changed_ = false;
		period = velocity_measurement_period_;
		return ret;
	}
	void CANifierHWCommand::resetVelocityMeasurementPeriod(void)
	{
		velocity_measurement_period_changed_ = true;
	}

	void CANifierHWCommand::setVelocityMeasurementWindow(int window)
	{
		if (velocity_measurement_window_ != window)
		{
			velocity_measurement_window_ = window;
			velocity_measurement_window_changed_ = true;
		}
	}
	int CANifierHWCommand::getVelocityMeasurementWindow(void) const
	{
		return velocity_measurement_window_;
	}
	bool CANifierHWCommand::velocityMeasurementWindowChanged(int &window)
	{
		auto ret = velocity_measurement_window_changed_;
		velocity_measurement_window_changed_ = false;
		window = velocity_measurement_window_;
		return ret;
	}
	void CANifierHWCommand::resetVelocityMeasurementWindow(void)
	{
		velocity_measurement_window_changed_ = true;
	}

	void CANifierHWCommand::setClearPositionOnLimitF(bool value)
	{
		if (clear_position_on_limit_f_ != value)
		{
			clear_position_on_limit_f_ = value;
			clear_position_on_limit_f_changed_ = true;
		}
	}
	bool CANifierHWCommand::getClearPositionOnLimitF(void) const
	{
		return clear_position_on_limit_f_;
	}
	bool CANifierHWCommand::clearPositionOnLimitFChanged(bool &value)
	{
		auto ret = clear_position_on_limit_f_changed_;
		clear_position_on_limit_f_changed_ = false;
		value = clear_position_on_limit_f_;
		return ret;
	}
	void CANifierHWCommand::resetClearPositionOnLimitF(void)
	{
		clear_position_on_limit_f_changed_ = true;
	}

	void CANifierHWCommand::setClearPositionOnLimitR(bool value)
	{
		if (clear_position_on_limit_r_ != value)
		{
			clear_position_on_limit_r_ = value;
			clear_position_on_limit_r_changed_ = true;
		}
	}
	bool CANifierHWCommand::getClearPositionOnLimitR(void) const
	{
		return clear_position_on_limit_r_;
	}
	bool CANifierHWCommand::clearPositionOnLimitRChanged(bool &value)
	{
		auto ret = clear_position_on_limit_r_changed_;
		clear_position_on_limit_r_changed_ = false;
		value = clear_position_on_limit_r_;
		return ret;
	}
	void CANifierHWCommand::resetClearPositionOnLimitR(void)
	{
		clear_position_on_limit_r_changed_ = true;
	}

	void CANifierHWCommand::setClearPositionOnQuadIdx(bool value)
	{
		if (clear_position_on_quad_idx_ != value)
		{
			clear_position_on_quad_idx_ = value;
			clear_position_on_quad_idx_changed_ = true;
		}
	}
	bool CANifierHWCommand::getClearPositionOnQuadIdx(void) const
	{
		return clear_position_on_quad_idx_;
	}
	bool CANifierHWCommand::clearPositionOnQuadIdxChanged(bool &value)
	{
		auto ret = clear_position_on_quad_idx_changed_;
		clear_position_on_quad_idx_changed_ = false;
		value = clear_position_on_quad_idx_;
		return ret;
	}
	void CANifierHWCommand::resetClearPositionOnQuadIdx(void)
	{
		clear_position_on_quad_idx_changed_ = true;
	}

	void CANifierHWCommand::setPWMOutput(PWMChannel channel, double value)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return;
		}
		if (fabs(pwm_output_[channel] - value) > 0.0001)
		{
			pwm_output_[channel] = value;
			pwm_output_changed_[channel] = true;
		}
	}
	double CANifierHWCommand::getPWMOutput(PWMChannel channel) const
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return -std::numeric_limits<double>::max();
		}
		return pwm_output_[channel];
	}
	bool CANifierHWCommand::pwmOutputChanged(PWMChannel channel, double &value)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return false;
		}
		auto ret = pwm_output_changed_[channel];
		pwm_output_changed_[channel] = false;
		value = pwm_output_[channel];
		return ret;
	}
	void  CANifierHWCommand::resetPWMOutput(PWMChannel channel)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return;
		}
		pwm_output_changed_[channel] = true;
	}

	void CANifierHWCommand::setPWMOutputEnable(PWMChannel channel, bool value)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return;
		}
		if (pwm_output_enable_[channel] != value)
		{
			pwm_output_enable_[channel] = value;
			pwm_output_enable_changed_[channel] = true;
		}
	}
	bool CANifierHWCommand::getPWMOutputEnable(PWMChannel channel) const
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return false;
		}
		return pwm_output_enable_[channel];
	}
	bool CANifierHWCommand::pwmOutputEnableChanged(PWMChannel channel, bool &value)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return false;
		}
		auto ret = pwm_output_enable_changed_[channel];
		pwm_output_enable_changed_[channel] = false;
		value = pwm_output_enable_[channel];
		return ret;
	}
	void  CANifierHWCommand::resetPWMOutputEnable(PWMChannel channel)
	{
		if ((channel <= PWMChannel::PWMChannelFirst) || (channel >= PWMChannel::PWMChannelLast))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : PWM channel out of range");
			return;
		}
		pwm_output_enable_changed_[channel] = true;
	}

	void CANifierHWCommand::setStatusFramePeriod(CANifierStatusFrame frame_id, int period)
	{
		if ((frame_id <= CANifierStatusFrame::CANifierStatusFrame_First) ||
			(frame_id >= CANifierStatusFrame::CANifierStatusFrame_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return;
		}
		if (status_frame_period_[frame_id] != period)
		{
			status_frame_period_[frame_id] = period;
			status_frame_period_changed_[frame_id] = true;
		}
	}
	int CANifierHWCommand::getStatusFramePeriod(CANifierStatusFrame frame_id) const
	{
		if ((frame_id <= CANifierStatusFrame::CANifierStatusFrame_First) ||
			(frame_id >= CANifierStatusFrame::CANifierStatusFrame_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return 0;
		}
		return status_frame_period_[frame_id];
	}
	bool CANifierHWCommand::statusFramePeriodChanged(CANifierStatusFrame frame_id, int &period)
	{
		if ((frame_id <= CANifierStatusFrame::CANifierStatusFrame_First) ||
			(frame_id >= CANifierStatusFrame::CANifierStatusFrame_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return false;
		}
		auto ret = status_frame_period_changed_[frame_id];
		status_frame_period_changed_[frame_id] = false;
		period = status_frame_period_[frame_id];
		return ret;
	}
	void CANifierHWCommand::resetStatusFramePeriod(CANifierStatusFrame frame_id)
	{
		if ((frame_id <= CANifierStatusFrame::CANifierStatusFrame_First) ||
			(frame_id >= CANifierStatusFrame::CANifierStatusFrame_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return;
		}
		status_frame_period_changed_[frame_id] = true;
	}

	void CANifierHWCommand::setControlFramePeriod(CANifierControlFrame frame_id, int period)
	{
		if ((frame_id <= CANifierControlFrame::CANifier_Control_First) ||
			(frame_id >= CANifierControlFrame::CANifier_Control_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return;
		}
		if (control_frame_period_[frame_id] != period)
		{
			control_frame_period_[frame_id] = period;
			control_frame_period_changed_[frame_id] = true;
		}
	}
	int CANifierHWCommand::getControlFramePeriod(CANifierControlFrame frame_id) const
	{
		if ((frame_id <= CANifierControlFrame::CANifier_Control_First) ||
			(frame_id >= CANifierControlFrame::CANifier_Control_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return 0;
		}
		return control_frame_period_[frame_id];
	}
	bool CANifierHWCommand::controlFramePeriodChanged(CANifierControlFrame frame_id, int &period)
	{
		if ((frame_id <= CANifierControlFrame::CANifier_Control_First) ||
			(frame_id >= CANifierControlFrame::CANifier_Control_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return false;
		}
		auto ret = control_frame_period_changed_[frame_id];
		control_frame_period_changed_[frame_id] = false;
		period = control_frame_period_[frame_id];
		return ret;
	}
	void CANifierHWCommand::resetControlFramePeriod(CANifierControlFrame frame_id)
	{
		if ((frame_id <= CANifierControlFrame::CANifier_Control_First) ||
			(frame_id >= CANifierControlFrame::CANifier_Control_Last))
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : frame_id out of range");
			return;
		}
		control_frame_period_changed_[frame_id] = true;
	}


	// One-shot control clear sticky faults has slightly different semantics than the rest
	void CANifierHWCommand::setClearStickyFaults(void)
	{
		clear_sticky_faults_ = true;
	}
	bool CANifierHWCommand::getClearStickyFaults(void) const
	{
		return clear_sticky_faults_;
	}
	bool CANifierHWCommand::clearStickyFaultsChanged(void)
	{
		auto ret = clear_sticky_faults_;
		clear_sticky_faults_ = false;
		return ret;
	}

	void CANifierHWCommand::setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation)
	{
		encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
	}
	unsigned int CANifierHWCommand::getEncoderTicksPerRotation(void) const
	{
		return encoder_ticks_per_rotation_;
	}

	void CANifierHWCommand::setConversionFactor(double conversion_factor)
	{
		conversion_factor_ = conversion_factor;
	}
	double CANifierHWCommand::getConversionFactor(void) const
	{
		return conversion_factor_;
	}
} // namespace canifier
} // namespace hardware_interface
