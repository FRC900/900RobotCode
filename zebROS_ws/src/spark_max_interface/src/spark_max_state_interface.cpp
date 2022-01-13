#include "spark_max_interface/spark_max_state_interface.h"

namespace hardware_interface
{
SparkMaxHWState::SparkMaxHWState(int device_id, MotorType motor_type)
	: device_id_(device_id)
	  , motor_type_(motor_type)
{
	p_gain_.fill(0);
	i_gain_.fill(0);
	d_gain_.fill(0);
	f_gain_.fill(0);
	i_zone_.fill(0);
	d_filter_.fill(0);
	pidf_output_min_.fill(-1);
	pidf_output_max_.fill(1);
	pidf_reference_value_.fill(0);
	pidf_reference_ctrl_.fill(kDutyCycle);
	pidf_arb_feed_forward_.fill(0);
	pidf_arb_feed_forward_units_.fill(ArbFFUnits::kVoltage);
}

int SparkMaxHWState::getDeviceId(void) const
{
	return device_id_;
}
MotorType SparkMaxHWState::getMotorType(void) const
{
	return motor_type_;
}
void SparkMaxHWState::setSetPoint(double set_point)
{
	set_point_ = set_point;
}
double SparkMaxHWState::getSetPoint(void) const
{
	return set_point_;
}

void SparkMaxHWState::setInverted(bool inverted)
{
	inverted_ = inverted;
}
bool SparkMaxHWState::getInverted(void) const
{
	return inverted_;
}

void SparkMaxHWState::setPosition(double position)
{
	position_ = position;
}
double SparkMaxHWState::getPosition(void) const
{
	return position_;
}

void SparkMaxHWState::setVelocity(double velocity)
{
	velocity_ = velocity;
}
double SparkMaxHWState::getVelocity(void) const
{
	return velocity_;
}

void SparkMaxHWState::setPGain(size_t slot, double p_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPGain() : invalid slot " << slot);
		return;
	}
	p_gain_[slot] = p_gain;
}
double SparkMaxHWState::getPGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPGain() : invalid slot " << slot);
		return -1;
	}
	return p_gain_[slot];
}

void SparkMaxHWState::setIGain(size_t slot, double i_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setIGain() : invalid slot " << slot);
		return;
	}
	i_gain_[slot] = i_gain;
}
double SparkMaxHWState::getIGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getIGain() : invalid slot " << slot);
		return -1;
	}
	return i_gain_[slot];
}

void SparkMaxHWState::setDGain(size_t slot, double d_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setDGain() : invalid slot " << slot);
		return;
	}
	d_gain_[slot] = d_gain;
}
double SparkMaxHWState::getDGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getDGain() : invalid slot " << slot);
		return -1;
	}
	return d_gain_[slot];
}

void SparkMaxHWState::setFGain(size_t slot, double f_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setFGain() : invalid slot " << slot);
		return;
	}
	f_gain_[slot] = f_gain;
}
double SparkMaxHWState::getFGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getFGain() : invalid slot " << slot);
		return -1;
	}
	return f_gain_[slot];
}

void SparkMaxHWState::setIZone(size_t slot, double i_zone)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setIZone() : invalid slot " << slot);
		return;
	}
	i_zone_[slot] = i_zone;
}
double SparkMaxHWState::getIZone(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getIZone() : invalid slot " << slot);
		return -1;
	}
	return i_zone_[slot];
}

void SparkMaxHWState::setDFilter(size_t slot, double d_filter)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setDFilter() : invalid slot " << slot);
		return;
	}
	d_filter_[slot] = d_filter;
}
double SparkMaxHWState::getDFilter(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getDFilter() : invalid slot " << slot);
		return -1;
	}
	return d_filter_[slot];
}

void SparkMaxHWState::setPIDFOutputMin(size_t slot, double pidf_output_min)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFOutputMin() : invalid slot " << slot);
		return;
	}
	pidf_output_min_[slot] = pidf_output_min;
}
double SparkMaxHWState::getPIDFOutputMin(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFOutputMin() : invalid slot " << slot);
		return std::numeric_limits<double>::max();
	}
	return pidf_output_min_[slot];
}

void SparkMaxHWState::setPIDFOutputMax(size_t slot, double pidf_output_max)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFOutputMax() : invalid slot " << slot);
		return;
	}
	pidf_output_max_[slot] = pidf_output_max;
}
double SparkMaxHWState::getPIDFOutputMax(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFOutputMax() : invalid slot " << slot);
		return -std::numeric_limits<double>::max();
	}
	return pidf_output_max_[slot];
}

void SparkMaxHWState::setPIDFReferenceOutput(size_t slot, double pidf_reference_value)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceOutput() : invalid slot " << slot);
		return;
	}
	pidf_reference_value_[slot] = pidf_reference_value;
}
double SparkMaxHWState::getPIDFReferenceOutput(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFReferenceOutput() : invalid slot " << slot);
		return 0;
	}
	return SparkMaxHWState::pidf_reference_value_[slot];
}

void SparkMaxHWState::setPIDFReferenceCtrl(size_t slot, ControlType pidf_reference_ctrl)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceCtrl() : invalid slot " << slot);
		return;
	}
	pidf_reference_ctrl_[slot] = pidf_reference_ctrl;
}
ControlType SparkMaxHWState::getPIDFReferenceCtrl(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFReferenceCtrl() : invalid slot " << slot);
		return kDutyCycle;
	}
	return pidf_reference_ctrl_[slot];
}

void SparkMaxHWState::setPIDFArbFeedForward(size_t slot, double pidf_arb_feed_forward)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFArbFeedForward() : invalid slot " << slot);
		return;
	}
	pidf_arb_feed_forward_[slot] = pidf_arb_feed_forward;
}
double SparkMaxHWState::getPIDFArbFeedForward(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFArbFeedForward() : invalid slot " << slot);
		return -1;
	}
	return pidf_arb_feed_forward_[slot];
}

void SparkMaxHWState::setPIDFArbFeedForwardUnits(size_t slot, ArbFFUnits pidf_arb_feed_forward_units)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFArbFeedForwardUnits() : invalid slot " << slot);
		return;
	}
	pidf_arb_feed_forward_units_[slot] = pidf_arb_feed_forward_units;
}
ArbFFUnits SparkMaxHWState::getPIDFArbFeedForwardUnits(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFArbFeedForwardUnits() : invalid slot " << slot);
		return ArbFFUnits::kVoltage;
	}
	return pidf_arb_feed_forward_units_[slot];
}
void SparkMaxHWState::setPIDFReferenceSlot(size_t slot)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceSlot() : invalid slot " << slot);
		return;
	}
	pidf_reference_slot_ = slot;
}
int SparkMaxHWState::getPIDFReferenceSlot(void) const
{
	return pidf_reference_slot_;
}


void SparkMaxHWState::setForwardLimitSwitchPolarity(LimitSwitchPolarity forward_limit_switch_polarity)
{
	forward_limit_switch_polarity_ = forward_limit_switch_polarity;
}
LimitSwitchPolarity SparkMaxHWState::getForwardLimitSwitchPolarity(void) const
{
	return forward_limit_switch_polarity_;
}

void SparkMaxHWState::setForwardLimitSwitchEnabled(bool forward_limit_switch_enabled)
{
	forward_limit_switch_enabled_ = forward_limit_switch_enabled;
}
bool SparkMaxHWState::getForwardLimitSwitchEnabled(void) const
{
	return forward_limit_switch_enabled_;
}

void SparkMaxHWState::setForwardLimitSwitch(bool forward_limit_switch)
{
	forward_limit_switch_ = forward_limit_switch;
}
bool SparkMaxHWState::getForwardLimitSwitch(void) const
{
	return forward_limit_switch_;
}

void SparkMaxHWState::setReverseLimitSwitchPolarity(LimitSwitchPolarity reverse_limit_switch_polarity)
{
	reverse_limit_switch_polarity_ = reverse_limit_switch_polarity;
}
LimitSwitchPolarity SparkMaxHWState::getReverseLimitSwitchPolarity(void) const
{
	return reverse_limit_switch_polarity_;
}

void SparkMaxHWState::setReverseLimitSwitchEnabled(bool reverse_limit_switch_enabled)
{
	reverse_limit_switch_enabled_ = reverse_limit_switch_enabled;
}
bool SparkMaxHWState::getReverseLimitSwitchEnabled(void) const
{
	return reverse_limit_switch_enabled_;
}

void SparkMaxHWState::setReverseLimitSwitch(bool reverse_limit_switch)
{
	reverse_limit_switch_ = reverse_limit_switch;
}
bool SparkMaxHWState::getReverseLimitSwitch(void) const
{
	return reverse_limit_switch_;
}

void SparkMaxHWState::setCurrentLimit(unsigned int current_limit)
{
	current_limit_ = current_limit;
}
unsigned int SparkMaxHWState::getCurrentLimit(void) const
{
	return current_limit_;
}

void SparkMaxHWState::setCurrentLimitFree(unsigned int current_limit_free)
{
	current_limit_free_ = current_limit_free;
}
unsigned int SparkMaxHWState::getCurrentLimitFree(void) const
{
	return current_limit_free_;
}

void SparkMaxHWState::setCurrentLimitStall(unsigned int current_limit_stall)
{
	current_limit_stall_ = current_limit_stall;
}
unsigned int SparkMaxHWState::getCurrentLimitStall(void) const
{
	return current_limit_stall_;
}

void SparkMaxHWState::setCurrentLimitRPM(unsigned int current_limit_rpm)
{
	current_limit_rpm_ = current_limit_rpm;
}
unsigned int SparkMaxHWState::getCurrentLimitRPM(void) const
{
	return current_limit_rpm_;
}

void SparkMaxHWState::setSecondaryCurrentLimit(double secondary_current_limit)
{
	secondary_current_limit_ = secondary_current_limit;
}
double SparkMaxHWState::getSecondaryCurrentLimit(void) const
{
	return secondary_current_limit_;
}

void SparkMaxHWState::setSecondaryCurrentLimitCycles(unsigned int secondary_current_limit_cycles)
{
	secondary_current_limit_cycles_ = secondary_current_limit_cycles;
}
unsigned int SparkMaxHWState::getSecondaryCurrentLimitCycles(void) const
{
	return secondary_current_limit_cycles_;
}

void SparkMaxHWState::setIdleMode(IdleMode idle_mode)
{
	idle_mode_ = idle_mode;
}
IdleMode SparkMaxHWState::getIdleMode(void) const
{
	return idle_mode_;
}

void SparkMaxHWState::setVoltageCompensationEnable(bool enable)
{
	voltage_compensation_enable_ = enable;
}
bool SparkMaxHWState::getVoltageCompensationEnable(void) const
{
	return voltage_compensation_enable_;
}

void SparkMaxHWState::setVoltageCompensationNominalVoltage(double nominal_voltage)
{
	voltage_compensation_nominal_voltage_ = nominal_voltage;
}
bool SparkMaxHWState::getVoltageCompensationNominalVoltage(void) const
{
	return voltage_compensation_nominal_voltage_;
}

void SparkMaxHWState::setOpenLoopRampRate(double open_loop_ramp_rate)
{
	open_loop_ramp_rate_ = open_loop_ramp_rate;
}
double SparkMaxHWState::getOpenLoopRampRate(void) const
{
	return open_loop_ramp_rate_;
}

void SparkMaxHWState::setClosedLoopRampRate(double closed_loop_ramp_rate)
{
	closed_loop_ramp_rate_ = closed_loop_ramp_rate;
}
double SparkMaxHWState::getClosedLoopRampRate(void) const
{
	return closed_loop_ramp_rate_;
}

void SparkMaxHWState::setForwardSoftlimitEnable(bool enable)
{
	forward_softlimit_enable_ = enable;
}
bool SparkMaxHWState::getForwardSoftlimitEnable(void) const
{
	return forward_softlimit_enable_;
}
void SparkMaxHWState::setForwardSoftlimit(double limit)
{
	reverse_softlimit_ = limit;
}
double SparkMaxHWState::getForwardSoftlimit(void) const
{
	return reverse_softlimit_;
}

void SparkMaxHWState::setReverseSoftlimitEnable(bool enable)
{
	forward_softlimit_enable_ = enable;
}
bool SparkMaxHWState::getReverseSoftlimitEnable(void) const
{
	return forward_softlimit_enable_;
}
void SparkMaxHWState::setReverseSoftlimit(double limit)
{
	reverse_softlimit_ = limit;
}
double SparkMaxHWState::getReverseSoftlimit(void) const
{
	return reverse_softlimit_;
}

void SparkMaxHWState::setFollowerType(ExternalFollower follower_type)
{
	follower_type_ = follower_type;
}
ExternalFollower SparkMaxHWState::getFollowerType(void) const
{
	return follower_type_;
}

void SparkMaxHWState::setFollowerID(int follower_id)
{
	follower_id_ = follower_id;
}
int SparkMaxHWState::getFollowerID(void) const
{
	return follower_id_;
}

void SparkMaxHWState::setFollowerInvert(bool follower_invert)
{
	follower_invert_ = follower_invert;
}
bool SparkMaxHWState::getFollowerInvert(void) const
{
	return follower_invert_;
}

void SparkMaxHWState::setFaults(uint16_t faults)
{
	faults_ = faults;
}
uint16_t SparkMaxHWState::getFaults(void) const
{
	return faults_;
}

void SparkMaxHWState::setStickyFaults(uint16_t sticky_faults)
{
	sticky_faults_ = sticky_faults;
}
uint16_t SparkMaxHWState::getStickyFaults(void) const
{
	return sticky_faults_;
}

void SparkMaxHWState::setBusVoltage(double bus_voltage)
{
	bus_voltage_ = bus_voltage;
}
double SparkMaxHWState::getBusVoltage(void) const
{
	return bus_voltage_;
}

void SparkMaxHWState::setAppliedOutput(double applied_output)
{
	applied_output_ = applied_output;
}
double SparkMaxHWState::getAppliedOutput(void) const
{
	return applied_output_;
}

void SparkMaxHWState::setOutputCurrent(double output_current)
{
	output_current_ = output_current;
}
double SparkMaxHWState::getOutputCurrent(void) const
{
	return output_current_;
}

void SparkMaxHWState::setMotorTemperature(double motor_temperature)
{
	motor_temperature_ = motor_temperature;
}
double SparkMaxHWState::getMotorTemperature(void) const
{
	return motor_temperature_;
}

unsigned int SparkMaxHWState::getEncoderTicksPerRotation(void) const
{
	return encoder_ticks_per_rotation_;
}
void SparkMaxHWState::setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation)
{
	encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
}

void SparkMaxHWState::setEncoderType(SensorType encoder_type)
{
	encoder_type_ = encoder_type;
}
SensorType SparkMaxHWState::getEncoderType(void) const
{
	return encoder_type_;
}


} // namespace hardware_interface

