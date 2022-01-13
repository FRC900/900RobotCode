#include "spark_max_interface/spark_max_command_interface.h"

namespace hardware_interface
{

SparkMaxHWCommand::SparkMaxHWCommand()
{
	p_gain_.fill(0);
	i_gain_.fill(0);
	d_gain_.fill(0);
	f_gain_.fill(0);
	i_zone_.fill(0);
	d_filter_.fill(0);
	pidf_constants_changed_.fill(false);
	pidf_output_min_.fill(-1);
	pidf_output_max_.fill(1);
	pidf_output_range_changed_.fill(false);
	pidf_reference_value_.fill(0);
	pidf_reference_ctrl_.fill(kDutyCycle);
	pidf_arb_feed_forward_.fill(0);
	pidf_arb_feed_forward_units_.fill(ArbFFUnits::kVoltage);
	pidf_reference_changed_.fill(false);
}

void SparkMaxHWCommand::setSetPoint(double set_point)
{
	if (set_point_ != set_point)
	{
		set_point_ = set_point;
		set_point_changed_ = true;
	}
}
double SparkMaxHWCommand::getSetPoint(void) const
{
	return set_point_;
}
bool SparkMaxHWCommand::changedSetPoint(double &set_point)
{
	set_point = set_point_;
	if (set_point_changed_)
	{
		set_point_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetSetPoint(void)
{
	set_point_changed_ = true;
}

void SparkMaxHWCommand::setInverted(bool inverted)
{
	if (inverted_ != inverted)
	{
		inverted_ = inverted;
		inverted_changed_ = true;
	}
}
bool SparkMaxHWCommand::getInverted(void) const
{
	return inverted_;
}
bool SparkMaxHWCommand::changedInverted(bool &inverted)
{
	inverted = inverted_;
	if (inverted_changed_)
	{
		inverted_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetInverted(void)
{
	inverted_changed_ = true;
}

void SparkMaxHWCommand::setPGain(size_t slot, double p_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setPGain() : invalid slot " << slot);
		return;
	}
	if (p_gain != p_gain_[slot])
	{
		p_gain_[slot] = p_gain;
		pidf_constants_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getPGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getPGain() : invalid slot " << slot);
		return -1;
	}
	return p_gain_[slot];
}

void SparkMaxHWCommand::setIGain(size_t slot, double i_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setIGain() : invalid slot " << slot);
		return;
	}
	if (i_gain != i_gain_[slot])
	{
		i_gain_[slot] = i_gain;
		pidf_constants_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getIGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getIGain() : invalid slot " << slot);
		return -1;
	}
	return i_gain_[slot];
}
void SparkMaxHWCommand::setDGain(size_t slot, double d_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setDGain() : invalid slot " << slot);
		return;
	}
	if (d_gain != d_gain_[slot])
	{
		d_gain_[slot] = d_gain;
		pidf_constants_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getDGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getDGain() : invalid slot " << slot);
		return -1;
	}
	return d_gain_[slot];
}

void SparkMaxHWCommand::setFGain(size_t slot, double f_gain)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setFGain() : invalid slot " << slot);
		return;
	}
	if (f_gain != f_gain_[slot])
	{
		f_gain_[slot] = f_gain;
		pidf_constants_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getFGain(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getFGain() : invalid slot " << slot);
		return -1;
	}
	return f_gain_[slot];
}

void SparkMaxHWCommand::setIZone(size_t slot, double i_zone)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setIZone() : invalid slot " << slot);
		return;
	}
	if (i_zone != i_zone_[slot])
	{
		i_zone_[slot] = i_zone;
		pidf_constants_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getIZone(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getIZone() : invalid slot " << slot);
		return -1;
	}
	return i_zone_[slot];
}

void SparkMaxHWCommand::setDFilter(size_t slot, double d_filter)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setDFilter() : invalid slot " << slot);
		return;
	}
	if (d_filter != d_filter_[slot])
	{
		d_filter_[slot] = d_filter;
		pidf_constants_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getDFilter(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getDFilter() : invalid slot " << slot);
		return -1;
	}
	return d_filter_[slot];
}

bool SparkMaxHWCommand::changedPIDFConstants(size_t slot,
		double &p_gain, double &i_gain,
		double &d_gain, double &f_gain,
		double &i_zone, double &d_filter)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDConstants() : invalid slot " << slot);
		return false;
	}
	p_gain = p_gain_[slot];
	i_gain = i_gain_[slot];
	d_gain = d_gain_[slot];
	f_gain = f_gain_[slot];
	i_zone = i_zone_[slot];
	d_filter = d_filter_[slot];
	if (pidf_constants_changed_[slot])
	{
		pidf_constants_changed_[slot] = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetPIDFConstants(size_t slot)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::resetPIDFConstants() : invalid slot " << slot);
		return;
	}
	pidf_constants_changed_[slot] = true;
}

void SparkMaxHWCommand::setPIDFOutputMin(size_t slot, double pidf_output_min)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFOutputMin() : invalid slot " << slot);
		return;
	}
	if (pidf_output_min != pidf_output_min_[slot])
	{
		pidf_output_min_[slot] = pidf_output_min;
		pidf_output_range_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getPIDFOutputMin(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFOutputMin() : invalid slot " << slot);
		return std::numeric_limits<double>::max();
	}
	return pidf_output_min_[slot];
}

void SparkMaxHWCommand::setPIDFOutputMax(size_t slot, double pidf_output_max)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFOutputMax() : invalid slot " << slot);
		return;
	}
	if (pidf_output_max != pidf_output_max_[slot])
	{
		pidf_output_max_[slot] = pidf_output_max;
		pidf_output_range_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getPIDFOutputMax(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFOutputMax() : invalid slot " << slot);
		return -std::numeric_limits<double>::max();
	}
	return pidf_output_max_[slot];
}
bool SparkMaxHWCommand::changedPIDOutputRange(size_t slot,
		double &output_min,
		double &output_max)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::changedPIDOutputRange() : invalid slot " << slot);
		return false;
	}
	output_min = pidf_output_min_[slot];
	output_max = pidf_output_max_[slot];
	if (pidf_output_range_changed_[slot])
	{
		pidf_output_range_changed_[slot] = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetPIDOutputRange(size_t slot)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::resetPIDOutputRange() : invalid slot " << slot);
		return;
	}
	pidf_output_range_changed_[slot] = true;
}

void SparkMaxHWCommand::setPIDFReferenceValue(size_t slot, double pidf_reference_value)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceValue() : invalid slot " << slot);
		return;
	}
	if (pidf_reference_value != pidf_reference_value_[slot])
	{
		pidf_reference_value_[slot] = pidf_reference_value;
		pidf_reference_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getPIDFReferenceValue(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFReferenceValue() : invalid slot " << slot);
		return -1;
	}
	return pidf_reference_value_[slot];
}

void SparkMaxHWCommand::setPIDFReferenceCtrl(size_t slot, ControlType pidf_reference_ctrl)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceCtrl() : invalid slot " << slot);
		return;
	}
	if (pidf_reference_ctrl != pidf_reference_ctrl_[slot])
	{
		pidf_reference_ctrl_[slot] = pidf_reference_ctrl;
		pidf_reference_changed_[slot] = true;
	}
}
ControlType SparkMaxHWCommand::getPIDFReferenceCtrl(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWState::getPIDFReferenceCtrl() : invalid slot " << slot);
		return kDutyCycle;
	}
	return pidf_reference_ctrl_[slot];
}
void SparkMaxHWCommand::setPIDFArbFeedForward(size_t slot, double pidf_arb_feed_forward)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFArbFeedForward() : invalid slot " << slot);
		return;
	}
	if (pidf_arb_feed_forward != pidf_arb_feed_forward_[slot])
	{
		pidf_arb_feed_forward_[slot] = pidf_arb_feed_forward;
		pidf_reference_changed_[slot] = true;
	}
}
double SparkMaxHWCommand::getPIDFArbFeedForward(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFArbFeedForward() : invalid slot " << slot);
		return -1;
	}
	return pidf_arb_feed_forward_[slot];
}
void SparkMaxHWCommand::setPIDFArbFeedForwardUnits(size_t slot, ArbFFUnits pidf_arb_feed_forward)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFArbFeedForwardUnits() : invalid slot " << slot);
		return;
	}
	if (pidf_arb_feed_forward != pidf_arb_feed_forward_units_[slot])
	{
		pidf_arb_feed_forward_units_[slot] = pidf_arb_feed_forward;
		pidf_reference_changed_[slot] = true;
	}
}
ArbFFUnits SparkMaxHWCommand::getPIDFArbFeedForwardUnits(size_t slot) const
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFArbFeedForwardUnits() : invalid slot " << slot);
		return ArbFFUnits::kVoltage; // probably should throw something here
	}
	return pidf_arb_feed_forward_units_[slot];
}
bool SparkMaxHWCommand::changedPIDFReference(size_t slot,
		double &pidf_reference_value,
		ControlType &pidf_reference_ctrl,
		double &pidf_arb_feed_forward,
		ArbFFUnits &pidf_arb_feed_forward_units)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::changedPIDFReference() : invalid slot " << slot);
		return false;
	}
	pidf_reference_value = pidf_reference_value_[slot];
	pidf_reference_ctrl = pidf_reference_ctrl_[slot];
	pidf_arb_feed_forward = pidf_arb_feed_forward_[slot];
	pidf_arb_feed_forward_units = pidf_arb_feed_forward_units_[slot];
	if (pidf_reference_changed_[slot])
	{
		pidf_reference_changed_[slot] = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetPIDReference(size_t slot)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::resetPIDFReference() : invalid slot " << slot);
		return;
	}
	pidf_reference_changed_[slot] = true;
}

void SparkMaxHWCommand::setPIDFReferenceSlot(size_t slot)
{
	if (slot >= SPARK_MAX_PID_SLOTS)
	{
		ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFReferenceSlot() : invalid slot " << slot);
		return;
	}
	if (slot != pidf_reference_slot_)
	{
		pidf_reference_slot_ = slot;
		pidf_reference_slot_changed_ = true;
	}
}
int SparkMaxHWCommand::getPIDFReferenceSlot(void) const
{
	return pidf_reference_slot_;
}

bool SparkMaxHWCommand::changedPIDFReferenceSlot(size_t &pidf_reference_slot)
{
	pidf_reference_slot = pidf_reference_slot_;
	if (pidf_reference_slot_changed_)
	{
		pidf_reference_slot_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetPIDFReferenceSlot(void)
{
	pidf_reference_slot_changed_ = true;
}

void SparkMaxHWCommand::setForwardLimitSwitchPolarity(LimitSwitchPolarity forward_limit_switch_polarity)
{
	if (forward_limit_switch_polarity_ != forward_limit_switch_polarity)
	{
		forward_limit_switch_polarity_ = forward_limit_switch_polarity;
		forward_limit_switch_changed_ = true;
	}
}
LimitSwitchPolarity SparkMaxHWCommand::getForwardLimitSwitchPolarity(void) const
{
	return forward_limit_switch_polarity_;
}

void SparkMaxHWCommand::setForwardLimitSwitchEnabled(bool forward_limit_switch_enabled)
{
	if (forward_limit_switch_enabled_ != forward_limit_switch_enabled)
	{
		forward_limit_switch_enabled_ = forward_limit_switch_enabled;
		forward_limit_switch_changed_ = true;
	}
}
bool SparkMaxHWCommand::getForwardLimitSwitchEnabled(void) const
{
	return forward_limit_switch_enabled_;
}

bool SparkMaxHWCommand::changedForwardLimitSwitch(
		LimitSwitchPolarity &forward_limit_switch_polarity,
		bool &forward_limit_switch_enabled)
{
	forward_limit_switch_polarity = forward_limit_switch_polarity_;
	forward_limit_switch_enabled = forward_limit_switch_enabled_;
	if (forward_limit_switch_changed_)
	{
		forward_limit_switch_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetForwardLimitSwitch(void)
{
	forward_limit_switch_changed_ = true;
}

void SparkMaxHWCommand::setReverseLimitSwitchPolarity(LimitSwitchPolarity reverse_limit_switch_polarity)
{
	if (reverse_limit_switch_polarity_ != reverse_limit_switch_polarity)
	{
		reverse_limit_switch_polarity_ = reverse_limit_switch_polarity;
		reverse_limit_switch_changed_ = true;
	}
}
LimitSwitchPolarity SparkMaxHWCommand::getReverseLimitSwitchPolarity(void) const
{
	return reverse_limit_switch_polarity_;
}

void SparkMaxHWCommand::setReverseLimitSwitchEnabled(bool reverse_limit_switch_enabled)
{
	if (reverse_limit_switch_enabled_ != reverse_limit_switch_enabled)
	{
		reverse_limit_switch_enabled_ = reverse_limit_switch_enabled;
		reverse_limit_switch_changed_ = true;
	}
}
bool SparkMaxHWCommand::getReverseLimitSwitchEnabled(void) const
{
	return reverse_limit_switch_enabled_;
}

bool SparkMaxHWCommand::changedReverseLimitSwitch(
		LimitSwitchPolarity &reverse_limit_switch_polarity,
		bool &reverse_limit_switch_enabled)
{
	reverse_limit_switch_polarity = reverse_limit_switch_polarity_;
	reverse_limit_switch_enabled = reverse_limit_switch_enabled_;
	if (reverse_limit_switch_changed_)
	{
		reverse_limit_switch_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetReverseLimitSwitch(void)
{
	reverse_limit_switch_changed_ = true;
}

void SparkMaxHWCommand::setCurrentLimit(unsigned int current_limit)
{
	if (current_limit_ != current_limit)
	{
		current_limit_ = current_limit;
		current_limit_one_changed_ = true;
	}
}
unsigned int SparkMaxHWCommand::getCurrentLimit(void) const
{
	return current_limit_;
}
bool SparkMaxHWCommand::changedCurrentLimitOne(unsigned int &current_limit)
{
	current_limit = current_limit_;
	if (current_limit_one_changed_)
	{
		current_limit_one_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetCurrentLimitOne(void)
{
	current_limit_one_changed_ = true;
}

void SparkMaxHWCommand::setCurrentLimitStall(unsigned int current_limit_stall)
{
	if (current_limit_stall_ != current_limit_stall)
	{
		current_limit_stall_ = current_limit_stall;
		current_limit_changed_ = true;
	}
}
unsigned int SparkMaxHWCommand::getCurrentLimitStall(void) const
{
	return current_limit_stall_;
}

void SparkMaxHWCommand::setCurrentLimitFree(unsigned int current_limit_free)
{
	if (current_limit_free_ != current_limit_free)
	{
		current_limit_free_ = current_limit_free;
		current_limit_changed_ = true;
	}
}
unsigned int SparkMaxHWCommand::getCurrentLimitFree(void) const
{
	return current_limit_free_;
}

void SparkMaxHWCommand::setCurrentLimitRPM(unsigned int current_limit_rpm)
{
	if (current_limit_rpm_ != current_limit_rpm)
	{
		current_limit_rpm_ = current_limit_rpm;
		current_limit_changed_ = true;
	}
}
unsigned int SparkMaxHWCommand::getCurrentLimitRPM(void) const
{
	return current_limit_rpm_;
}

bool SparkMaxHWCommand::changedCurrentLimit(
		unsigned int &current_limit_stall,
		unsigned int &current_limit_free,
		unsigned int &current_limit_rpm)
{
	current_limit_free = current_limit_free_;
	current_limit_stall = current_limit_stall_;
	current_limit_rpm = current_limit_rpm_;
	if (current_limit_changed_)
	{
		current_limit_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetCurrentLimit(void)
{
	current_limit_changed_ = true;
}

void SparkMaxHWCommand::setSecondaryCurrentLimit(double secondary_current_limit)
{
	if (secondary_current_limit_ != secondary_current_limit)
	{
		secondary_current_limit_ = secondary_current_limit;
		secondary_current_limit_changed_ = true;
	}
}
double SparkMaxHWCommand::getSecondaryCurrentLimit(void) const
{
	return secondary_current_limit_;
}

void SparkMaxHWCommand::setSecondaryCurrentLimitCycles(unsigned int secondary_current_limit_cycles)
{
	if (secondary_current_limit_cycles_ != secondary_current_limit_cycles)
	{
		secondary_current_limit_cycles_ = secondary_current_limit_cycles;
		secondary_current_limit_changed_ = true;
	}
}
unsigned int SparkMaxHWCommand::getSecondaryCurrentLimitCycles(void) const
{
	return secondary_current_limit_cycles_;
}

bool SparkMaxHWCommand::changedSecondaryCurrentLimits(double &secondary_current_limit,
		unsigned int &secondary_current_limit_cycles)
{
	secondary_current_limit = secondary_current_limit_;
	secondary_current_limit_cycles = secondary_current_limit_cycles_;
	if (secondary_current_limit_changed_)
	{
		secondary_current_limit_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetSecondaryCurrentLimits(void)
{
	secondary_current_limit_changed_ = true;
}

void SparkMaxHWCommand::setIdleMode(IdleMode idle_mode)
{
	if (idle_mode_ != idle_mode)
	{
		idle_mode_ = idle_mode;
		idle_mode_changed_ = true;
	}
}
IdleMode SparkMaxHWCommand::getIdleMode(void) const
{
	return idle_mode_;
}
bool SparkMaxHWCommand::changedIdleMode(IdleMode &idle_mode)
{
	idle_mode = idle_mode_;
	if (idle_mode_changed_)
	{
		idle_mode_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetIdleMode(void)
{
	idle_mode_changed_ = true;
}

void SparkMaxHWCommand::setVoltageCompensationEnable(bool enable)
{
	if (voltage_compensation_enable_ != enable)
	{
		voltage_compensation_enable_ = enable;
		voltage_compensation_changed_ = true;
	}
}
bool SparkMaxHWCommand::getVoltageCompensationEnable(void)
{
	return voltage_compensation_enable_;
}

void SparkMaxHWCommand::setVoltageCompensationNominalVoltage(double nominal_voltage)
{
	if (voltage_compensation_nominal_voltage_ != nominal_voltage)
	{
		voltage_compensation_nominal_voltage_ = nominal_voltage;
		voltage_compensation_changed_ = true;
	}
}
bool SparkMaxHWCommand::getVoltageCompensationNominalVoltage(void)
{
	return voltage_compensation_nominal_voltage_;
}
bool SparkMaxHWCommand::changedVoltageCompensation(bool &enable,
		double &nominal_voltage)
{
	enable = voltage_compensation_enable_;
	nominal_voltage = voltage_compensation_nominal_voltage_;
	if (voltage_compensation_changed_)
	{
		voltage_compensation_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetVoltageCompensation(void)
{
	voltage_compensation_changed_ = true;
}

void SparkMaxHWCommand::setOpenLoopRampRate(double open_loop_ramp_rate)
{
	if (open_loop_ramp_rate_ != open_loop_ramp_rate)
	{
		open_loop_ramp_rate_ = open_loop_ramp_rate;
		open_loop_ramp_rate_changed_ = true;
	}
}
double SparkMaxHWCommand::getOpenLoopRampRate(void) const
{
	return open_loop_ramp_rate_;
}
bool SparkMaxHWCommand::changedOpenLoopRampRate(double &open_loop_ramp_rate)
{
	open_loop_ramp_rate = open_loop_ramp_rate_;
	if (open_loop_ramp_rate_changed_)
	{
		open_loop_ramp_rate_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetOpenLoopRampRate(void)
{
	open_loop_ramp_rate_changed_ = true;
}

void SparkMaxHWCommand::setClosedLoopRampRate(double closed_loop_ramp_rate)
{
	if (closed_loop_ramp_rate_ != closed_loop_ramp_rate)
	{
		closed_loop_ramp_rate_ = closed_loop_ramp_rate;
		closed_loop_ramp_rate_changed_ = true;
	}
}
double SparkMaxHWCommand::getClosedLoopRampRate(void) const
{
	return closed_loop_ramp_rate_;
}
bool SparkMaxHWCommand::changedClosedLoopRampRate(double &closed_loop_ramp_rate)
{
	closed_loop_ramp_rate = closed_loop_ramp_rate_;
	if (closed_loop_ramp_rate_changed_)
	{
		closed_loop_ramp_rate_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetClosedLoopRampRate(void)
{
	closed_loop_ramp_rate_changed_ = true;
}

void SparkMaxHWCommand::setFollowerType(ExternalFollower follower_type)
{
	if (follower_type_ != follower_type)
	{
		follower_type_ = follower_type;
		follower_changed_ = true;
	}
}
ExternalFollower SparkMaxHWCommand::getFollowerType(void) const
{
	return follower_type_;
}

void SparkMaxHWCommand::setFollowerID(double follower_id)
{
	if (follower_id_ != follower_id)
	{
		follower_id_ = follower_id;
		follower_changed_ = true;
	}
}
double SparkMaxHWCommand::getFollowerID(void) const
{
	return follower_id_;
}

void SparkMaxHWCommand::setFollowerInvert(double follower_invert)
{
	if (follower_invert_ != follower_invert)
	{
		follower_invert_ = follower_invert;
		follower_changed_ = true;
	}
}
double SparkMaxHWCommand::getFollowerInvert(void) const
{
	return follower_invert_;
}

bool SparkMaxHWCommand::changedFollower(ExternalFollower &follower_type,
		int &follower_id,
		bool &follower_invert)
{
	follower_type = follower_type_;
	follower_id = follower_id_;
	follower_invert = follower_invert_;
	if (follower_changed_)
	{
		follower_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetFollower(void)
{
	follower_changed_ = true;
}

void SparkMaxHWCommand::setForwardSoftlimitEnable(bool enable)
{
	if (forward_softlimit_enable_ != enable)
	{
		forward_softlimit_enable_ = enable;
		forward_softlimit_changed_ = true;
	}
}
bool SparkMaxHWCommand::getForwardSoftlimitEnable(void) const
{
	return forward_softlimit_enable_;
}
void SparkMaxHWCommand::setForwardSoftlimit(double limit)
{
	if (forward_softlimit_ != limit)
	{
		forward_softlimit_ = limit;
		forward_softlimit_changed_ = true;
	}
}
double SparkMaxHWCommand::getForwardSoftlimit(void) const
{
	return forward_softlimit_;
}
bool SparkMaxHWCommand::changedForwardSoftlimit(bool &enable, double &limit)
{
	enable = forward_softlimit_enable_;
	limit  = forward_softlimit_;
	if (forward_softlimit_changed_)
	{
		forward_softlimit_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetForwardSoftlimit(void)
{
	forward_softlimit_changed_ = true;
}

void SparkMaxHWCommand::setReverseSoftlimitEnable(bool enable)
{
	if (reverse_softlimit_enable_ != enable)
	{
		reverse_softlimit_enable_ = enable;
		reverse_softlimit_changed_ = true;
	}
}
bool SparkMaxHWCommand::getReverseSoftlimitEnable(void) const
{
	return reverse_softlimit_enable_;
}
void SparkMaxHWCommand::setReverseSoftlimit(double limit)
{
	if (reverse_softlimit_ != limit)
	{
		reverse_softlimit_ = limit;
		reverse_softlimit_changed_ = true;
	}
}
double SparkMaxHWCommand::getReverseSoftlimit(void) const
{
	return reverse_softlimit_;
}
bool SparkMaxHWCommand::changedReverseSoftlimit(bool &enable, double &limit)
{
	enable = reverse_softlimit_enable_;
	limit  = reverse_softlimit_;
	if (reverse_softlimit_changed_)
	{
		reverse_softlimit_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetReverseSoftlimit(void)
{
	reverse_softlimit_changed_ = true;
}
unsigned int SparkMaxHWCommand::getEncoderTicksPerRotation(void) const
{
	return encoder_ticks_per_rotation_;
}

void SparkMaxHWCommand::setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation)
{
	encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
}

void SparkMaxHWCommand::setEncoderType(SensorType encoder_type)
{
	if (encoder_type_ != encoder_type)
	{
		encoder_type_ = encoder_type;
		encoder_type_changed_ = true;
	}
}
SensorType SparkMaxHWCommand::getEncoderType(void) const
{
	return encoder_type_;
}

bool SparkMaxHWCommand::changedEncoderType(SensorType &encoder_type)
{
	encoder_type = encoder_type_;
	if (encoder_type_changed_)
	{
		encoder_type_changed_ = false;
		return true;
	}
	return false;
}
void SparkMaxHWCommand::resetEncoderType(void)
{
	encoder_type_changed_ = true;
}

} // namespace hardware_interface
