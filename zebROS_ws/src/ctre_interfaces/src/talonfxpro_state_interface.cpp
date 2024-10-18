#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "ctre_interfaces/talonfxpro_state_types.h"

namespace hardware_interface::talonfxpro
{

// Set up default values
// Set most of the changed_ vars to true
// to force a write of these values to the Talon
// That should put the talon in a known state
// rather than relying on them being setup to
// a certain state previously
TalonFXProHWState::TalonFXProHWState(const int can_id)
	: can_id_(can_id)
{
}

TalonFXProHWState::~TalonFXProHWState() = default;

int TalonFXProHWState::getCANID(void) const
{
	return can_id_;
}

void TalonFXProHWState::setkP(const double kP, const size_t index)
{
	if (index >= kP_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	kP_[index] = kP;
}
double TalonFXProHWState::getkP(const size_t index) const
{
	if (index >= kP_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kP_[index];
}

void TalonFXProHWState::setkI(const double kI, const size_t index)
{
	if (index >= kI_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	kI_[index] = kI;
}
double TalonFXProHWState::getkI(const size_t index) const
{
	if (index >= kI_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kI_[index];
}

void TalonFXProHWState::setkD(const double kD, const size_t index)
{
	if (index >= kD_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	kD_[index] = kD;
}
double TalonFXProHWState::getkD(const size_t index) const
{
	if (index >= kD_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kD_[index];
}

void TalonFXProHWState::setkS(const double kS, const size_t index)
{
	if (index >= kS_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	kS_[index] = kS;
}
double TalonFXProHWState::getkS(const size_t index) const
{
	if (index >= kS_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kS_[index];
}

void TalonFXProHWState::setkV(const double kV, const size_t index)
{
	if (index >= kV_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	kV_[index] = kV;
}
double TalonFXProHWState::getkV(const size_t index) const
{
	if (index >= kV_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kV_[index];
}

void TalonFXProHWState::setkA(const double kA, const size_t index)
{
	if (index >= kA_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	kA_[index] = kA;
}
double TalonFXProHWState::getkA(const size_t index) const
{
	if (index >= kA_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kA_[index];
}

void TalonFXProHWState::setkG(const double kG, const size_t index)
{
	if (index >= kG_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	kG_[index] = kG;
}
double TalonFXProHWState::getkG(const size_t index) const
{
	if (index >= kG_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kG_[index];
}

void TalonFXProHWState::setGravityType(const GravityType gravity_type, const size_t index)
{
	if (index >= gravity_type_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if ((gravity_type <= GravityType::First) ||
		(gravity_type >= GravityType::Last))
	{
		ROS_WARN_STREAM("Invalid gravity type (" << static_cast<int>(gravity_type) << ") passed to " << __PRETTY_FUNCTION__);
		return;
	}
	gravity_type_[index] = gravity_type;
}
GravityType TalonFXProHWState::getGravityType(const size_t index) const
{
	if (index >= gravity_type_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return GravityType::Last;
	}
	return gravity_type_[index];
}

void TalonFXProHWState::setStaticFeedforwardSign(const StaticFeedforwardSign static_feedforward_sign, const size_t index)
{
	if (index >= static_feedforward_sign_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if ((static_feedforward_sign <= StaticFeedforwardSign::First) ||
		(static_feedforward_sign >= StaticFeedforwardSign::Last))
	{
		ROS_WARN_STREAM("Invalid gravity type (" << static_cast<int>(static_feedforward_sign) << ") passed to " << __PRETTY_FUNCTION__);
		return;
	}
	static_feedforward_sign_[index] = static_feedforward_sign;
}
StaticFeedforwardSign TalonFXProHWState::getStaticFeedforwardSign(const size_t index) const
{
	if (index >= static_feedforward_sign_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return StaticFeedforwardSign::Last;
	}
	return static_feedforward_sign_[index];
}

void TalonFXProHWState::setInvert(const Inverted invert)
{
	if (invert == Inverted::Uninitialized)
		return; // Don't warn on this?
	else if ((invert < Inverted::Uninitialized) ||
			 (invert >= Inverted::Last))
	{
		ROS_WARN("Invalid invert passed to TalonFXProHWState::setInvert(const )");
		return;
	}
	invert_ = invert;
}
Inverted TalonFXProHWState::getInvert(void) const
{
	return invert_;
}

void TalonFXProHWState::setNeutralMode(const NeutralMode neutral_mode)
{
	if (neutral_mode == NeutralMode::Uninitialized)
		return; // Don't warn on this?
	else if ((neutral_mode < NeutralMode::Uninitialized) ||
			 (neutral_mode >= NeutralMode::Last))
	{
		ROS_WARN("Invalid neutral_mode passed to TalonFXProHWState::setNeutralMode(const )");
		return;
	}
	neutral_mode_ = neutral_mode;
}
NeutralMode TalonFXProHWState::getNeutralMode(void) const
{
	return neutral_mode_;
}

void TalonFXProHWState::setDutyCycleNeutralDeadband(const double duty_cycle_neutral_deadband)
{
	duty_cycle_neutral_deadband_ = duty_cycle_neutral_deadband;
}
double TalonFXProHWState::getDutyCycleNeutralDeadband(void) const
{
	return duty_cycle_neutral_deadband_;
}
void TalonFXProHWState::setPeakForwardDutyCycle(const double peak_forward_duty_cycle)
{
	peak_forward_duty_cycle_ = peak_forward_duty_cycle;
}
double TalonFXProHWState::getPeakForwardDutyCycle(void) const
{
	return peak_forward_duty_cycle_;
}
void TalonFXProHWState::setPeakReverseDutyCycle(const double peak_reverse_duty_cycle)
{
	peak_reverse_duty_cycle_ = peak_reverse_duty_cycle;
}
double TalonFXProHWState::getPeakReverseDutyCycle(void) const
{
	return peak_reverse_duty_cycle_;
}

void TalonFXProHWState::setControlTimesyncFreqHz(const double control_timesync_freq_hz)
{
	control_timesync_freq_hz_ = control_timesync_freq_hz;
}
double TalonFXProHWState::getControlTimesyncFreqHz(void) const
{
	return control_timesync_freq_hz_;
}

void TalonFXProHWState::setStatorCurrentLimit(const double stator_current_limit)
{
	stator_current_limit_ = stator_current_limit;
}
double TalonFXProHWState::getStatorCurrentLimit(void) const
{
	return stator_current_limit_;
}
void TalonFXProHWState::setStatorCurrentLimitEnable(const bool stator_current_limit_enable)
{
	stator_current_limit_enable_ = stator_current_limit_enable;
}
bool TalonFXProHWState::getStatorCurrentLimitEnable(void) const
{
	return stator_current_limit_enable_;
}

void TalonFXProHWState::setSupplyCurrentLimit(const double supply_current_limit)
{
	supply_current_limit_ = supply_current_limit;
}
double TalonFXProHWState::getSupplyCurrentLimit(void) const
{
	return supply_current_limit_;
}
void TalonFXProHWState::setSupplyCurrentLimitEnable(const bool supply_current_limit_enable)
{
	supply_current_limit_enable_ = supply_current_limit_enable;
}
bool TalonFXProHWState::getSupplyCurrentLimitEnable(void) const
{
	return supply_current_limit_enable_;
}
void TalonFXProHWState::setSupplyCurrentLowerLimit(const double supply_current_lower_limit)
{
	supply_current_lower_limit_ = supply_current_lower_limit;
}
double TalonFXProHWState::getSupplyCurrentLowerLimit(void) const
{
	return supply_current_lower_limit_;
}
void TalonFXProHWState::setSupplyCurrentLowerTime(const double supply_current_lower_time)
{
	supply_current_lower_time_ = supply_current_lower_time;
}
double TalonFXProHWState::getSupplyCurrentLowerTime(void) const
{
	return supply_current_lower_time_;
}

void TalonFXProHWState::setSupplyVoltageTimeConstant(const double supply_voltage_time_constant) 
{
	supply_voltage_time_constant_ = supply_voltage_time_constant;
}
double TalonFXProHWState::getSupplyVoltageTimeConstant(void) const
{
	return supply_voltage_time_constant_;
}

void TalonFXProHWState::setPeakForwardVoltage(const double peak_forward_voltage)
{
	peak_forward_voltage_ = peak_forward_voltage;
}
double TalonFXProHWState::getPeakForwardVoltage(void) const
{
	return peak_forward_voltage_;
}

void TalonFXProHWState::setPeakReverseVoltage(const double peak_reverse_voltage)
{
	peak_reverse_voltage_ = peak_reverse_voltage;
}
double TalonFXProHWState::getPeakReverseVoltage(void) const
{
	return peak_reverse_voltage_;
}

void TalonFXProHWState::setPeakForwardTorqueCurrent(const double peak_forward_torque_current)
{
	peak_forward_torque_current_ = peak_forward_torque_current;
}
double TalonFXProHWState::getPeakForwardTorqueCurrent(void) const
{
	return peak_forward_torque_current_;
}
void TalonFXProHWState::setPeakReverseTorqueCurrent(const double peak_reverse_torque_current)
{
	peak_reverse_torque_current_ = peak_reverse_torque_current;
}
double TalonFXProHWState::getPeakReverseTorqueCurrent(void) const
{
	return peak_reverse_torque_current_;
}
void TalonFXProHWState::setTorqueNeutralDeadband(const double torque_neutral_deadband)
{
	torque_neutral_deadband_ = torque_neutral_deadband;
}
double TalonFXProHWState::getTorqueNeutralDeadband(void) const
{
	return torque_neutral_deadband_;
}

void TalonFXProHWState::setFeedbackRotorOffset(const double feedback_rotor_offset)
{
	feedback_rotor_offset_ = feedback_rotor_offset;
}
double TalonFXProHWState::getFeedbackRotorOffset(void) const
{
	return feedback_rotor_offset_;
}

void TalonFXProHWState::setSensorToMechanismRatio(const double sensor_to_mechanism_ratio)
{
	sensor_to_mechanism_ratio_ = sensor_to_mechanism_ratio;
}
double TalonFXProHWState::getSensorToMechanismRatio(void) const
{
	return sensor_to_mechanism_ratio_;
}

void TalonFXProHWState::setRotorToSensorRatio(const double rotor_to_sensor_ratio)
{
	rotor_to_sensor_ratio_ = rotor_to_sensor_ratio;
}
double TalonFXProHWState::getRotorToSensorRatio(void) const
{
	return rotor_to_sensor_ratio_;
}

void TalonFXProHWState::setFeedbackSensorSource(const FeedbackSensorSource feedback_sensor_source)
{
	feedback_sensor_source_ = feedback_sensor_source;
}
FeedbackSensorSource TalonFXProHWState::getFeedbackSensorSource(void) const
{
	return feedback_sensor_source_;
}

void TalonFXProHWState::setFeedbackRemoteSensorID(const int feedback_remote_sensor_id)
{
	feedback_remote_sensor_id_ = feedback_remote_sensor_id;
}
int TalonFXProHWState::getFeedbackRemoteSensorID(void) const
{
	return feedback_remote_sensor_id_;
}

void TalonFXProHWState::setVelocityFilterTimeConstant(const double velocity_filter_time_constant)
{
	velocity_filter_time_constant_ = velocity_filter_time_constant;
}
double TalonFXProHWState::getVelocityFilterTimeConstant(void) const
{
	return velocity_filter_time_constant_;
}

void TalonFXProHWState::setDifferentialSensorSource(const DifferentialSensorSource differential_sensor_source)
{
	if ((differential_sensor_source <= hardware_interface::talonfxpro::DifferentialSensorSource::First) ||
		(differential_sensor_source >= hardware_interface::talonfxpro::DifferentialSensorSource::Last))
	{
		ROS_WARN_STREAM("Invalid differential_sensor_source " << static_cast<int>(differential_sensor_source) << " passed to " << __PRETTY_FUNCTION__);
		return;
	}
	differential_sensor_source_ = differential_sensor_source;
}
DifferentialSensorSource TalonFXProHWState::getDifferentialSensorSource(void) const
{
	return differential_sensor_source_;
}

void TalonFXProHWState::setDifferentialTalonFXSensorID(const int differential_talonfx_sensor_id)
{
	differential_talonfx_sensor_id_ = differential_talonfx_sensor_id;
}
int TalonFXProHWState::getDifferentialTalonFXSensorID(void) const
{
	return differential_talonfx_sensor_id_;
}

void TalonFXProHWState::setDifferentialRemoteSensorID(const int differential_remote_sensor_id)
{
	differential_remote_sensor_id_ = differential_remote_sensor_id;
}
int TalonFXProHWState::getDifferentialRemoteSensorID(void) const
{
	return differential_remote_sensor_id_;
}

void TalonFXProHWState::setPeakDifferentialDutyCycle(const double peak_differential_duty_cycle)
{
	peak_differential_duty_cycle_ = peak_differential_duty_cycle;
}
double TalonFXProHWState::getPeakDifferentialDutyCycle(void) const
{
	return peak_differential_duty_cycle_;
}

void TalonFXProHWState::setPeakDifferentialVoltage(const double peak_differential_voltage)
{
	peak_differential_voltage_ = peak_differential_voltage;
}
double TalonFXProHWState::getPeakDifferentialVoltage(void) const
{
	return peak_differential_voltage_;
}

void TalonFXProHWState::setPeakDifferentialTorqueCurrent(const double peak_differential_torque_current)
{
	peak_differential_torque_current_ = peak_differential_torque_current;
}
double TalonFXProHWState::getPeakDifferentialTorqueCurrent(void) const
{
	return peak_differential_torque_current_;
}

void TalonFXProHWState::setDutyCycleOpenLoopRampPeriod(const double duty_cycle_open_loop_ramp_period)
{
	duty_cycle_open_loop_ramp_period_ = duty_cycle_open_loop_ramp_period;
}
double TalonFXProHWState::getDutyCycleOpenLoopRampPeriod(void) const
{
	return duty_cycle_open_loop_ramp_period_;
}

void TalonFXProHWState::setVoltageOpenLoopRampPeriod(const double voltage_open_loop_ramp_period)
{
	voltage_open_loop_ramp_period_ = voltage_open_loop_ramp_period;
}
double TalonFXProHWState::getVoltageOpenLoopRampPeriod(void) const
{
	return voltage_open_loop_ramp_period_;
}

void TalonFXProHWState::setTorqueOpenLoopRampPeriod(const double torque_open_loop_ramp_period) 
{
	torque_open_loop_ramp_period_ = torque_open_loop_ramp_period;
}
double TalonFXProHWState::getTorqueOpenLoopRampPeriod(void) const
{
	return torque_open_loop_ramp_period_;
}

void TalonFXProHWState::setDutyCycleClosedLoopRampPeriod(const double duty_cycle_closed_loop_ramp_period)
{
	duty_cycle_closed_loop_ramp_period_ = duty_cycle_closed_loop_ramp_period;
}
double TalonFXProHWState::getDutyCycleClosedLoopRampPeriod(void) const
{
	return duty_cycle_closed_loop_ramp_period_;
}

void TalonFXProHWState::setVoltageClosedLoopRampPeriod(const double voltage_closed_loop_ramp_period)
{
	voltage_closed_loop_ramp_period_ = voltage_closed_loop_ramp_period;
}
double TalonFXProHWState::getVoltageClosedLoopRampPeriod(void) const
{
	return voltage_closed_loop_ramp_period_;
}

void TalonFXProHWState::setTorqueClosedLoopRampPeriod(const double torque_closed_loop_ramp_period) 
{
	torque_closed_loop_ramp_period_ = torque_closed_loop_ramp_period;
}
double TalonFXProHWState::getTorqueClosedLoopRampPeriod(void) const
{
	return torque_closed_loop_ramp_period_;
}

void TalonFXProHWState::setForwardLimitType(const LimitType forward_limit_type)
{
	forward_limit_type_ = forward_limit_type;
}
LimitType TalonFXProHWState::getForwardLimitType(void) const
{
	return forward_limit_type_;
}

void TalonFXProHWState::setForwardLimitAutosetPositionEnable(const bool forward_limit_autoset_position_enable)
{
	forward_limit_autoset_position_enable_ = forward_limit_autoset_position_enable;
}
bool TalonFXProHWState::getForwardLimitAutosetPositionEnable(void) const
{
	return forward_limit_autoset_position_enable_;
}

void TalonFXProHWState::setForwardLimitAutosetPositionValue(const double forward_limit_autoset_position_value)
{
	forward_limit_autoset_position_value_ = forward_limit_autoset_position_value;
}
double TalonFXProHWState::getForwardLimitAutosetPositionValue(void) const
{
	return forward_limit_autoset_position_value_;
}

void TalonFXProHWState::setForwardLimitEnable(const bool forward_limit_enable)
{
	forward_limit_enable_ = forward_limit_enable;
}
bool TalonFXProHWState::getForwardLimitEnable(void) const
{
	return forward_limit_enable_;
}

void TalonFXProHWState::setForwardLimitSource(const LimitSource forward_limit_source)
{
	forward_limit_source_ = forward_limit_source;
}
LimitSource TalonFXProHWState::getForwardLimitSource(void) const
{
	return forward_limit_source_;
}

void TalonFXProHWState::setForwardLimitRemoteSensorID(const int forward_limit_remote_sensor_id)
{
	forward_limit_remote_sensor_id_ = forward_limit_remote_sensor_id;
}
int TalonFXProHWState::getForwardLimitRemoteSensorID(void) const
{
	return forward_limit_remote_sensor_id_;
}

void TalonFXProHWState::setReverseLimitType(const LimitType reverse_limit_type)
{
	reverse_limit_type_ = reverse_limit_type;
}
LimitType TalonFXProHWState::getReverseLimitType(void) const
{
	return reverse_limit_type_;
}

void TalonFXProHWState::setReverseLimitAutosetPositionEnable(const bool reverse_limit_autoset_position_enable)
{
	reverse_limit_autoset_position_enable_ = reverse_limit_autoset_position_enable;
}
bool TalonFXProHWState::getReverseLimitAutosetPositionEnable(void) const
{
	return reverse_limit_autoset_position_enable_;
}

void TalonFXProHWState::setReverseLimitAutosetPositionValue(const double reverse_limit_autoset_position_value)
{
	reverse_limit_autoset_position_value_ = reverse_limit_autoset_position_value;
}
double TalonFXProHWState::getReverseLimitAutosetPositionValue(void) const
{
	return reverse_limit_autoset_position_value_;
}

void TalonFXProHWState::setReverseLimitEnable(const bool reverse_limit_enable)
{
	reverse_limit_enable_ = reverse_limit_enable;
}
bool TalonFXProHWState::getReverseLimitEnable(void) const
{
	return reverse_limit_enable_;
}

void TalonFXProHWState::setReverseLimitSource(const LimitSource reverse_limit_source)
{
	reverse_limit_source_ = reverse_limit_source;
}
LimitSource TalonFXProHWState::getReverseLimitSource(void) const
{
	return reverse_limit_source_;
}

void TalonFXProHWState::setReverseLimitRemoteSensorID(const int reverse_limit_remote_sensor_id)
{
	reverse_limit_remote_sensor_id_ = reverse_limit_remote_sensor_id;
}
int TalonFXProHWState::getReverseLimitRemoteSensorID(void) const
{
	return reverse_limit_remote_sensor_id_;
}

void TalonFXProHWState::setBeepOnBoot(const bool beep_on_boot)
{
	beep_on_boot_ = beep_on_boot;
}
bool TalonFXProHWState::getBeepOnBoot(void) const
{
	return beep_on_boot_;
}
void TalonFXProHWState::setBeepOnConfig(const bool beep_on_config)
{
	beep_on_config_ = beep_on_config;
}
bool TalonFXProHWState::getBeepOnConfig(void) const
{
	return beep_on_config_;
}
void TalonFXProHWState::setAllowMusicDurDisable(const bool allow_music_dur_disable)
{
	allow_music_dur_disable_ = allow_music_dur_disable;
}
bool TalonFXProHWState::getAllowMusicDurDisable(void) const
{
	return allow_music_dur_disable_;
}


void TalonFXProHWState::setForwardSoftLimitEnable(const bool enable)
{
	softlimit_forward_enable_ = enable;
}
bool TalonFXProHWState::getForwardSoftLimitEnable(void) const
{
	return softlimit_forward_enable_;
}
void TalonFXProHWState::setReverseSoftLimitEnable(const bool enable)
{
	softlimit_reverse_enable_ = enable;
}
bool TalonFXProHWState::getReverseSoftLimitEnable(void) const
{
	return softlimit_reverse_enable_;
}
void TalonFXProHWState::setForwardSoftLimitThreshold(const double threshold)
{
	softlimit_forward_threshold_ = threshold;
}
double TalonFXProHWState::getForwardSoftLimitThreshold(void) const
{
	return softlimit_forward_threshold_;
}

void TalonFXProHWState::setReverseSoftLimitThreshold(const double threshold)
{
	softlimit_reverse_threshold_ = threshold;
}
double TalonFXProHWState::getReverseSoftLimitThreshold(void) const
{
	return softlimit_reverse_threshold_;
}

void TalonFXProHWState::setMotionMagicCruiseVelocity(const double motion_magic_cruise_velocity)
{
	motion_magic_cruise_velocity_ = motion_magic_cruise_velocity;
}
double TalonFXProHWState::getMotionMagicCruiseVelocity(void) const
{
	return motion_magic_cruise_velocity_;
}
void TalonFXProHWState::setMotionMagicAcceleration(const double motion_magic_acceleration)
{
	motion_magic_acceleration_ = motion_magic_acceleration;
}
double TalonFXProHWState::getMotionMagicAcceleration(void) const
{
	return motion_magic_acceleration_;
}
void TalonFXProHWState::setMotionMagicJerk(const double motion_magic_jerk)
{
	motion_magic_jerk_ = motion_magic_jerk;
}
double TalonFXProHWState::getMotionMagicJerk(void) const
{
	return motion_magic_jerk_;
}
void TalonFXProHWState::setMotionMagicExpoKV(const double motion_magic_expo_kV)
{
	motion_magic_expo_kV_ = motion_magic_expo_kV;
}
double TalonFXProHWState::getMotionMagicExpoKV(void) const
{
	return motion_magic_expo_kV_;
}
void TalonFXProHWState::setMotionMagicExpoKA(const double motion_magic_expo_kA)
{
	motion_magic_expo_kA_ = motion_magic_expo_kA;
}
double TalonFXProHWState::getMotionMagicExpoKA(void) const
{
	return motion_magic_expo_kA_;
}

void TalonFXProHWState::setContinuousWrap(const bool continuous_wrap)
{
	continuous_wrap_ = continuous_wrap;
}
bool TalonFXProHWState::getContinuousWrap(void) const
{
	return continuous_wrap_;
}

void TalonFXProHWState::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
bool TalonFXProHWState::getClearStickyFaults(void) const
{
	return clear_sticky_faults_;
}

void TalonFXProHWState::setControlMode(const TalonMode control_mode)
{
	control_mode_ = control_mode;
}
TalonMode TalonFXProHWState::getControlMode(void) const
{
	return control_mode_;
}

void TalonFXProHWState::setControlOutput(const double control_output)
{
	control_output_ = control_output;
}
double TalonFXProHWState::getControlOutput(void) const
{
	return control_output_;
}

void TalonFXProHWState::setControlPosition(const double control_position)
{
	control_position_ = control_position;
}
double TalonFXProHWState::getControlPosition(void) const
{
	return control_position_;
}

void TalonFXProHWState::setControlVelocity(const double control_velocity)
{
	control_velocity_ = control_velocity;
}
double TalonFXProHWState::getControlVelocity(void) const
{
	return control_velocity_;
}

void TalonFXProHWState::setControlAcceleration(const double control_acceleration)
{
	control_acceleration_ = control_acceleration;
}
double TalonFXProHWState::getControlAcceleration(void) const
{
	return control_acceleration_;
}

void TalonFXProHWState::setControlJerk(const double control_jerk)
{
	control_jerk_ = control_jerk;
}
double TalonFXProHWState::getControlJerk(void) const
{
	return control_jerk_;
}

void TalonFXProHWState::setControlEnableFOC(const bool control_enable_foc)
{
	control_enable_foc_ = control_enable_foc;
}
bool TalonFXProHWState::getControlEnableFOC(void) const
{
	return control_enable_foc_;
}

void TalonFXProHWState::setControlOverrideBrakeDurNeutral(const bool control_override_brake_dur_neutral)
{
	control_override_brake_dur_neutral_ = control_override_brake_dur_neutral;
}
bool TalonFXProHWState::getControlOverrideBrakeDurNeutral(void) const
{
	return control_override_brake_dur_neutral_;
}

void TalonFXProHWState::setControlMaxAbsDutyCycle(const double control_max_abs_duty_cycle)
{
	control_max_abs_duty_cycle_ = control_max_abs_duty_cycle;
}
double TalonFXProHWState::getControlMaxAbsDutyCycle(void) const
{
	return control_max_abs_duty_cycle_;
}

void TalonFXProHWState::setControlDeadband(const double control_deadband)
{
	control_deadband_ = control_deadband;
}
double TalonFXProHWState::getControlDeadband(void) const
{
	return control_deadband_;
}

void TalonFXProHWState::setControlFeedforward(const double control_feedforward)
{
	control_feedforward_ = control_feedforward;
}
double TalonFXProHWState::getControlFeedforward(void) const
{
	return control_feedforward_;
}

void TalonFXProHWState::setControlSlot(const int control_slot)
{
	if ((control_slot < 0) || (control_slot > 2))
	{
		ROS_ERROR_STREAM("Invalid control slot (" << control_slot << ") passed to " << __PRETTY_FUNCTION__);
		return;
	}
	control_slot_ = control_slot;
}
int TalonFXProHWState::getControlSlot(void) const
{
	return control_slot_;
}

void TalonFXProHWState::setControlOpposeMasterDirection(const bool control_oppose_master_direction)
{
	control_oppose_master_direction_ = control_oppose_master_direction;
}
bool TalonFXProHWState::getControlOpposeMasterDirection(void) const
{
	return control_oppose_master_direction_;
}

void TalonFXProHWState::setControlLimitForwardMotion(const bool control_limit_forward_motion)
{
	control_limit_forward_motion_ = control_limit_forward_motion;
}
bool TalonFXProHWState::getControlLimitForwardMotion(void) const
{
	return control_limit_forward_motion_;
}

void TalonFXProHWState::setControlLimitReverseMotion(const bool control_limit_reverse_motion)
{
	control_limit_reverse_motion_ = control_limit_reverse_motion;
}
bool TalonFXProHWState::getControlLimitReverseMotion(void) const
{
	return control_limit_reverse_motion_;
}

void TalonFXProHWState::setControlDifferentialPosition(const double control_differential_position)
{
	control_differential_position_ = control_differential_position;
}
double TalonFXProHWState::getControlDifferentialPosition(void) const
{
	return control_differential_position_;
}

void TalonFXProHWState::setControlDifferentialSlot(const int control_differential_slot)
{
	if ((control_differential_slot < 0) || (control_differential_slot > 2))
	{
		ROS_ERROR_STREAM("Invalid control slot (" << control_differential_slot << ") passed to " << __PRETTY_FUNCTION__);
		return;
	}
	control_differential_slot_ = control_differential_slot;
}
int TalonFXProHWState::getControlDifferentialSlot(void) const
{
	return control_differential_slot_;
}

void TalonFXProHWState::setControlUseTimesync(const bool control_use_timesync)
{
	control_use_timesync_ = control_use_timesync;
}
bool TalonFXProHWState::getControlUseTimesync(void) const
{
	return control_use_timesync_;
}

void TalonFXProHWState::setEnableReadThread(const bool enable_read_thread)
{
	enable_read_thread_ = enable_read_thread;
}
bool TalonFXProHWState::getEnableReadThread(void) const
{
	return enable_read_thread_;
}

void TalonFXProHWState::setHasResetOccurred(const bool has_reset_occurred)
{
	has_reset_occurred_ = has_reset_occurred;
}
bool TalonFXProHWState::getHasResetOccurred(void) const
{
	return has_reset_occurred_;
}

void TalonFXProHWState::setVersionMajor(const int version_major)
{
	version_major_ = version_major;
}
int TalonFXProHWState::getVersionMajor(void) const
{
	return version_major_;
}

void TalonFXProHWState::setVersionMinor(const int version_minor)
{
	version_minor_ = version_minor;
}
int TalonFXProHWState::getVersionMinor(void) const
{
	return version_minor_;
}

void TalonFXProHWState::setVersionBugfix(const int version_bugfix)
{
	version_bugfix_ = version_bugfix;
}
int TalonFXProHWState::getVersionBugfix(void) const
{
	return version_bugfix_;
}

void TalonFXProHWState::setVersionBuild(const int version_build)
{
	version_build_ = version_build;
}
int TalonFXProHWState::getVersionBuild(void) const
{
	return version_build_;
}

void TalonFXProHWState::setMotorVoltage(const double motor_voltage)
{
	motor_voltage_ = motor_voltage;
}
double TalonFXProHWState::getMotorVoltage(void) const
{
	return motor_voltage_;
}

void TalonFXProHWState::setForwardLimit(const bool forward_limit)
{
	forward_limit_ = forward_limit;
}
bool TalonFXProHWState::getForwardLimit(void) const
{
	return forward_limit_;
}

void TalonFXProHWState::setReverseLimit(const bool reverse_limit)
{
	reverse_limit_ = reverse_limit;
}
bool TalonFXProHWState::getReverseLimit(void) const
{
	return reverse_limit_;
}

void TalonFXProHWState::setAppliedRotorPolarity(const Inverted applied_rotor_polarity)
{
	applied_rotor_polarity_ = applied_rotor_polarity;
}
Inverted TalonFXProHWState::getAppliedRotorPolarity(void) const
{
	return applied_rotor_polarity_;
}

void TalonFXProHWState::setDutyCycle(const double duty_cycle)
{
	duty_cycle_ = duty_cycle;
}
double TalonFXProHWState::getDutyCycle(void) const
{
	return duty_cycle_;
}

void TalonFXProHWState::setTorqueCurrent(const double torque_current)
{
	torque_current_ = torque_current;
}
double TalonFXProHWState::getTorqueCurrent(void) const
{
	return torque_current_;
}

void TalonFXProHWState::setStatorCurrent(const double stator_current)
{
	stator_current_ = stator_current;
}
double TalonFXProHWState::getStatorCurrent(void) const
{
	return stator_current_;
}

void TalonFXProHWState::setSupplyCurrent(const double supply_current)
{
	supply_current_ = supply_current;
}
double TalonFXProHWState::getSupplyCurrent(void) const
{
	return supply_current_;
}

void TalonFXProHWState::setSupplyVoltage(const double supply_voltage)
{
	supply_voltage_ = supply_voltage;
}
double TalonFXProHWState::getSupplyVoltage(void) const
{
	return supply_voltage_;
}

void TalonFXProHWState::setDeviceTemp(const double device_temp)
{
	device_temp_ = device_temp;
}
double TalonFXProHWState::getDeviceTemp(void) const
{
	return device_temp_;
}

void TalonFXProHWState::setProcessorTemp(const double processor_temp)
{
	processor_temp_ = processor_temp;
}
double TalonFXProHWState::getProcessorTemp(void) const
{
	return processor_temp_;
}

void TalonFXProHWState::setRotorVelocity(const double rotor_velocity)
{
	rotor_velocity_ = rotor_velocity;
}
double TalonFXProHWState::getRotorVelocity(void) const
{
	return rotor_velocity_;
}

void TalonFXProHWState::setRotorPosition(const double rotor_position)
{
	rotor_position_ = rotor_position;
}
double TalonFXProHWState::getRotorPosition(void) const
{
	return rotor_position_;
}

void TalonFXProHWState::setVelocity(const double velocity)
{
	velocity_ = velocity;
}
double TalonFXProHWState::getVelocity(void) const
{
	return velocity_;
}

void TalonFXProHWState::setPosition(const double position)
{
	position_ = position;
}
double TalonFXProHWState::getPosition(void) const
{
	return position_;
}

void TalonFXProHWState::setAcceleration(const double acceleration)
{
	acceleration_ = acceleration;
}
double TalonFXProHWState::getAcceleration(void) const
{
	return acceleration_;
}

void TalonFXProHWState::setMotionMagicIsRunning(const bool motion_magic_is_running)
{
	motion_magic_is_running_ = motion_magic_is_running;
}
bool TalonFXProHWState::getMotionMagicIsRunning(void) const
{
	return motion_magic_is_running_;
}

void TalonFXProHWState::setDeviceEnable(const bool device_enable)
{
	device_enable_ = device_enable;
}
bool TalonFXProHWState::getDeviceEnable(void) const
{
	return device_enable_;
}

void TalonFXProHWState::setDifferentialControlMode(const DifferentialControlMode differential_control_mode)
{
	differential_control_mode_ = differential_control_mode;
}
DifferentialControlMode TalonFXProHWState::getDifferentialControlMode(void) const
{
	return differential_control_mode_;
}

void TalonFXProHWState::setDifferentialAverageVelocity(const double differential_average_velocity)
{
	differential_average_velocity_ = differential_average_velocity;
}
double TalonFXProHWState::getDifferentialAverageVelocity(void) const
{
	return differential_average_velocity_;
}

void TalonFXProHWState::setDifferentialAveragePosition(const double differential_average_position)
{
	differential_average_position_ = differential_average_position;
}
double TalonFXProHWState::getDifferentialAveragePosition(void) const
{
	return differential_average_position_;
}

void TalonFXProHWState::setDifferentialDifferenceVelocity(const double differential_difference_velocity)
{
	differential_difference_velocity_ = differential_difference_velocity;
}
double TalonFXProHWState::getDifferentialDifferenceVelocity(void) const
{
	return differential_difference_velocity_;
}

void TalonFXProHWState::setDifferentialDifferencePosition(const double differential_difference_position)
{
	differential_difference_position_ = differential_difference_position;
}
double TalonFXProHWState::getDifferentialDifferencePosition(void) const
{
	return differential_difference_position_;
}

void TalonFXProHWState::setBridgeOutput(const BridgeOutput bridge_output_value)
{
	bridge_output_value_ = bridge_output_value;
}
BridgeOutput TalonFXProHWState::getBridgeOutput(void) const
{
	return bridge_output_value_;
}

void TalonFXProHWState::setFaultHardware(const bool fault_hardware) { fault_hardware_ = fault_hardware; }
bool TalonFXProHWState::getFaultHardware(void) const { return fault_hardware_; }
void TalonFXProHWState::setFaultProcTemp(const bool fault_proctemp) { fault_proctemp_ = fault_proctemp; }
bool TalonFXProHWState::getFaultProcTemp(void) const { return fault_proctemp_; }
void TalonFXProHWState::setFaultDeviceTemp(const bool fault_devicetemp) {fault_devicetemp_ = fault_devicetemp;}
bool TalonFXProHWState::getFaultDeviceTemp(void) const {return fault_devicetemp_;}
void TalonFXProHWState::setFaultUndervoltage(const bool fault_undervoltage) {fault_undervoltage_ = fault_undervoltage;}
bool TalonFXProHWState::getFaultUndervoltage(void) const {return fault_undervoltage_;}
void TalonFXProHWState::setFaultBootDuringEnable(const bool fault_bootduringenable) {fault_bootduringenable_ = fault_bootduringenable;}
bool TalonFXProHWState::getFaultBootDuringEnable(void) const {return fault_bootduringenable_;}
void TalonFXProHWState::setFaultUnlicensedFeatureInUse(const bool fault_unlicensed_feature_in_use) { fault_unlicensed_feature_in_use_ = fault_unlicensed_feature_in_use; }
void TalonFXProHWState::setFaultBridgeBrownout(const bool fault_bridgebrownout) {fault_bridgebrownout_ = fault_bridgebrownout;}
bool TalonFXProHWState::getFaultBridgeBrownout(void) const {return fault_bridgebrownout_;}
bool TalonFXProHWState::getFaultUnlicensedFeatureInUse(void) const { return fault_unlicensed_feature_in_use_; }
void TalonFXProHWState::setFaultRemoteSensorReset(const bool fault_remotesensorreset) { fault_remotesensorreset_ = fault_remotesensorreset; }
bool TalonFXProHWState::getFaultRemoteSensorReset(void) const { return fault_remotesensorreset_; }
void TalonFXProHWState::setFaultMissingDifferentialFX(const bool fault_missingdifferentialfx) { fault_missingdifferentialfx_ = fault_missingdifferentialfx; }
bool TalonFXProHWState::getFaultMissingDifferentialFX(void) const { return fault_missingdifferentialfx_; }
void TalonFXProHWState::setFaultRemoteSensorPosOverfow(const bool fault_remotesensorposoverflow) { fault_remotesensorposoverflow_ = fault_remotesensorposoverflow; }
bool TalonFXProHWState::getFaultRemoteSensorPosOverfow(void) const { return fault_remotesensorposoverflow_; }
void TalonFXProHWState::setFaultOverSupplyV(const bool fault_oversupplyv) {fault_oversupplyv_ = fault_oversupplyv;}
bool TalonFXProHWState::getFaultOverSupplyV(void) const {return fault_oversupplyv_;}
void TalonFXProHWState::setFaultUnstableSupplyV(const bool fault_unstablesupplyv) {fault_unstablesupplyv_ = fault_unstablesupplyv;}
bool TalonFXProHWState::getFaultUnstableSupplyV(void) const {return fault_unstablesupplyv_;}
void TalonFXProHWState::setFaultReverseHardLimit(const bool fault_reversehardlimit) {fault_reversehardlimit_ = fault_reversehardlimit;}
bool TalonFXProHWState::getFaultReverseHardLimit(void) const {return fault_reversehardlimit_;}
void TalonFXProHWState::setFaultForwardHardLimit(const bool fault_forwardhardlimit) {fault_forwardhardlimit_ = fault_forwardhardlimit;}
bool TalonFXProHWState::getFaultForwardHardLimit(void) const {return fault_forwardhardlimit_;}
void TalonFXProHWState::setFaultReverseSoftLimit(const bool fault_reversesoftlimit) {fault_reversesoftlimit_ = fault_reversesoftlimit;}
bool TalonFXProHWState::getFaultReverseSoftLimit(void) const {return fault_reversesoftlimit_;}
void TalonFXProHWState::setFaultForwardSoftLimit(const bool fault_forwardsoftlimit) {fault_forwardsoftlimit_ = fault_forwardsoftlimit;}
bool TalonFXProHWState::getFaultForwardSoftLimit(void) const {return fault_forwardsoftlimit_;}
void TalonFXProHWState::setFaultRemoteSensorDataInvalid(const bool fault_remotesensordatainvalid) {fault_remotesensordatainvalid_ = fault_remotesensordatainvalid;}
bool TalonFXProHWState::getFaultRemoteSensorDataInvalid(void) const {return fault_remotesensordatainvalid_;}
void TalonFXProHWState::setFaultFusedSensorOutOfSync(const bool fault_fusedsensoroutofsync) {fault_fusedsensoroutofsync_ = fault_fusedsensoroutofsync;}
bool TalonFXProHWState::getFaultFusedSensorOutOfSync(void) const {return fault_fusedsensoroutofsync_;}
void TalonFXProHWState::setFaultStatorCurrLimit(const bool fault_statorcurrlimit) {fault_statorcurrlimit_ = fault_statorcurrlimit;}
bool TalonFXProHWState::getFaultStatorCurrLimit(void) const {return fault_statorcurrlimit_;}
void TalonFXProHWState::setFaultSupplyCurrLimit(const bool fault_supplycurrlimit) {fault_supplycurrlimit_ = fault_supplycurrlimit;}
bool TalonFXProHWState::getFaultSupplyCurrLimit(void) const {return fault_supplycurrlimit_;}

void TalonFXProHWState::setStickyFaultHardware(const bool sticky_fault_hardware) { sticky_fault_hardware_ = sticky_fault_hardware; }
bool TalonFXProHWState::getStickyFaultHardware(void) const { return sticky_fault_hardware_; }
void TalonFXProHWState::setStickyFaultProcTemp(const bool sticky_fault_proctemp) { sticky_fault_proctemp_ = sticky_fault_proctemp; }
bool TalonFXProHWState::getStickyFaultProcTemp(void) const { return sticky_fault_proctemp_; }
void TalonFXProHWState::setStickyFaultDeviceTemp(const bool sticky_fault_devicetemp) {sticky_fault_devicetemp_ = sticky_fault_devicetemp;}
bool TalonFXProHWState::getStickyFaultDeviceTemp(void) const {return sticky_fault_devicetemp_;}
void TalonFXProHWState::setStickyFaultUndervoltage(const bool sticky_fault_undervoltage) {sticky_fault_undervoltage_ = sticky_fault_undervoltage;}
bool TalonFXProHWState::getStickyFaultUndervoltage(void) const {return sticky_fault_undervoltage_;}
void TalonFXProHWState::setStickyFaultBootDuringEnable(const bool sticky_fault_bootduringenable) {sticky_fault_bootduringenable_ = sticky_fault_bootduringenable;}
bool TalonFXProHWState::getStickyFaultBootDuringEnable(void) const {return sticky_fault_bootduringenable_;}
void TalonFXProHWState::setStickyFaultBridgeBrownout(const bool sticky_fault_bridgebrownout) {sticky_fault_bridgebrownout_ = sticky_fault_bridgebrownout;}
bool TalonFXProHWState::getStickyFaultBridgeBrownout(void) const {return sticky_fault_bridgebrownout_;}
void TalonFXProHWState::setStickyFaultUnlicensedFeatureInUse(const bool sticky_fault_unlicensed_feature_in_use) { sticky_fault_unlicensed_feature_in_use_ = sticky_fault_unlicensed_feature_in_use; }
bool TalonFXProHWState::getStickyFaultUnlicensedFeatureInUse(void) const { return sticky_fault_unlicensed_feature_in_use_; }
void TalonFXProHWState::setStickyFaultRemoteSensorReset(const bool sticky_fault_remotesensorreset) { sticky_fault_remotesensorreset_ = sticky_fault_remotesensorreset; }
bool TalonFXProHWState::getStickyFaultRemoteSensorReset(void) const { return sticky_fault_remotesensorreset_; }
void TalonFXProHWState::setStickyFaultMissingDifferentialFX(const bool sticky_fault_missingdifferentialfx) { sticky_fault_missingdifferentialfx_ = sticky_fault_missingdifferentialfx; }
bool TalonFXProHWState::getStickyFaultMissingDifferentialFX(void) const { return sticky_fault_missingdifferentialfx_; }
void TalonFXProHWState::setStickyFaultRemoteSensorPosOverfow(const bool sticky_fault_remotesensorposoverflow) { sticky_fault_remotesensorposoverflow_ = sticky_fault_remotesensorposoverflow; }
bool TalonFXProHWState::getStickyFaultRemoteSensorPosOverfow(void) const { return sticky_fault_remotesensorposoverflow_; }
void TalonFXProHWState::setStickyFaultOverSupplyV(const bool sticky_fault_oversupplyv) {sticky_fault_oversupplyv_ = sticky_fault_oversupplyv;}
bool TalonFXProHWState::getStickyFaultOverSupplyV(void) const {return sticky_fault_oversupplyv_;}
void TalonFXProHWState::setStickyFaultUnstableSupplyV(const bool sticky_fault_unstablesupplyv) {sticky_fault_unstablesupplyv_ = sticky_fault_unstablesupplyv;}
bool TalonFXProHWState::getStickyFaultUnstableSupplyV(void) const {return sticky_fault_unstablesupplyv_;}
void TalonFXProHWState::setStickyFaultReverseHardLimit(const bool sticky_fault_reversehardlimit) {sticky_fault_reversehardlimit_ = sticky_fault_reversehardlimit;}
bool TalonFXProHWState::getStickyFaultReverseHardLimit(void) const {return sticky_fault_reversehardlimit_;}
void TalonFXProHWState::setStickyFaultForwardHardLimit(const bool sticky_fault_forwardhardlimit) {sticky_fault_forwardhardlimit_ = sticky_fault_forwardhardlimit;}
bool TalonFXProHWState::getStickyFaultForwardHardLimit(void) const {return sticky_fault_forwardhardlimit_;}
void TalonFXProHWState::setStickyFaultReverseSoftLimit(const bool sticky_fault_reversesoftlimit) {sticky_fault_reversesoftlimit_ = sticky_fault_reversesoftlimit;}
bool TalonFXProHWState::getStickyFaultReverseSoftLimit(void) const {return sticky_fault_reversesoftlimit_;}
void TalonFXProHWState::setStickyFaultForwardSoftLimit(const bool sticky_fault_forwardsoftlimit) {sticky_fault_forwardsoftlimit_ = sticky_fault_forwardsoftlimit;}
bool TalonFXProHWState::getStickyFaultForwardSoftLimit(void) const {return sticky_fault_forwardsoftlimit_;}
void TalonFXProHWState::setStickyFaultRemoteSensorDataInvalid(const bool sticky_fault_remotesensordatainvalid) {sticky_fault_remotesensordatainvalid_ = sticky_fault_remotesensordatainvalid;}
bool TalonFXProHWState::getStickyFaultRemoteSensorDataInvalid(void) const {return sticky_fault_remotesensordatainvalid_;}
void TalonFXProHWState::setStickyFaultFusedSensorOutOfSync(const bool sticky_fault_fusedsensoroutofsync) {sticky_fault_fusedsensoroutofsync_ = sticky_fault_fusedsensoroutofsync;}
bool TalonFXProHWState::getStickyFaultFusedSensorOutOfSync(void) const {return sticky_fault_fusedsensoroutofsync_;}
void TalonFXProHWState::setStickyFaultStatorCurrLimit(const bool sticky_fault_statorcurrlimit) {sticky_fault_statorcurrlimit_ = sticky_fault_statorcurrlimit;}
bool TalonFXProHWState::getStickyFaultStatorCurrLimit(void) const {return sticky_fault_statorcurrlimit_;}
void TalonFXProHWState::setStickyFaultSupplyCurrLimit(const bool sticky_fault_supplycurrlimit) {sticky_fault_supplycurrlimit_ = sticky_fault_supplycurrlimit;}
bool TalonFXProHWState::getStickyFaultSupplyCurrLimit(void) const { return sticky_fault_supplycurrlimit_; }

void TalonFXProHWState::setClosedLoopProportionalOutput(const double closed_loop_proportional_output)
{
	closed_loop_proportional_output_ = closed_loop_proportional_output;
}
double TalonFXProHWState::getClosedLoopProportionalOutput(void) const
{
	return closed_loop_proportional_output_;
}

void TalonFXProHWState::setClosedLoopIntegratedOutput(const double closed_loop_integrated_output)
{
	closed_loop_integrated_output_ = closed_loop_integrated_output;
}
double TalonFXProHWState::getClosedLoopIntegratedOutput(void) const
{
	return closed_loop_integrated_output_;
}

void TalonFXProHWState::setClosedLoopFeedForward(const double closed_loop_feed_forward)
{
	closed_loop_feed_forward_ = closed_loop_feed_forward;
}
double TalonFXProHWState::getClosedLoopFeedForward(void) const
{
	return closed_loop_feed_forward_;
}

void TalonFXProHWState::setClosedLoopDerivativeOutput(const double closed_loop_derivative_output)
{
	closed_loop_derivative_output_ = closed_loop_derivative_output;
}
double TalonFXProHWState::getClosedLoopDerivativeOutput(void) const
{
	return closed_loop_derivative_output_;
}

void TalonFXProHWState::setClosedLoopOutput(const double closed_loop_output)
{
	closed_loop_output_ = closed_loop_output;
}
double TalonFXProHWState::getClosedLoopOutput(void) const
{
	return closed_loop_output_;
}

void TalonFXProHWState::setClosedLoopReference(const double closed_loop_reference)
{
	closed_loop_reference_ = closed_loop_reference;
}
double TalonFXProHWState::getClosedLoopReference(void) const
{
	return closed_loop_reference_;
}

void TalonFXProHWState::setClosedLoopReferenceSlope(const double closed_loop_refrence_slope)
{
	closed_loop_reference_slope_ = closed_loop_refrence_slope;
}
double TalonFXProHWState::getClosedLoopReferenceSlope(void) const
{
	return closed_loop_reference_slope_;
}

void TalonFXProHWState::setClosedLoopError(const double closed_loop_error)
{
	closed_loop_error_ = closed_loop_error;
}
double TalonFXProHWState::getClosedLoopError(void) const
{
	return closed_loop_error_;
}

void TalonFXProHWState::setDifferentialOutput(const double differential_output)
{
	differential_output_ = differential_output;
}
double TalonFXProHWState::getDifferentialOutput(void) const
{
	return differential_output_;
}

void TalonFXProHWState::setDifferentialClosedLoopProportionalOutput(const double differential_closed_loop_proportional_output)
{
	differential_closed_loop_proportional_output_ = differential_closed_loop_proportional_output;
}
double TalonFXProHWState::getDifferentialClosedLoopProportionalOutput(void) const
{
	return differential_closed_loop_proportional_output_;
}

void TalonFXProHWState::setDifferentialClosedLoopIntegratedOutput(const double differential_closed_loop_integrated_output)
{
	differential_closed_loop_integrated_output_ = differential_closed_loop_integrated_output;
}
double TalonFXProHWState::getDifferentialClosedLoopIntegratedOutput(void) const
{
	return differential_closed_loop_integrated_output_;
}

void TalonFXProHWState::setDifferentialClosedLoopFeedForward(const double differential_closed_loop_feed_forward)
{
	differential_closed_loop_feed_forward_ = differential_closed_loop_feed_forward;
}
double TalonFXProHWState::getDifferentialClosedLoopFeedForward(void) const
{
	return differential_closed_loop_feed_forward_;
}

void TalonFXProHWState::setDifferentialClosedLoopDerivativeOutput(const double differential_closed_loop_derivative_output)
{
	differential_closed_loop_derivative_output_ = differential_closed_loop_derivative_output;
}
double TalonFXProHWState::getDifferentialClosedLoopDerivativeOutput(void) const
{
	return differential_closed_loop_derivative_output_;
}

void TalonFXProHWState::setDifferentialClosedLoopOutput(const double differential_closed_loop_output)
{
	differential_closed_loop_output_ = differential_closed_loop_output;
}
double TalonFXProHWState::getDifferentialClosedLoopOutput(void) const
{
	return differential_closed_loop_output_;
}

void TalonFXProHWState::setDifferentialClosedLoopReference(const double differential_closed_loop_reference)
{
	differential_closed_loop_reference_ = differential_closed_loop_reference;
}
double TalonFXProHWState::getDifferentialClosedLoopReference(void) const
{
	return differential_closed_loop_reference_;
}

void TalonFXProHWState::setDifferentialClosedLoopReferenceSlope(const double differential_closed_loop_refrence_slope)
{
	differential_closed_loop_reference_slope_ = differential_closed_loop_refrence_slope;
}
double TalonFXProHWState::getDifferentialClosedLoopReferenceSlope(void) const
{
	return differential_closed_loop_reference_slope_;
}

void TalonFXProHWState::setDifferentialClosedLoopError(const double differential_closed_loop_error)
{
	differential_closed_loop_error_ = differential_closed_loop_error;
}
double TalonFXProHWState::getDifferentialClosedLoopError(void) const
{
	return differential_closed_loop_error_;
}

} // namespace talonfxpro::hardware_interface