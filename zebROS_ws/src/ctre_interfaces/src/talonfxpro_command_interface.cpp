#include "ctre_interfaces/talonfxpro_command_interface.h"

namespace hardware_interface::talonfxpro
{

// Set up default values
// Set most of the changed_ vars to true
// to force a write of these values to the Talon
// That should put the talon in a known state
// rather than relying on them being setup to
// a certain state previously
TalonFXProHWCommand::TalonFXProHWCommand(void) = default;

TalonFXProHWCommand::~TalonFXProHWCommand() = default;

void TalonFXProHWCommand::setkP(const double kP, const size_t index)
{
	if (index >= kP_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (fabs(kP - kP_[index]) > double_value_epsilon)
	{
		slot_changed_[index] = true;
		kP_[index] = kP;
	}
}
double TalonFXProHWCommand::getkP(const size_t index) const
{
	if (index >= kP_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kP_[index];
}

void TalonFXProHWCommand::setkI(const double kI, const size_t index)
{
	if (index >= kI_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (fabs(kI - kI_[index]) > double_value_epsilon)
	{
		slot_changed_[index] = true;
		kI_[index] = kI;
	}
}
double TalonFXProHWCommand::getkI(const size_t index) const
{
	if (index >= kI_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kI_[index];
}

void TalonFXProHWCommand::setkD(const double kD, const size_t index)
{
	if (index >= kD_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (fabs(kD - kD_[index]) > double_value_epsilon)
	{
		slot_changed_[index] = true;
		kD_[index] = kD;
	}
}
double TalonFXProHWCommand::getkD(const size_t index) const
{
	if (index >= kD_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kD_[index];
}

void TalonFXProHWCommand::setkS(const double kS, const size_t index)
{
	if (index >= kS_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (fabs(kS - kS_[index]) > double_value_epsilon)
	{
		slot_changed_[index] = true;
		kS_[index] = kS;
	}
}
double TalonFXProHWCommand::getkS(const size_t index) const
{
	if (index >= kS_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kS_[index];
}

void TalonFXProHWCommand::setkV(const double kV, const size_t index)
{
	if (index >= kV_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (fabs(kV - kV_[index]) > double_value_epsilon)
	{
		slot_changed_[index] = true;
		kV_[index] = kV;
	}
}
double TalonFXProHWCommand::getkV(const size_t index) const
{
	if (index >= kV_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kV_[index];
}

void TalonFXProHWCommand::setkA(const double kA, const size_t index)
{
	if (index >= kA_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (fabs(kA - kA_[index]) > double_value_epsilon)
	{
		slot_changed_[index] = true;
		kA_[index] = kA;
	}
}
double TalonFXProHWCommand::getkA(const size_t index) const
{
	if (index >= kA_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kA_[index];
}

void TalonFXProHWCommand::setkG(const double kG, const size_t index)
{
	if (index >= kG_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (fabs(kG - kG_[index]) > double_value_epsilon)
	{
		slot_changed_[index] = true;
		kG_[index] = kG;
	}
}
double TalonFXProHWCommand::getkG(const size_t index) const
{
	if (index >= kG_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return 0.0;
	}
	return kG_[index];
}

void TalonFXProHWCommand::setGravityType(const GravityType gravity_type, const size_t index)
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
	if (gravity_type != gravity_type_[index])
	{
		slot_changed_[index] = true;
		gravity_type_[index] = gravity_type;
	}
}
GravityType TalonFXProHWCommand::getGravityType(const size_t index) const
{
	if (index >= gravity_type_.size())
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return GravityType::Last;
	}
	return gravity_type_[index];
}

bool TalonFXProHWCommand::slotChanged(double &kP,
									  double &kI,
									  double &kD,
									  double &kS,
									  double &kV,
									  double &kA,
									  double &kG,
									  GravityType &gravity_type,
									  size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return false;
	}
	kP = kP_[index];
	kI = kI_[index];
	kD = kD_[index];
	kS = kS_[index];
	kV = kV_[index];
	kA = kA_[index];
	kG = kG_[index];
	gravity_type = gravity_type_[index];
	const auto ret = slot_changed_[index];
	slot_changed_[index] = false;
	return ret;
}

void TalonFXProHWCommand::resetSlot(const size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN_STREAM("Invalid index passed to " << __PRETTY_FUNCTION__);
		return;
	}
	slot_changed_[index] = true;
}


void TalonFXProHWCommand::setInvert(const Inverted invert)
{
	if (invert == Inverted::Uninitialized)
		return; // Don't warn on this?
	else if ((invert < Inverted::Uninitialized) ||
			 (invert >= Inverted::Last))
	{
		ROS_WARN("Invalid invert passed to TalonFXProHWCommand::setInvert()");
		return;
	}
	if (invert != invert_)
	{
		invert_         = invert;
		motor_output_config_changed_ = true;
	}
}
Inverted TalonFXProHWCommand::getInvert(void) const
{
	return invert_;
}

void TalonFXProHWCommand::setNeutralMode(const NeutralMode neutral_mode)
{
	if (neutral_mode == NeutralMode::Uninitialized)
	{
		return; // Don't warn on this?
	}
	if ((neutral_mode < NeutralMode::Uninitialized) ||
		(neutral_mode >= NeutralMode::Last))
	{
		ROS_WARN("Invalid neutral_mode passed to TalonFXProHWCommand::setNeutralMode()");
		return;
	}
	if (neutral_mode != neutral_mode_)
	{
		neutral_mode_         = neutral_mode;
		motor_output_config_changed_ = true;
	}
}
NeutralMode TalonFXProHWCommand::getNeutralMode(void) const
{
	return neutral_mode_;
}

void TalonFXProHWCommand::setDutyCycleNeutralDeadband(const double duty_cycle_neutral_deadband)
{
	if (fabs(duty_cycle_neutral_deadband - duty_cycle_neutral_deadband_) > double_value_epsilon)
	{
		duty_cycle_neutral_deadband_ = duty_cycle_neutral_deadband;
		motor_output_config_changed_ = true;
	}
}
double TalonFXProHWCommand::getDutyCycleNeutralDeadband(void) const
{
	return duty_cycle_neutral_deadband_;
}
void TalonFXProHWCommand::setPeakForwardDutyCycle(const double peak_forward_duty_cycle)
{
	if (fabs(peak_forward_duty_cycle - peak_forward_duty_cycle_) > double_value_epsilon)
	{
		peak_forward_duty_cycle_ = peak_forward_duty_cycle;
		motor_output_config_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakForwardDutyCycle(void) const
{
	return peak_forward_duty_cycle_;
}
void TalonFXProHWCommand::setPeakReverseDutyCycle(const double peak_reverse_duty_cycle)
{
	if (fabs(peak_reverse_duty_cycle - peak_reverse_duty_cycle_) > double_value_epsilon)
	{
		peak_reverse_duty_cycle_ = peak_reverse_duty_cycle;
		motor_output_config_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakReverseDutyCycle(void) const
{
	return peak_reverse_duty_cycle_;
}

bool TalonFXProHWCommand::motorOutputConfigChanged(Inverted &invert,
												   NeutralMode &neutral_mode,
												   double &duty_cycle_neutral_deadband,
												   double &peak_forward_duty_cycle,
												   double &peak_reverse_duty_cycle) const
{
	invert = invert_;
	neutral_mode = neutral_mode_;
	duty_cycle_neutral_deadband = duty_cycle_neutral_deadband_;
	peak_forward_duty_cycle = peak_forward_duty_cycle_;
	peak_reverse_duty_cycle = peak_reverse_duty_cycle_;
	const auto ret = motor_output_config_changed_;
	motor_output_config_changed_ = false;
	return ret;
}

void TalonFXProHWCommand::resetMotorOutputConfig(void)
{
	motor_output_config_changed_ = true;
}

void TalonFXProHWCommand::setStatorCurrentLimit(const double stator_current_limit)
{
	if (fabs(stator_current_limit_ - stator_current_limit) > double_value_epsilon)
	{
		stator_current_limit_ = stator_current_limit;
		current_limit_changed_ = true;
	}
}
double TalonFXProHWCommand::getStatorCurrentLimit(void) const
{
	return stator_current_limit_;
}
void TalonFXProHWCommand::setStatorCurrentLimitEnable(const bool stator_current_limit_enable)
{
	if (stator_current_limit_enable_ != stator_current_limit_enable)
	{
		stator_current_limit_enable_ = stator_current_limit_enable;
		current_limit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getStatorCurrentLimitEnable(void) const
{
	return stator_current_limit_enable_;
}

void TalonFXProHWCommand::setSupplyCurrentLimit(const double supply_current_limit)
{
	if (fabs(supply_current_limit_ - supply_current_limit) > double_value_epsilon)
	{
		supply_current_limit_ = supply_current_limit;
		current_limit_changed_ = true;
	}
}
double TalonFXProHWCommand::getSupplyCurrentLimit(void) const
{
	return supply_current_limit_;
}
void TalonFXProHWCommand::setSupplyCurrentLimitEnable(const bool supply_current_limit_enable)
{
	if (supply_current_limit_enable_ != supply_current_limit_enable)
	{
		supply_current_limit_enable_ = supply_current_limit_enable;
		current_limit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getSupplyCurrentLimitEnable(void) const
{
	return supply_current_limit_enable_;
}
void TalonFXProHWCommand::setSupplyCurrentThreshold(const double supply_current_threshold)
{
	if (fabs(supply_current_threshold_ - supply_current_threshold) > double_value_epsilon)
	{
		supply_current_threshold_ = supply_current_threshold;
		current_limit_changed_ = true;
	}
}
double TalonFXProHWCommand::getSupplyCurrentThreshold(void) const
{
	return supply_current_threshold_;
}
void TalonFXProHWCommand::setSupplyTimeThreshold(const double supply_time_threshold)
{
	if (fabs(supply_time_threshold_ - supply_time_threshold) > double_value_epsilon)
	{
		supply_time_threshold_ = supply_time_threshold;
		current_limit_changed_ = true;
	}
}
double TalonFXProHWCommand::getSupplyTimeThreshold(void) const
{
	return supply_time_threshold_;
}

bool TalonFXProHWCommand::currentLimitChanged(double &stator_current_limit,
											  bool &stator_current_limit_enable,
											  double &supply_current_limit,
											  bool &supply_current_limit_enable,
											  double &supply_current_threshold,
											  double &supply_time_threshold) const
{
	stator_current_limit = stator_current_limit_;
	stator_current_limit_enable = stator_current_limit_enable_;
	supply_current_limit = supply_current_limit_;
	supply_current_limit_enable = supply_current_limit_enable_;
	supply_current_threshold = supply_current_threshold_;
	supply_time_threshold = supply_time_threshold_;
	const bool ret = current_limit_changed_;
	current_limit_changed_ = false;
	return ret;
}
void  TalonFXProHWCommand::resetCurrentLimit(void)
{
	current_limit_changed_ = true;
}

void TalonFXProHWCommand::setSupplyVoltageTimeConstant(const double supply_voltage_time_constant) 
{
	if (fabs(supply_voltage_time_constant - supply_voltage_time_constant_) > double_value_epsilon)
	{
		supply_voltage_time_constant_ = supply_voltage_time_constant;
		voltage_configs_changed_ = true;
	}
}
double TalonFXProHWCommand::getSupplyVoltageTimeConstant(void) const
{
	return supply_voltage_time_constant_;
}

void TalonFXProHWCommand::setPeakForwardVoltage(const double peak_forward_voltage)
{
	if (fabs(peak_forward_voltage - peak_forward_voltage_) > double_value_epsilon)
	{
		peak_forward_voltage_ = peak_forward_voltage;
		voltage_configs_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakForwardVoltage(void) const
{
	return peak_forward_voltage_;
}

void TalonFXProHWCommand::setPeakReverseVoltage(const double peak_reverse_voltage)
{
	if (fabs(peak_reverse_voltage - peak_reverse_voltage_) > double_value_epsilon)
	{
		peak_reverse_voltage_ = peak_reverse_voltage;
		voltage_configs_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakReverseVoltage(void) const
{
	return peak_reverse_voltage_;
}

bool TalonFXProHWCommand::voltageConfigsChanged(double &supply_voltage_time_constant,
												double &peak_forward_voltage,
												double &peak_reverse_voltage) const
{
	supply_voltage_time_constant = supply_voltage_time_constant_;
	peak_forward_voltage = peak_forward_voltage_;
	peak_reverse_voltage = peak_reverse_voltage_;
	const auto ret = voltage_configs_changed_;
	voltage_configs_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetVoltageConfigs(void)
{
	voltage_configs_changed_ = true;
}

void TalonFXProHWCommand::setPeakForwardTorqueCurrent(const double peak_forward_torque_current)
{
	if (fabs(peak_forward_torque_current - peak_forward_torque_current_) > double_value_epsilon)
	{
		peak_forward_torque_current_ = peak_forward_torque_current;
		torque_current_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakForwardTorqueCurrent(void) const
{
	return peak_forward_torque_current_;
}
void TalonFXProHWCommand::setPeakReverseTorqueCurrent(const double peak_reverse_torque_current)
{
	if (fabs(peak_reverse_torque_current - peak_reverse_torque_current_) > double_value_epsilon)
	{
		peak_reverse_torque_current_ = peak_reverse_torque_current;
		torque_current_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakReverseTorqueCurrent(void) const
{
	return peak_reverse_torque_current_;
}
void TalonFXProHWCommand::setTorqueNeutralDeadband(const double torque_neutral_deadband)
{
	if (fabs(torque_neutral_deadband - torque_neutral_deadband_) > double_value_epsilon)
	{
		torque_neutral_deadband_ = torque_neutral_deadband;
		torque_current_changed_ = true;
	}
}
double TalonFXProHWCommand::getTorqueNeutralDeadband(void) const
{
	return torque_neutral_deadband_;
}

bool TalonFXProHWCommand::torqueCurrentChanged(double &peak_forward_torque_current,
											   double &peak_reverse_torque_current,
											   double &torque_neutral_deadband) const
{
	peak_forward_torque_current = peak_forward_torque_current_;
	peak_reverse_torque_current = peak_reverse_torque_current_;
	torque_neutral_deadband = torque_neutral_deadband_;
	const auto ret = torque_current_changed_;
	torque_current_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetTorqueCurrent(void)
{
	torque_current_changed_ = true;
}

void TalonFXProHWCommand::setFeedbackRotorOffset(const double feedback_rotor_offset)
{
	if (fabs(feedback_rotor_offset - feedback_rotor_offset_) > double_value_epsilon)
	{
		feedback_rotor_offset_ = feedback_rotor_offset;
		feedback_changed_ = true;
	}
}
double TalonFXProHWCommand::getFeedbackRotorOffset(void) const
{
	return feedback_rotor_offset_;
}

void TalonFXProHWCommand::setSensorToMechanismRatio(const double sensor_to_mechanism_ratio)
{
	if (fabs(sensor_to_mechanism_ratio - sensor_to_mechanism_ratio_) > double_value_epsilon)
	{
		sensor_to_mechanism_ratio_ = sensor_to_mechanism_ratio;
		feedback_changed_ = true;
	}
}
double TalonFXProHWCommand::getSensorToMechanismRatio(void) const
{
	return sensor_to_mechanism_ratio_;
}

void TalonFXProHWCommand::setRotorToSensorRatio(const double rotor_to_sensor_ratio)
{
	if (fabs(rotor_to_sensor_ratio - rotor_to_sensor_ratio_) > double_value_epsilon)
	{
		rotor_to_sensor_ratio_ = rotor_to_sensor_ratio;
		feedback_changed_ = true;
	}
}
double TalonFXProHWCommand::setRotorToSensorRatio(void) const
{
	return rotor_to_sensor_ratio_;
}

void TalonFXProHWCommand::setFeedbackSensorSource(const FeedbackSensorSource feedback_sensor_source)
{
	if (feedback_sensor_source != feedback_sensor_source_)
	{
		feedback_sensor_source_ = feedback_sensor_source;
		feedback_changed_ = true;
	}
}
FeedbackSensorSource TalonFXProHWCommand::getFeedbackSensorSource(void) const
{
	return feedback_sensor_source_;
}

void TalonFXProHWCommand::setFeedbackRemoteSensorID(const int feedback_remote_sensor_id)
{
	if (feedback_remote_sensor_id != feedback_remote_sensor_id_)
	{
		feedback_remote_sensor_id_ = feedback_remote_sensor_id;
		feedback_changed_ = true;
	}
}
int TalonFXProHWCommand::setFeedbackRemoteSensorID(void) const
{
	return feedback_remote_sensor_id_;
}

bool TalonFXProHWCommand::feebackChanged(double &feedback_rotor_offset,
										 double &sensor_to_mechanism_ratio,
										 double &rotor_to_sensor_ratio,
										 FeedbackSensorSource &feedback_sensor_source,
										 int &feedback_remote_sensor_id) const
{
	feedback_rotor_offset = feedback_rotor_offset_;
	sensor_to_mechanism_ratio = sensor_to_mechanism_ratio_;
	rotor_to_sensor_ratio = rotor_to_sensor_ratio_;
	feedback_sensor_source = feedback_sensor_source_;
	feedback_remote_sensor_id = feedback_remote_sensor_id_;
	const auto ret = feedback_changed_;
	feedback_changed_ = false;
	return ret;
}

void TalonFXProHWCommand::resetFeedback(void)
{
	feedback_changed_ = true;
}

void TalonFXProHWCommand::setDifferentialSensorSource(const DifferentialSensorSource differential_sensor_source)
{
	if ((differential_sensor_source <= hardware_interface::talonfxpro::DifferentialSensorSource::First) ||
		(differential_sensor_source >= hardware_interface::talonfxpro::DifferentialSensorSource::Last))
	{
		ROS_WARN_STREAM("Invalid differential_sensor_source " << static_cast<int>(differential_sensor_source) << " passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (differential_sensor_source != differential_sensor_source_)
	{
		differential_sensor_source_ = differential_sensor_source;
		differential_sensors_changed_ = true;
	}
}
DifferentialSensorSource TalonFXProHWCommand::getDifferentialSensorSource(void) const
{
	return differential_sensor_source_;
}

void TalonFXProHWCommand::setDifferentialTalonFXSensorID(const int differential_talonfx_sensor_id)
{
	if (differential_talonfx_sensor_id != differential_talonfx_sensor_id_)
	{
		differential_talonfx_sensor_id_ = differential_talonfx_sensor_id;
		differential_sensors_changed_ = true;
	}
}
int  TalonFXProHWCommand::getDifferentialTalonFXSensorID(void) const
{
	return differential_talonfx_sensor_id_;
}

void TalonFXProHWCommand::setDifferentialRemoteSensorID(const int differential_remote_sensor_id)
{
	if (differential_remote_sensor_id != differential_remote_sensor_id_)
	{
		differential_remote_sensor_id_ = differential_remote_sensor_id;
		differential_sensors_changed_ = true;
	}
}
int  TalonFXProHWCommand::getDifferentialRemoteSensorID(void) const
{
	return differential_remote_sensor_id_;
}

bool TalonFXProHWCommand::differentialSensorsChanged(DifferentialSensorSource &differential_sensor_source,
													 int &differential_talonfx_sensor_id,
													 int &differential_remote_sensor_id) const
{
	differential_sensor_source = differential_sensor_source_;
	differential_talonfx_sensor_id = differential_talonfx_sensor_id_;
	differential_remote_sensor_id = differential_remote_sensor_id_;
	const auto ret = differential_sensors_changed_;
	differential_sensors_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetDifferentialSensors(void)
{
	differential_sensors_changed_ = true;
}

void TalonFXProHWCommand::setPeakDifferentialDutyCycle(const double peak_differential_duty_cycle)
{
	if (fabs(peak_differential_duty_cycle - peak_differential_duty_cycle_) > double_value_epsilon)
	{
		peak_differential_duty_cycle_ = peak_differential_duty_cycle;
		differential_constants_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakDifferentialDutyCycle(void) const
{
	return peak_differential_duty_cycle_;
}

void TalonFXProHWCommand::setPeakDifferentialVoltage(const double peak_differential_voltage)
{
	if (fabs(peak_differential_voltage - peak_differential_voltage_) > double_value_epsilon)
	{
		peak_differential_voltage_ = peak_differential_voltage;
		differential_constants_changed_ = true;
	}
}
double TalonFXProHWCommand::getPeakDifferentialVoltage(void) const
{
	return peak_differential_voltage_;
}

void TalonFXProHWCommand::setPeakDifferentialTorqueCurrent(const double peak_differential_torque_current)
{
	if (fabs(peak_differential_torque_current - peak_differential_torque_current_) > double_value_epsilon)
	{
		peak_differential_torque_current_ = peak_differential_torque_current;
		differential_constants_changed_ = true;
	}
}

double TalonFXProHWCommand::getPeakDifferentialTorqueCurrent(void) const
{
	return peak_differential_torque_current_;
}
bool TalonFXProHWCommand::differentialConstantsChanged(double &peak_differential_duty_cycle,
													   double &peak_differential_voltage,
													   double &peak_differential_torque_current) const
{
	peak_differential_duty_cycle = peak_differential_duty_cycle_;
	peak_differential_voltage = peak_differential_voltage_;
	peak_differential_torque_current = peak_differential_torque_current_;
	const auto ret = differential_constants_changed_;
	differential_constants_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetDifferentialConstants(void)
{
	differential_constants_changed_ = true;
}

void TalonFXProHWCommand::setDutyCycleOpenLoopRampPeriod(const double duty_cycle_open_loop_ramp_period)
{
	if (fabs(duty_cycle_open_loop_ramp_period - duty_cycle_open_loop_ramp_period_) > double_value_epsilon)
	{
		duty_cycle_open_loop_ramp_period_ = duty_cycle_open_loop_ramp_period;
		open_loop_ramps_changed_ = true;
	}
}
double TalonFXProHWCommand::getDutyCycleOpenLoopRampPeriod(void) const
{
	return duty_cycle_open_loop_ramp_period_;
}

void TalonFXProHWCommand::setVoltageOpenLoopRampPeriod(const double voltage_open_loop_ramp_period)
{
	if (fabs(voltage_open_loop_ramp_period - voltage_open_loop_ramp_period_) > double_value_epsilon)
	{
		voltage_open_loop_ramp_period_ = voltage_open_loop_ramp_period;
		open_loop_ramps_changed_ = true;
	}
}
double TalonFXProHWCommand::getVoltageOpenLoopRampPeriod(void) const
{
	return voltage_open_loop_ramp_period_;
}

void TalonFXProHWCommand::setTorqueOpenLoopRampPeriod(const double torque_open_loop_ramp_period) 
{
	if (fabs(torque_open_loop_ramp_period - torque_open_loop_ramp_period_) > double_value_epsilon)
	{
		torque_open_loop_ramp_period_ = torque_open_loop_ramp_period;
		open_loop_ramps_changed_ = true;
	}
}
double TalonFXProHWCommand::getTorqueOpenLoopRampPeriod(void) const
{
	return torque_open_loop_ramp_period_;
}

bool TalonFXProHWCommand::openLoopRampsChanged(double &duty_cycle_open_loop_ramp_period,
											   double &voltage_open_loop_ramp_period,
											   double &torque_open_loop_ramp_period) const
{
	duty_cycle_open_loop_ramp_period = duty_cycle_open_loop_ramp_period_;
	voltage_open_loop_ramp_period = voltage_open_loop_ramp_period_;
	torque_open_loop_ramp_period = torque_open_loop_ramp_period_;
	const auto ret = open_loop_ramps_changed_;
	open_loop_ramps_changed_ = false;
	return ret;
}

void TalonFXProHWCommand::resetOpenLoopRamps(void)
{
	open_loop_ramps_changed_ = true;
}

void TalonFXProHWCommand::setDutyCycleClosedLoopRampPeriod(const double duty_cycle_closed_loop_ramp_period)
{
	if (fabs(duty_cycle_closed_loop_ramp_period - duty_cycle_closed_loop_ramp_period_) > double_value_epsilon)
	{
		duty_cycle_closed_loop_ramp_period_ = duty_cycle_closed_loop_ramp_period;
		closed_loop_ramps_changed_ = true;
	}
}
double TalonFXProHWCommand::getDutyCycleClosedLoopRampPeriod(void) const
{
	return duty_cycle_closed_loop_ramp_period_;
}

void TalonFXProHWCommand::setVoltageClosedLoopRampPeriod(const double voltage_closed_loop_ramp_period)
{
	if (fabs(voltage_closed_loop_ramp_period - voltage_closed_loop_ramp_period_) > double_value_epsilon)
	{
		voltage_closed_loop_ramp_period_ = voltage_closed_loop_ramp_period;
		closed_loop_ramps_changed_ = true;
	}
}
double TalonFXProHWCommand::getVoltageClosedLoopRampPeriod(void) const
{
	return voltage_closed_loop_ramp_period_;
}

void TalonFXProHWCommand::setTorqueClosedLoopRampPeriod(const double torque_closed_loop_ramp_period) 
{
	if (fabs(torque_closed_loop_ramp_period - torque_closed_loop_ramp_period_) > double_value_epsilon)
	{
		torque_closed_loop_ramp_period_ = torque_closed_loop_ramp_period;
		closed_loop_ramps_changed_ = true;
	}
}
double TalonFXProHWCommand::getTorqueClosedLoopRampPeriod(void) const
{
	return torque_closed_loop_ramp_period_;
}

bool TalonFXProHWCommand::closedLoopRampsChanged(double &duty_cycle_closed_loop_ramp_period,
												 double &voltage_closed_loop_ramp_period,
												 double &torque_closed_loop_ramp_period) const
{
	duty_cycle_closed_loop_ramp_period = duty_cycle_closed_loop_ramp_period_;
	voltage_closed_loop_ramp_period = voltage_closed_loop_ramp_period_;
	torque_closed_loop_ramp_period = torque_closed_loop_ramp_period_;
	const auto ret = closed_loop_ramps_changed_;
	closed_loop_ramps_changed_ = false;
	return ret;
}

void TalonFXProHWCommand::resetClosedLoopRamps(void)
{
	closed_loop_ramps_changed_ = true;
}

void TalonFXProHWCommand::setForwardLimitType(const LimitType forward_limit_type)
{
	if (forward_limit_type != forward_limit_type_)
	{
		forward_limit_type_ = forward_limit_type;
		limit_changed_ = true;
	}
}
LimitType TalonFXProHWCommand::getForwardLimitType(void) const
{
	return forward_limit_type_;
}

void TalonFXProHWCommand::setForwardLimitAutosetPositionEnable(const bool forward_limit_autoset_position_enable)
{
	if (forward_limit_autoset_position_enable_ != forward_limit_autoset_position_enable)
	{
		forward_limit_autoset_position_enable_ = forward_limit_autoset_position_enable;
		limit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getForwardLimitAutosetPositionEnable(void) const
{
	return forward_limit_autoset_position_enable_;
}

void TalonFXProHWCommand::setForwardLimitAutosetPositionValue(const double forward_limit_autoset_position_value)
{
	if (fabs(forward_limit_autoset_position_value - forward_limit_autoset_position_value_) > double_value_epsilon)
	{
		forward_limit_autoset_position_value_ = forward_limit_autoset_position_value;
		limit_changed_ = true;
	}
}
double TalonFXProHWCommand::setForwardLimitAutosetPositionValue(void) const
{
	return forward_limit_autoset_position_value_;
}

void TalonFXProHWCommand::setForwardLimitEnable(bool forward_limit_enable)
{
	if (forward_limit_enable != forward_limit_enable_)
	{
		forward_limit_enable_ = forward_limit_enable;
		limit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getForwardLimitEnable(void) const
{
	return forward_limit_enable_;
}

void TalonFXProHWCommand::setForwardLimitSource(const LimitSource forward_limit_source)
{
	if (forward_limit_source != forward_limit_source_)
	{
		forward_limit_source_ = forward_limit_source;
		limit_changed_ = true;
	}
}
LimitSource TalonFXProHWCommand::getForwardLimitSource(void) const
{
	return forward_limit_source_;
}

void TalonFXProHWCommand::setForwardLimitRemoteSensorID(const int forward_limit_remote_sensor_id)
{
	if (forward_limit_remote_sensor_id != forward_limit_remote_sensor_id_)
	{
		forward_limit_remote_sensor_id_ = forward_limit_remote_sensor_id;
		limit_changed_ = true;
	}
}
int TalonFXProHWCommand::getForwardLimitRemoteSensorID(void) const
{
	return forward_limit_remote_sensor_id_;
}

void TalonFXProHWCommand::setReverseLimitType(const LimitType reverse_limit_type)
{
	if (reverse_limit_type != reverse_limit_type_)
	{
		reverse_limit_type_ = reverse_limit_type;
		limit_changed_ = true;
	}
}
LimitType TalonFXProHWCommand::getReverseLimitType(void) const
{
	return reverse_limit_type_;
}

void TalonFXProHWCommand::setReverseLimitAutosetPositionEnable(const bool reverse_limit_autoset_position_enable)
{
	if (reverse_limit_autoset_position_enable_ != reverse_limit_autoset_position_enable)
	{
		reverse_limit_autoset_position_enable_ = reverse_limit_autoset_position_enable;
		limit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getReverseLimitAutosetPositionEnable(void) const
{
	return reverse_limit_autoset_position_enable_;
}

void TalonFXProHWCommand::setReverseLimitAutosetPositionValue(const double reverse_limit_autoset_position_value)
{
	if (fabs(reverse_limit_autoset_position_value - reverse_limit_autoset_position_value_) > double_value_epsilon)
	{
		reverse_limit_autoset_position_value_ = reverse_limit_autoset_position_value;
		limit_changed_ = true;
	}
}
double TalonFXProHWCommand::setReverseLimitAutosetPositionValue(void) const
{
	return reverse_limit_autoset_position_value_;
}

void TalonFXProHWCommand::setReverseLimitEnable(const bool reverse_limit_enable)
{
	if (reverse_limit_enable != reverse_limit_enable_)
	{
		reverse_limit_enable_ = reverse_limit_enable;
		limit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getReverseLimitEnable(void) const
{
	return reverse_limit_enable_;
}

void TalonFXProHWCommand::setReverseLimitSource(const LimitSource reverse_limit_source)
{
	if (reverse_limit_source != reverse_limit_source_)
	{
		reverse_limit_source_ = reverse_limit_source;
		limit_changed_ = true;
	}
}
LimitSource TalonFXProHWCommand::getReverseLimitSource(void) const
{
	return reverse_limit_source_;
}

void TalonFXProHWCommand::setReverseLimitRemoteSensorID(const int reverse_limit_remote_sensor_id)
{
	if (reverse_limit_remote_sensor_id != reverse_limit_remote_sensor_id_)
	{
		reverse_limit_remote_sensor_id_ = reverse_limit_remote_sensor_id;
		limit_changed_ = true;
	}
}
int TalonFXProHWCommand::getReverseLimitRemoteSensorID(void) const
{
	return reverse_limit_remote_sensor_id_;
}

bool TalonFXProHWCommand::limitChanged(LimitType &forward_limit_type,
									   bool &forward_limit_autoset_position_enable,
									   double &forward_limit_autoset_position_value,
									   bool &forward_limit_enable,
									   LimitSource &forward_limit_source,
									   int &forward_limit_remote_sensor_id,
									   LimitType &reverse_limit_type,
									   bool &reverse_limit_autoset_position_enable,
									   double &reverse_limit_autoset_position_value,
									   bool &reverse_limit_enable,
									   LimitSource &reverse_limit_source,
									   int &reverse_limit_remote_sensor_id) const
{
	forward_limit_type = forward_limit_type_;
	forward_limit_autoset_position_enable = forward_limit_autoset_position_enable_;
	forward_limit_autoset_position_value = forward_limit_autoset_position_value_;
	forward_limit_enable = forward_limit_enable_;
	forward_limit_source = forward_limit_source_;
	forward_limit_remote_sensor_id = forward_limit_remote_sensor_id_;
	reverse_limit_type = reverse_limit_type_;
	reverse_limit_autoset_position_enable = reverse_limit_autoset_position_enable_;
	reverse_limit_autoset_position_value = reverse_limit_autoset_position_value_;
	reverse_limit_enable = reverse_limit_enable_;
	reverse_limit_source = reverse_limit_source_;
	reverse_limit_remote_sensor_id = reverse_limit_remote_sensor_id_;
	const auto ret = limit_changed_;
	limit_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetLimit(void)
{
	limit_changed_ = true;
}

void TalonFXProHWCommand::setBeepOnBoot(const bool beep_on_boot)
{
	if (beep_on_boot != beep_on_boot_)
	{
		beep_on_boot_ = beep_on_boot;
		audio_changed_ = true;
	}
}
bool TalonFXProHWCommand::getBeepOnBoot(void) const
{
	return beep_on_boot_;
}
void TalonFXProHWCommand::setBeepOnConfig(const bool beep_on_config)
{
	if (beep_on_config != beep_on_config_)
	{
		beep_on_config_ = beep_on_config;
		audio_changed_ = true;
	}
}
bool TalonFXProHWCommand::getBeepOnConfig(void) const
{
	return beep_on_config_;
}
void TalonFXProHWCommand::setAllowMusicDurDisable(const bool allow_music_dur_disable)
{
	if (allow_music_dur_disable != allow_music_dur_disable_)
	{
		allow_music_dur_disable_ = allow_music_dur_disable;
		audio_changed_ = true;
	}
}
bool TalonFXProHWCommand::getAllowMusicDurDisable(void) const
{
	return allow_music_dur_disable_;
}
bool TalonFXProHWCommand::audioChanged(bool &beep_on_boot,
									   bool &beep_on_config,
									   bool &allow_music_dur_disable) const
{
	beep_on_boot = beep_on_boot_;
	beep_on_config = beep_on_config_;
	allow_music_dur_disable = allow_music_dur_disable_;
	const auto ret = audio_changed_;
	audio_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetAudio(void)
{
	audio_changed_ = true;
}

void TalonFXProHWCommand::setForwardSoftLimitEnable(const bool enable)
{
	if (enable != softlimit_forward_enable_)
	{
		softlimit_forward_enable_ = enable;
		softlimit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getForwardSoftLimitEnable(void) const
{
	return softlimit_forward_enable_;
}
void TalonFXProHWCommand::setReverseSoftLimitEnable(const bool enable)
{
	if (enable != softlimit_reverse_enable_)
	{
		softlimit_reverse_enable_ = enable;
		softlimit_changed_ = true;
	}
}
bool TalonFXProHWCommand::getReverseSoftLimitEnable(void) const
{
	return softlimit_reverse_enable_;
}
void TalonFXProHWCommand::setForwardSoftLimitThreshold(const double threshold)
{
	if (fabs(threshold - softlimit_forward_threshold_) > double_value_epsilon)
	{
		softlimit_forward_threshold_ = threshold;
		softlimit_changed_ = true;
	}
}
double TalonFXProHWCommand::getForwardSoftLimitThreshold(void) const
{
	return softlimit_forward_threshold_;
}

void TalonFXProHWCommand::setReverseSoftLimitThreshold(const double threshold)
{
	if (fabs(threshold - softlimit_reverse_threshold_) > double_value_epsilon)
	{
		softlimit_reverse_threshold_ = threshold;
		softlimit_changed_ = true;
	}
}
double TalonFXProHWCommand::getReverseSoftLimitThreshold(void) const
{
	return softlimit_reverse_threshold_;
}

bool TalonFXProHWCommand::softLimitChanged(bool &forward_enable, bool &reverse_enable, double &forward_threshold, double &reverse_threshold) const
{
	forward_enable = softlimit_forward_enable_;
	reverse_enable = softlimit_reverse_enable_;
	forward_threshold = softlimit_forward_threshold_;
	reverse_threshold = softlimit_reverse_threshold_;
	const auto ret = softlimit_changed_;
	softlimit_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetSoftLimit(void)
{
	softlimit_changed_ = true;
}

void TalonFXProHWCommand::setMotionMagicCruiseVelocity(const double motion_magic_cruise_velocity)
{
	if (fabs(motion_magic_cruise_velocity - motion_magic_cruise_velocity_) > double_value_epsilon)
	{
		motion_magic_cruise_velocity_ = motion_magic_cruise_velocity;
		motion_magic_changed_ = true;
	}
}
double TalonFXProHWCommand::getMotionMagicCruiseVelocity(void) const
{
	return motion_magic_cruise_velocity_;
}
void TalonFXProHWCommand::setMotionMagicAcceleration(const double motion_magic_acceleration)
{
	if (fabs(motion_magic_acceleration - motion_magic_acceleration_) > double_value_epsilon)
	{
		motion_magic_acceleration_ = motion_magic_acceleration;
		motion_magic_changed_ = true;
	}
}
double TalonFXProHWCommand::getMotionMagicAcceleration(void) const
{
	return motion_magic_acceleration_;
}
void TalonFXProHWCommand::setMotionMagicJerk(const double motion_magic_jerk)
{
	if (fabs(motion_magic_jerk - motion_magic_jerk_) > double_value_epsilon)
	{
		motion_magic_jerk_ = motion_magic_jerk;
		motion_magic_changed_ = true;
	}
}
double TalonFXProHWCommand::getMotionMagicJerk(void) const
{
	return motion_magic_jerk_;
}

void TalonFXProHWCommand::setMotionMagicExpoKV(const double motion_magic_expo_kV)
{
	if (fabs(motion_magic_expo_kV - motion_magic_expo_kV_) > double_value_epsilon)
	{
		motion_magic_expo_kV_ = motion_magic_expo_kV;
		motion_magic_changed_ = true;
	}
}
double TalonFXProHWCommand::getMotionMagicExpoKV(void) const
{
	return motion_magic_expo_kV_;
}

void TalonFXProHWCommand::setMotionMagicExpoKA(const double motion_magic_expo_kA)
{
	if (fabs(motion_magic_expo_kA - motion_magic_expo_kA_) > double_value_epsilon)
	{
		motion_magic_expo_kA_ = motion_magic_expo_kA;
		motion_magic_changed_ = true;
	}
}
double TalonFXProHWCommand::getMotionMagicExpoKA(void) const
{
	return motion_magic_expo_kA_;
}

bool TalonFXProHWCommand::motionMagicChanged(double &motion_magic_cruise_velocity,
											 double &motion_magic_acceleration,
											 double &motion_magic_jerk,
											 double &motion_magic_expo_kV,
											 double &motion_magic_expo_kA) const
{
	motion_magic_cruise_velocity = motion_magic_cruise_velocity_;
	motion_magic_acceleration = motion_magic_acceleration_;
	motion_magic_jerk = motion_magic_jerk_;
	motion_magic_expo_kV = motion_magic_expo_kV_;
	motion_magic_expo_kA = motion_magic_expo_kA_;
	const auto ret = motion_magic_changed_;
	motion_magic_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetMotionMagic(void)
{
	motion_magic_changed_ = true;
}

void TalonFXProHWCommand::setContinuousWrap(const bool continuous_wrap)
{
	if (continuous_wrap != continuous_wrap_)
	{
		continuous_wrap_ = continuous_wrap;
		continuous_wrap_changed_ = true;
	}
}
bool TalonFXProHWCommand::getContinuousWrap(void) const
{
	return continuous_wrap_;
}
bool TalonFXProHWCommand::continuousWrapChanged(bool &continuous_wrap) const
{
	continuous_wrap = continuous_wrap_;
	const auto ret = continuous_wrap_changed_;
	continuous_wrap_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetContinuousWrap(void)
{
	continuous_wrap_changed_ = true;
}

void TalonFXProHWCommand::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
bool TalonFXProHWCommand::getClearStickyFaults(void) const
{
	return clear_sticky_faults_;
}
bool TalonFXProHWCommand::clearStickyFaultsChanged(void) const
{
	const auto ret = clear_sticky_faults_;
	clear_sticky_faults_ = false;
	return ret;
}

void TalonFXProHWCommand::setRotorPosition(const double position)
{
	rotor_position_ = position;
	rotor_position_changed_ = true;
}
double TalonFXProHWCommand::getRotorPosition(void) const
{
	return rotor_position_;
}
bool TalonFXProHWCommand::rotorPositionChanged(double &position) const
{
	position = rotor_position_;
	const bool ret = rotor_position_changed_;
	rotor_position_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetRotorPosition(void)
{
	rotor_position_changed_ = true;
}

void TalonFXProHWCommand::setControlMode(const TalonMode control_mode)
{
	if (control_mode != control_mode_)
	{
		ROS_INFO_STREAM("setControlMode : mode = " << (int)control_mode);
		control_mode_ = control_mode;
		control_changed_ = true;
	}
}
TalonMode TalonFXProHWCommand::getControlMode(void) const
{
	return control_mode_;
}

void TalonFXProHWCommand::setControlOutput(const double control_output)
{
	if (fabs(control_output - control_output_) > double_value_epsilon)
	{
		control_output_ = control_output;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlOutput(void) const
{
	return control_output_;
}

void TalonFXProHWCommand::setControlPosition(const double control_position)
{
	if (fabs(control_position - control_position_) > double_value_epsilon)
	{
		control_position_ = control_position;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlPosition(void) const
{
	return control_position_;
}

void TalonFXProHWCommand::setControlVelocity(const double control_velocity)
{
	if (fabs(control_velocity - control_velocity_) > double_value_epsilon)
	{
		control_velocity_ = control_velocity;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlVelocity(void) const
{
	return control_velocity_;
}

void TalonFXProHWCommand::setControlAcceleration(const double control_acceleration)
{
	if (fabs(control_acceleration - control_acceleration_) > double_value_epsilon)
	{
		control_acceleration_ = control_acceleration;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlAcceleration(void) const
{
	return control_acceleration_;
}

void TalonFXProHWCommand::setControlJerk(const double control_jerk)
{
	if (fabs(control_jerk - control_jerk_) > double_value_epsilon)
	{
		control_jerk_ = control_jerk;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlJerk(void) const
{
	return control_jerk_;
}

void TalonFXProHWCommand::setControlEnableFOC(const bool control_enable_foc)
{
	if (control_enable_foc != control_enable_foc_)
	{
		control_enable_foc_ = control_enable_foc;
		control_changed_ = true;
	}
}
bool TalonFXProHWCommand::getControlEnableFOC(void) const
{
	return control_enable_foc_;
}

void TalonFXProHWCommand::setControlOverrideBrakeDurNeutral(const bool control_override_brake_dur_neutral)
{
	if (control_override_brake_dur_neutral != control_override_brake_dur_neutral_)
	{
		control_override_brake_dur_neutral_ = control_override_brake_dur_neutral;
		control_changed_ = true;
	}
}
bool TalonFXProHWCommand::getControlOverrideBrakeDurNeutral(void) const
{
	return control_override_brake_dur_neutral_;
}

void TalonFXProHWCommand::setControlMaxAbsDutyCycle(const double control_max_abs_duty_cycle)
{
	if (fabs(control_max_abs_duty_cycle - control_max_abs_duty_cycle_) > double_value_epsilon)
	{
		control_max_abs_duty_cycle_ = control_max_abs_duty_cycle;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlMaxAbsDutyCycle(void) const
{
	return control_max_abs_duty_cycle_;
}

void TalonFXProHWCommand::setControlDeadband(const double control_deadband)
{
	if (fabs(control_deadband - control_deadband_) > double_value_epsilon)
	{
		control_deadband_ = control_deadband;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlDeadband(void) const
{
	return control_deadband_;
}

void TalonFXProHWCommand::setControlFeedforward(const double control_feedforward)
{
	if (fabs(control_feedforward - control_feedforward_) > double_value_epsilon)
	{
		control_feedforward_ = control_feedforward;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlFeedforward(void) const
{
	return control_feedforward_;
}

void TalonFXProHWCommand::setControlSlot(const int control_slot)
{
	if ((control_slot < 0) || (control_slot > 2))
	{
		ROS_ERROR_STREAM("Invalid control slot (" << control_slot << ") passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (control_slot != control_slot_)
	{
		control_slot_ = control_slot;
		control_changed_ = true;
	}
}
int TalonFXProHWCommand::getControlSlot(void) const
{
	return control_slot_;
}

void TalonFXProHWCommand::setControlDifferentialPosition(const double control_differential_position)
{
	if (fabs(control_differential_position - control_position_) > double_value_epsilon)
	{
		control_differential_position_ = control_differential_position;
		control_changed_ = true;
	}
}
double TalonFXProHWCommand::getControlDifferentialPosition(void) const
{
	return control_differential_position_;
}

void TalonFXProHWCommand::setControlDifferentialSlot(const int control_differential_slot)
{
	if ((control_differential_slot < 0) || (control_differential_slot > 2))
	{
		ROS_ERROR_STREAM("Invalid differential control slot (" << control_differential_slot << ") passed to " << __PRETTY_FUNCTION__);
		return;
	}
	if (control_differential_slot != control_differential_slot_)
	{
		control_differential_slot_ = control_differential_slot;
		control_changed_ = true;
	}
}
int TalonFXProHWCommand::getControlDifferentialSlot(void) const
{
	return control_differential_slot_;
}

void TalonFXProHWCommand::setControlOpposeMasterDirection(const bool control_oppose_master_direction)
{
	if (control_oppose_master_direction != control_oppose_master_direction_)
	{
		control_oppose_master_direction_ = control_oppose_master_direction;
		control_changed_ = true;
	}
}
bool TalonFXProHWCommand::getControlOpposeMasterDirection(void) const
{
	return control_oppose_master_direction_;
}

void TalonFXProHWCommand::setControlLimitForwardMotion(const bool control_limit_forward_motion)
{
	if (control_limit_forward_motion != control_limit_forward_motion_)
	{
		control_limit_forward_motion_ = control_limit_forward_motion;
		control_changed_ = true;
	}
}
bool TalonFXProHWCommand::getControlLimitForwardMotion(void) const
{
	return control_limit_forward_motion_;
}

void TalonFXProHWCommand::setControlLimitReverseMotion(const bool control_limit_reverse_motion)
{
	if (control_limit_reverse_motion != control_limit_reverse_motion_)
	{
		control_limit_reverse_motion_ = control_limit_reverse_motion;
		control_changed_ = true;
	}
}
bool TalonFXProHWCommand::getControlLimitReverseMotion(void) const
{
	return control_limit_reverse_motion_;
}

bool TalonFXProHWCommand::controlChanged(TalonMode &control_mode,
										 double &control_output,
										 double &control_position,
										 double &control_velocity,
										 double &control_acceleration,
										 double &control_jerk,
										 bool &control_enable_foc,
										 bool &control_override_brake_dur_neutral,
										 double &control_max_abs_duty_cycle,
										 double &control_deadband,
										 double &control_feedforward,
										 int &control_slot,
										 bool &control_limit_forward_motion,
										 bool &control_limit_reverse_motion,
										 double &control_differential_position,
										 int &control_differential_slot,
										 bool &control_oppose_master_direction) const
{
	control_mode = control_mode_;
	control_output = control_output_;
	control_position = control_position_;
	control_velocity = control_velocity_;
	control_acceleration = control_acceleration_;
	control_jerk = control_jerk_;
	control_enable_foc = control_enable_foc_;
	control_override_brake_dur_neutral = control_override_brake_dur_neutral_;
	control_max_abs_duty_cycle = control_max_abs_duty_cycle_;
	control_deadband = control_deadband_;
	control_feedforward = control_feedforward_;
	control_slot = control_slot_;
	control_limit_forward_motion = control_limit_forward_motion_;
	control_limit_reverse_motion = control_limit_reverse_motion_;
	control_differential_position = control_differential_position_;
	control_differential_slot = control_differential_slot_;
	control_oppose_master_direction = control_oppose_master_direction_;
	const auto ret = control_changed_;
	control_changed_ = false;
	return ret;
}
void TalonFXProHWCommand::resetControl(void)
{
	control_changed_ = true;
}

void TalonFXProHWCommand::setEnableReadThread(const bool enable_read_thread)
{
	enable_read_thread_ = enable_read_thread;
}
bool TalonFXProHWCommand::getEnableReadThread(void) const
{
	return enable_read_thread_;
}

} // namespace talonfxpro