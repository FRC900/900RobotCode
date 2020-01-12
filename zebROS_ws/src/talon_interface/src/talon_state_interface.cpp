#include "talon_interface/talon_state_interface.h"

namespace hardware_interface
{
TalonHWState::TalonHWState(int can_id) :
	setpoint_(0.),
	position_(0),
	speed_(0),
	output_voltage_(0),
	output_current_(0),
	bus_voltage_(0),
	motor_output_percent_(0),
	temperature_(0),
	pidf_p_ {0, 0, 0, 0},
	pidf_i_ {0, 0, 0, 0},
	pidf_d_ {0, 0, 0, 0},
	pidf_f_ {0, 0, 0, 0},
	pidf_izone_ {0, 0, 0, 0},
	allowable_closed_loop_error_ {0, 0, 0, 0},
	max_integral_accumulator_ {0, 0, 0, 0},
	closed_loop_peak_output_{1, 1, 1, 1},
	closed_loop_period_{1, 1, 1, 1},
	aux_pid_polarity_(false),
	closed_loop_error_(0.0),
	integral_accumulator_(0.0),
	error_derivative_(0.0),
	closed_loop_target_(0.0),
	p_term_(0.0),
	i_term_(0.0),
	d_term_(0.0),
	f_term_(0.0),
	active_trajectory_position_(0.0),
	active_trajectory_velocity_(0.0),
	active_trajectory_arbitrary_feed_forward_(0.0),
	active_trajectory_heading_(0.0),
	forward_limit_switch_closed_(false),
	reverse_limit_switch_closed_(false),
	forward_softlimit_hit_(false),
	reverse_softlimit_hit_(false),
	talon_mode_(TalonMode_Disabled),
	demand1_type_(DemandType_Neutral),
	demand1_value_(0),
	can_id_(can_id),
	slot_(0),
	invert_(false),
	sensor_phase_(false),
	neutral_mode_(NeutralMode_Coast),
	neutral_output_(false),
	encoder_feedback_(FeedbackDevice_Uninitialized),
	feedback_coefficient_(1.0),
	encoder_feedback_remote_(RemoteFeedbackDevice_None),
	encoder_ticks_per_rotation_(4096),
	remote_feedback_device_ids_{0, 0},
	remote_feedback_filters_{RemoteSensorSource_Off, RemoteSensorSource_Off},
	sensor_terms_{FeedbackDevice_QuadEncoder, FeedbackDevice_QuadEncoder, FeedbackDevice_QuadEncoder, FeedbackDevice_QuadEncoder},

	//output shaping
	close_loop_ramp_(0),
	open_loop_ramp_(0),
	peak_output_forward_(1.),
	peak_output_reverse_(-1.),
	nominal_output_forward_(0.),
	nominal_output_reverse_(0.),
	neutral_deadband_(41.0 / 1023.0),

	// voltage compensation
	voltage_compensation_saturation_(12),
	voltage_measurement_filter_(32),
	voltage_compensation_enable_(true),

	// velocity signal conditioning
	velocity_measurement_period_(Period_100Ms),
	velocity_measurement_window_(64),

	limit_switch_local_forward_source_(LimitSwitchSource_FeedbackConnector),
	limit_switch_local_forward_normal_(LimitSwitchNormal_Disabled),
	limit_switch_local_reverse_source_(LimitSwitchSource_FeedbackConnector),
	limit_switch_local_reverse_normal_(LimitSwitchNormal_Disabled),
	limit_switch_remote_forward_source_(RemoteLimitSwitchSource_Deactivated),
	limit_switch_remote_forward_normal_(LimitSwitchNormal_Disabled),
	limit_switch_remote_forward_id_(0),
	limit_switch_remote_reverse_source_(RemoteLimitSwitchSource_Deactivated),
	limit_switch_remote_reverse_normal_(LimitSwitchNormal_Disabled),
	limit_switch_remote_reverse_id_(0),

	// soft limits
	softlimit_forward_threshold_(0.0),
	softlimit_forward_enable_(false),
	softlimit_reverse_threshold_(0.0),
	softlimit_reverse_enable_(false),
	softlimits_override_enable_(false),

	// current limiting
	current_limit_peak_amps_(1),
	current_limit_peak_msec_(1),
	current_limit_continuous_amps_(1),
	current_limit_enable_(false),

	motion_cruise_velocity_(0),
	motion_acceleration_(0),
	motion_s_curve_strength_(0),

	// motion profiling
	motion_profile_top_level_buffer_count_(0),
	motion_profile_top_level_buffer_full_(false),
	motion_profile_trajectory_period_(0),

	// faults
	faults_(0),
	sticky_faults_(0),

	conversion_factor_(1.0),

	// control of read thread
	enable_read_thread_(true),

	// motor controller firmware version
	firmware_version_(-1)
{
	status_frame_periods_[Status_1_General] = status_1_general_default;
	status_frame_periods_[Status_2_Feedback0] = status_2_feedback0_default;
	status_frame_periods_[Status_3_Quadrature] = status_3_quadrature_default;
	status_frame_periods_[Status_4_AinTempVbat] = status_4_aintempvbat_default;
	status_frame_periods_[Status_6_Misc] = status_6_misc_default;
	status_frame_periods_[Status_7_CommStatus] = status_7_commstatus_default;
	status_frame_periods_[Status_8_PulseWidth] = status_8_pulsewidth_default;
	status_frame_periods_[Status_9_MotProfBuffer] = status_9_motprofbuffer_default;
	status_frame_periods_[Status_10_MotionMagic] = status_10_motionmagic_default;
	status_frame_periods_[Status_11_UartGadgeteer] = status_11_uartgadgeteer_default;
	status_frame_periods_[Status_12_Feedback1] = status_12_feedback1_default;
	status_frame_periods_[Status_13_Base_PIDF0] = status_13_base_pidf0_default;
	status_frame_periods_[Status_14_Turn_PIDF1] = status_14_turn_pidf1_default;
	status_frame_periods_[Status_15_FirmwareApiStatus] = status_15_firmwareapistatus_default;

	control_frame_periods_[Control_3_General] = control_3_general_default;
	control_frame_periods_[Control_4_Advanced] = control_4_advanced_default;
	control_frame_periods_[Control_5_FeedbackOutputOverride] = control_5_feedbackoutputoverride_default;
	control_frame_periods_[Control_6_MotProfAddTrajPoint] = control_6_motprofaddtrajpoint_default;
}

TalonHWState::~TalonHWState()
{
}

double TalonHWState::getSetpoint(void) const
{
	return setpoint_;
}
double TalonHWState::getPosition(void) const
{
	return position_;
}
double TalonHWState::getSpeed(void) const
{
	return speed_;
}
double TalonHWState::getOutputVoltage(void) const
{
	return output_voltage_;
}
int    TalonHWState::getCANID(void) const
{
	return can_id_;
}
double TalonHWState::getOutputCurrent(void) const
{
	return output_current_;
}
double TalonHWState::getBusVoltage(void) const
{
	return bus_voltage_;
}
double TalonHWState::getMotorOutputPercent(void) const
{
	return motor_output_percent_;
}
double TalonHWState::getTemperature(void) const
{
	return temperature_;
}
double TalonHWState::getPidfP(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return pidf_p_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getPidfP. Must be < TALON_PIDF_SLOTS.");
		return 0.0;
	}
}
double TalonHWState::getPidfI(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return pidf_i_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getPidfI. Must be < TALON_PIDF_SLOTS.");
		return 0.0;
	}
}
double TalonHWState::getPidfD(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return pidf_d_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getPidfD. Must be < TALON_PIDF_SLOTS.");
		return 0.0;
	}
}
double TalonHWState::getPidfF(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return pidf_f_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getPidfF. Must be < TALON_PIDF_SLOTS.");
		return 0.0;
	}
}
int TalonHWState::getPidfIzone(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return pidf_izone_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getPidfIzone. Must be < TALON_PIDF_SLOTS.");
		return 0;
	}
}
int TalonHWState::getAllowableClosedLoopError(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return allowable_closed_loop_error_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getAllowableClosedLoopError. Must be < TALON_PIDF_SLOTS.");
		return 0;
	}
}
double TalonHWState::getMaxIntegralAccumulator(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return max_integral_accumulator_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getMaxIntegralAccumulator. Must be < TALON_PIDF_SLOTS.");
		return 0;
	}
}
double TalonHWState::getClosedLoopPeakOutput(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return closed_loop_peak_output_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getClosedLoopPeakOutput. Must be < TALON_PIDF_SLOTS.");
		return 0;
	}
}
int TalonHWState::getClosedLoopPeriod(size_t index) const
{
	if (index < TALON_PIDF_SLOTS)
		return closed_loop_period_[index];
	else
	{
		ROS_WARN_STREAM("Invalid index in talon_state getClosedLoopPeriod. Must be < TALON_PIDF_SLOTS.");
		return 0;
	}
}
bool TalonHWState::getAuxPidPolarity(void) const
{
	return aux_pid_polarity_;
}

double TalonHWState::getClosedLoopError(void) const
{
	return closed_loop_error_;
}
double TalonHWState::getIntegralAccumulator(void) const
{
	return integral_accumulator_;
}
double TalonHWState::getErrorDerivative(void) const
{
	return error_derivative_;
}
double TalonHWState::getClosedLoopTarget(void) const
{
	return closed_loop_target_;
}
double TalonHWState::getPTerm(void) const
{
	return p_term_;
}
double TalonHWState::getITerm(void) const
{
	return i_term_;
}
double TalonHWState::getDTerm(void) const
{
	return d_term_;
}
double TalonHWState::getFTerm(void) const
{
	return f_term_;
}
double TalonHWState::getActiveTrajectoryPosition(void) const
{
	return active_trajectory_position_;
}
double TalonHWState::getActiveTrajectoryVelocity(void) const
{
	return active_trajectory_velocity_;
}
double TalonHWState::getActiveTrajectoryArbitraryFeedForward(void) const
{
	return active_trajectory_arbitrary_feed_forward_;
}
double TalonHWState::getActiveTrajectoryHeading(void) const
{
	return active_trajectory_heading_;
}
bool TalonHWState::getForwardLimitSwitch(void) const
{
	return forward_limit_switch_closed_;
}
bool TalonHWState::getReverseLimitSwitch(void) const
{
	return reverse_limit_switch_closed_;
}
bool TalonHWState::getForwardSoftlimitHit(void) const
{
	return forward_softlimit_hit_;
}
bool TalonHWState::getReverseSoftlimitHit(void) const
{
	return reverse_softlimit_hit_;
}

TalonMode TalonHWState::getTalonMode(void) const
{
	return talon_mode_;
}
DemandType TalonHWState::getDemand1Type(void) const
{
	return demand1_type_;
}
double TalonHWState::getDemand1Value(void) const
{
	return demand1_value_;
}
int  TalonHWState::getSlot(void) const
{
	return slot_;
}
bool TalonHWState::getInvert(void) const
{
	return invert_;
}
bool TalonHWState::getSensorPhase(void) const
{
	return sensor_phase_;
}
NeutralMode TalonHWState::getNeutralMode(void) const
{
	return neutral_mode_;
}
bool TalonHWState::getNeutralOutput(void) const
{
	return neutral_output_;
}
FeedbackDevice TalonHWState::getEncoderFeedback(void) const
{
	return encoder_feedback_;
}
RemoteFeedbackDevice TalonHWState::getRemoteEncoderFeedback(void) const
{
	return encoder_feedback_remote_;
}
double TalonHWState::getFeedbackCoefficient(void) const
{
	return feedback_coefficient_;
}
int TalonHWState::getRemoteFeedbackDeviceId(size_t remote_ordinal) const
{
	if (remote_ordinal >= 2)
	{
		ROS_WARN("getRemoteFeedbackDeviceId: remote_ordinal too large");
		return -1;
	}
	return remote_feedback_device_ids_[remote_ordinal];
}
RemoteSensorSource TalonHWState::getRemoteFeedbackFilter(size_t remote_ordinal) const
{
	if (remote_ordinal >= 2)
	{
		ROS_WARN("getRemoteFeedbackFilter : remote_ordinal too large");
		return RemoteSensorSource_Last;
	}
	return remote_feedback_filters_[remote_ordinal];
}
FeedbackDevice TalonHWState::getSensorTerm(SensorTerm sensor_term) const
{
	if (sensor_term < SensorTerm_Last)
		return sensor_terms_[sensor_term];
	ROS_WARN("getSensorTerm : sensor_term index too large");
	return FeedbackDevice_Last;
}
int TalonHWState::getEncoderTicksPerRotation(void) const
{
	return encoder_ticks_per_rotation_;
}

unsigned int TalonHWState::getFaults(void) const
{
	return faults_;
}
unsigned int TalonHWState::getStickyFaults(void) const
{
	return sticky_faults_;
}
double TalonHWState::getConversionFactor(void) const
{
	return conversion_factor_;
}
bool TalonHWState::getEnableReadThread(void) const
{
	return enable_read_thread_;
}
void TalonHWState::setSetpoint(double setpoint)
{
	setpoint_ = setpoint;
}
void TalonHWState::setPosition(double position)
{
	position_ = position;
}
void TalonHWState::setSpeed(double speed)
{
	speed_ = speed;
}
void TalonHWState::setOutputVoltage(double output_voltage)
{
	output_voltage_ = output_voltage;
}
void TalonHWState::setOutputCurrent(double output_current)
{
	output_current_ = output_current;
}
void TalonHWState::setBusVoltage(double bus_voltage)
{
	bus_voltage_ = bus_voltage;
}
void TalonHWState::setMotorOutputPercent(double motor_output_percent)
{
	motor_output_percent_ = motor_output_percent;
}
void TalonHWState::setTemperature(double temperature)
{
	temperature_ = temperature;
}

//output shaping
void TalonHWState::setClosedloopRamp(double close_loop_ramp)
{
	close_loop_ramp_ = close_loop_ramp;
}
double TalonHWState::getClosedloopRamp(void) const
{
	return close_loop_ramp_;
}

void TalonHWState::setOpenloopRamp(double open_loop_ramp)
{
	open_loop_ramp_ = open_loop_ramp;
}
double TalonHWState::getOpenloopRamp(void) const
{
	return open_loop_ramp_;
}

void TalonHWState::setPeakOutputForward(double peak_output_forward)
{
	peak_output_forward_ = peak_output_forward;
}
double TalonHWState::getPeakOutputForward(void) const
{
	return peak_output_forward_;
}
void TalonHWState::setPeakOutputReverse(double peak_output_reverse)
{
	peak_output_reverse_ = peak_output_reverse;
}
double TalonHWState::getPeakOutputReverse(void) const
{
	return peak_output_reverse_;
}

void TalonHWState::setNominalOutputForward(double nominal_output_forward)
{
	nominal_output_forward_ = nominal_output_forward;
}
double TalonHWState::getNominalOutputForward(void) const
{
	return nominal_output_forward_;
}
void TalonHWState::setNominalOutputReverse(double nominal_output_reverse)
{
	nominal_output_reverse_ = nominal_output_reverse;
}
double TalonHWState::getNominalOutputReverse(void) const
{
	return nominal_output_reverse_;
}

void TalonHWState::setNeutralDeadband(double neutral_deadband)
{
	neutral_deadband_ = neutral_deadband;
}
double TalonHWState::getNeutralDeadband(void) const
{
	return neutral_deadband_;
}

void TalonHWState::setVoltageCompensationSaturation(double voltage_compensation_saturation)
{
	voltage_compensation_saturation_ = voltage_compensation_saturation;
}
double TalonHWState::getVoltageCompensationSaturation(void) const
{
	return voltage_compensation_saturation_;
}

void TalonHWState::setVoltageMeasurementFilter(int voltage_measurement_filter)
{
	voltage_measurement_filter_ = voltage_measurement_filter;
}
int TalonHWState::getVoltageMeasurementFilter(void) const
{
	return voltage_measurement_filter_;
}

void TalonHWState::setVoltageCompensationEnable(bool voltage_compensation_enable)
{
	voltage_compensation_enable_ = voltage_compensation_enable;
}
bool TalonHWState::getVoltageCompensationEnable(void) const
{
	return voltage_compensation_enable_;
}
void TalonHWState::setVelocityMeasurementPeriod(hardware_interface::VelocityMeasurementPeriod period)
{
	velocity_measurement_period_ = period;
}

bool TalonHWState::getVelocityMeasurementPeriod(void) const
{
	return velocity_measurement_period_;
}

void TalonHWState::setVelocityMeasurementWindow(int window)
{
	velocity_measurement_window_ = window;
}

bool TalonHWState::getVelocityMeasurementWindow(void) const
{
	return velocity_measurement_window_;
}

void TalonHWState::setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
{
	limit_switch_local_forward_source_ = source;
	limit_switch_local_forward_normal_ = normal;
}

void TalonHWState::getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
{
	source = limit_switch_local_forward_source_;
	normal = limit_switch_local_forward_normal_;
}

void TalonHWState::setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
{
	limit_switch_local_reverse_source_ = source;
	limit_switch_local_reverse_normal_ = normal;
}

void TalonHWState::getReverseLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
{
	source = limit_switch_local_reverse_source_;
	normal = limit_switch_local_reverse_normal_;
}

void TalonHWState::setRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
{
	limit_switch_remote_forward_source_ = source;
	limit_switch_remote_forward_normal_ = normal;
	limit_switch_remote_forward_id_     = id;
}

void TalonHWState::getRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
{
	source = limit_switch_remote_forward_source_;
	normal = limit_switch_remote_forward_normal_;
	id     = limit_switch_remote_forward_id_;
}

void TalonHWState::setRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
{
	limit_switch_remote_reverse_source_ = source;
	limit_switch_remote_reverse_normal_ = normal;
	limit_switch_remote_reverse_id_     = id;
}

void TalonHWState::getRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
{
	source = limit_switch_remote_reverse_source_;
	normal = limit_switch_remote_reverse_normal_;
	id     = limit_switch_remote_reverse_id_;
}

void TalonHWState::setForwardSoftLimitThreshold(double threshold)
{
	softlimit_forward_threshold_ = threshold;
}
double TalonHWState::getForwardSoftLimitThreshold(void) const
{
	return softlimit_forward_threshold_;
}

void TalonHWState::setForwardSoftLimitEnable(bool enable)
{
	softlimit_forward_enable_ = enable;
}
bool TalonHWState::getForwardSoftLimitEnable(void) const
{
	return softlimit_forward_enable_;
}
void TalonHWState::setReverseSoftLimitThreshold(double threshold)
{
	softlimit_reverse_threshold_ = threshold;
}
double TalonHWState::getReverseSoftLimitThreshold(void) const
{
	return softlimit_reverse_threshold_;
}

void TalonHWState::setReverseSoftLimitEnable(bool enable)
{
	softlimit_reverse_enable_ = enable;
}
bool TalonHWState::getReverseSoftLimitEnable(void) const
{
	return softlimit_reverse_enable_;
}

void TalonHWState::setOverrideSoftLimitsEnable(bool enable)
{
	softlimits_override_enable_ = enable;
}
bool TalonHWState::getOverrideSoftLimitsEnable(void) const
{
	return softlimits_override_enable_;
}

void TalonHWState::setPeakCurrentLimit(int amps)
{
	current_limit_peak_amps_ = amps;
}
int TalonHWState::getPeakCurrentLimit(void) const
{
	return current_limit_peak_amps_;
}

void TalonHWState::setPeakCurrentDuration(int msec)
{
	current_limit_peak_msec_ = msec;
}
int TalonHWState::getPeakCurrentDuration(void) const
{
	return current_limit_peak_msec_;
}
void TalonHWState::setContinuousCurrentLimit(int amps)
{
	current_limit_continuous_amps_ = amps;
}
int TalonHWState::getContinuousCurrentLimit(void) const
{
	return current_limit_continuous_amps_;
}
void TalonHWState::setCurrentLimitEnable(bool enable)
{
	current_limit_enable_ = enable;
}
bool TalonHWState::getCurrentLimitEnable(void) const
{
	return current_limit_enable_;
}

void TalonHWState::setMotionCruiseVelocity(double velocity)
{
	motion_cruise_velocity_ = velocity;
}
double TalonHWState::getMotionCruiseVelocity(void) const
{
	return motion_cruise_velocity_;
}
void TalonHWState::setMotionAcceleration(double acceleration)
{
	motion_acceleration_ = acceleration;
}
double TalonHWState::getMotionAcceleration(void) const
{
	return motion_acceleration_;
}

void TalonHWState::setMotionSCurveStrength(unsigned int s_curve_strength)
{
	if (s_curve_strength > 8)
	{
		ROS_ERROR("setMotionSCurveStrength out of range");
		return;
	}

	motion_s_curve_strength_ = s_curve_strength;
}
unsigned int TalonHWState::getMotionSCurveStrength(void) const
{
	return motion_s_curve_strength_;
}

void TalonHWState::setMotionProfileTopLevelBufferCount(int count)
{
	motion_profile_top_level_buffer_count_ = count;
}
int TalonHWState::getMotionProfileTopLevelBufferCount(void) const
{
	return motion_profile_top_level_buffer_count_;
}
void TalonHWState::setMotionProfileTopLevelBufferFull(bool is_full)
{
	motion_profile_top_level_buffer_full_ = is_full;
}
bool TalonHWState::getMotionProfileTopLevelBufferFull(void) const
{
	return motion_profile_top_level_buffer_full_;
}
void TalonHWState::setMotionProfileStatus(const MotionProfileStatus &status)
{
	motion_profile_status_ = status;
}
MotionProfileStatus TalonHWState::getMotionProfileStatus(void) const
{
	return motion_profile_status_;
}

void TalonHWState::setStatusFramePeriod(StatusFrame status_frame, uint8_t period)
{
	if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
		status_frame_periods_[status_frame] = period;
	else
		ROS_ERROR("Invalid status_frame value passed to TalonHWState::setStatusFramePeriod()");
}

uint8_t TalonHWState::getStatusFramePeriod(StatusFrame status_frame) const
{
	if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
		return status_frame_periods_[status_frame];

	ROS_ERROR("Invalid status_frame value passed to TalonHWState::setStatusFramePeriod()");
	return 0;
}

void TalonHWState::setControlFramePeriod(ControlFrame control_frame, uint8_t period)
{
	if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
		control_frame_periods_[control_frame] = period;
	else
		ROS_ERROR("Invalid control_frame value passed to TalonHWState::setControlFramePeriod()");
}

uint8_t TalonHWState::getControlFramePeriod(ControlFrame control_frame) const
{
	if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
		return control_frame_periods_[control_frame];

	ROS_ERROR("Invalid control_frame value passed to TalonHWState::setControlFramePeriod()");
	return 0;
}

void TalonHWState::setMotionProfileTrajectoryPeriod(int msec)
{
	motion_profile_trajectory_period_ = msec;
}
int TalonHWState::getMotionProfileTrajectoryPeriod(void) const
{
	return motion_profile_trajectory_period_;
}
CustomProfileStatus TalonHWState::getCustomProfileStatus(void) const
{
	return custom_profile_status_;
}
void TalonHWState::setCustomProfileStatus(const CustomProfileStatus &status)
{
	custom_profile_status_ = status;
}
void TalonHWState::setPidfP(double pidf_p, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		pidf_p_[index] = pidf_p;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setPidfP. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setPidfI(double pidf_i, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		pidf_i_[index] = pidf_i;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setPidfI. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setPidfD(double pidf_d, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		pidf_d_[index] = pidf_d;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setPidfD. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setPidfF(double pidf_f, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		pidf_f_[index] = pidf_f;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setPidfF. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setPidfIzone(int pidf_izone, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		pidf_izone_[index] = pidf_izone;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setPidfIzone. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setAllowableClosedLoopError(int allowable_closed_loop_error, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		allowable_closed_loop_error_[index] = allowable_closed_loop_error;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setAllowableClosedLoopError. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setMaxIntegralAccumulator(double max_integral_accumulator, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		max_integral_accumulator_[index] = max_integral_accumulator;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setMaxIntegralAccumulator. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		closed_loop_peak_output_[index] = closed_loop_peak_output;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setClosedLoopPeakOutput. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setClosedLoopPeriod(double closed_loop_period, size_t index)
{
	if (index < TALON_PIDF_SLOTS)
		closed_loop_period_[index] = closed_loop_period;
	else
		ROS_WARN_STREAM("Invalid index in talon_state setClosedLoopPeriod. Must be < TALON_PIDF_SLOTS.");
}
void TalonHWState::setAuxPidPolarity(bool aux_pid_polarity)
{
	aux_pid_polarity_ = aux_pid_polarity;
}

void TalonHWState::setClosedLoopError(double closed_loop_error)
{
	closed_loop_error_ = closed_loop_error;
}
void TalonHWState::setIntegralAccumulator(double integral_accumulator)
{
	integral_accumulator_ = integral_accumulator;
}
void TalonHWState::setErrorDerivative(double error_derivative)
{
	error_derivative_ = error_derivative;
}
void TalonHWState::setClosedLoopTarget(double closed_loop_target)
{
	closed_loop_target_ = closed_loop_target;
}
void TalonHWState::setPTerm(double p_term)
{
	p_term_ = p_term;
}
void TalonHWState::setITerm(double i_term)
{
	i_term_ = i_term;
}
void TalonHWState::setDTerm(double d_term)
{
	d_term_ = d_term;
}
void TalonHWState::setFTerm(double f_term)
{
	f_term_ = f_term;
}
void TalonHWState::setActiveTrajectoryPosition(double active_trajectory_position)
{
	active_trajectory_position_ = active_trajectory_position;
}
void TalonHWState::setActiveTrajectoryVelocity(double active_trajectory_velocity)
{
	active_trajectory_velocity_ = active_trajectory_velocity;
}
void TalonHWState::setActiveTrajectoryArbitraryFeedForward(double active_trajectory_arbitrary_feed_forward)
{
	active_trajectory_arbitrary_feed_forward_ = active_trajectory_arbitrary_feed_forward;
}
void TalonHWState::setActiveTrajectoryHeading(double active_trajectory_heading)
{
	active_trajectory_heading_ = active_trajectory_heading;
}
void TalonHWState::setForwardLimitSwitch(bool forward_limit_switch_closed)
{
	forward_limit_switch_closed_ = forward_limit_switch_closed;
}
void TalonHWState::setReverseLimitSwitch(bool reverse_limit_switch_closed)
{
	reverse_limit_switch_closed_ = reverse_limit_switch_closed;
}
void TalonHWState::setForwardSoftlimitHit(bool forward_softlimit_hit)
{
	forward_softlimit_hit_ = forward_softlimit_hit;
}
void TalonHWState::setReverseSoftlimitHit(bool reverse_softlimit_hit)
{
	reverse_softlimit_hit_ = reverse_softlimit_hit;
}

void TalonHWState::setTalonMode(TalonMode talon_mode)
{
	if ((talon_mode_ > TalonMode_First) &&
		(talon_mode_ < TalonMode_Last) )
		talon_mode_ = talon_mode;
	else
		ROS_WARN("Invalid talon mode requested");
}
void TalonHWState::setDemand1Type(DemandType demand_type)
{
	if ((demand_type >= DemandType_Neutral) &&
		(demand_type < DemandType_Last))
		demand1_type_ = demand_type;
	else
		ROS_WARN("Invalid demand 1 type requested");
}
void TalonHWState::setDemand1Value(double value)
{
	demand1_value_ = value;
}
void TalonHWState::setSlot(int slot)
{
	slot_ = slot;
}
void TalonHWState::setInvert(bool invert)
{
	invert_ = invert;
}
void TalonHWState::setSensorPhase(bool sensor_phase)
{
	sensor_phase_ = sensor_phase;
}
void TalonHWState::setNeutralMode(NeutralMode neutral_mode)
{
	if ((neutral_mode_ >= NeutralMode_Uninitialized) &&
		(neutral_mode_ <  NeutralMode_Last) )
		neutral_mode_ = neutral_mode;
	else
		ROS_WARN_STREAM("Invalid neutral mode requested");
}
void TalonHWState::setNeutralOutput(bool neutral_output)
{
	neutral_output_ = neutral_output;
}

void TalonHWState::setEncoderFeedback(FeedbackDevice encoder_feedback)
{
	if ((encoder_feedback >= FeedbackDevice_Uninitialized) &&
		(encoder_feedback <  FeedbackDevice_Last) )
		encoder_feedback_ = encoder_feedback;
	else
		ROS_WARN_STREAM("Invalid feedback device requested");
}
void TalonHWState::setRemoteEncoderFeedback(RemoteFeedbackDevice encoder_feedback_remote)
{
	if ((encoder_feedback_remote >= RemoteFeedbackDevice_SensorSum) &&
		(encoder_feedback_remote <  RemoteFeedbackDevice_Last) )
		encoder_feedback_remote_ = encoder_feedback_remote;
	else
		ROS_WARN_STREAM("Invalid remote feedback device requested");
}
void TalonHWState::setFeedbackCoefficient(double feedback_coefficient)
{
	feedback_coefficient_ = feedback_coefficient;
}
void TalonHWState::setRemoteFeedbackDeviceId(int device_id, size_t remote_ordinal)
{
	if (remote_ordinal >= 2)
	{
		ROS_WARN("setRemoteFeedbackDeviceId : remote_ordinal too large");
		return;
	}
	remote_feedback_device_ids_[remote_ordinal] = device_id;
}
void TalonHWState::setRemoteFeedbackDeviceIds(const std::array<int, 2> &remote_feedback_device_ids)
{
	remote_feedback_device_ids_ = remote_feedback_device_ids;
}
void TalonHWState::setRemoteFeedbackFilter(RemoteSensorSource remote_sensor_source, size_t remote_ordinal)
{
	if (remote_ordinal >= 2)
	{
		ROS_WARN("setRemoteFeedbackFilter : remote_ordinal too large");
		return;
	}
	if ((remote_sensor_source <  RemoteSensorSource_Off) ||
		(remote_sensor_source >= RemoteSensorSource_Last))
	{
		ROS_WARN("setRemoteFeedbackFilter : remote_sensor_source out of range");
		return;
	}
	remote_feedback_filters_[remote_ordinal] = remote_sensor_source;
}
void TalonHWState::setRemoteFeedbackFilters(const std::array<RemoteSensorSource, 2> &remote_sensor_sources)
{
	remote_feedback_filters_ = remote_sensor_sources;
}
void TalonHWState::setSensorTerm(FeedbackDevice feedback_device, SensorTerm sensor_term)
{
	if (sensor_term >= SensorTerm_Last)
	{
		ROS_WARN_STREAM("setSensorTerm Invalid sensor term index requested");
		return;
	}

	if ((feedback_device <  FeedbackDevice_Uninitialized) ||
		(feedback_device >= FeedbackDevice_Last) )
	{
		ROS_WARN_STREAM("setSensorTerm Invalid feedback device requested");
		return;
	}
	sensor_terms_[sensor_term] = feedback_device;
}
void TalonHWState::setSensorTerms(const std::array<FeedbackDevice, SensorTerm_Last> &sensor_terms)
{
	sensor_terms_ = sensor_terms;
}
void TalonHWState::setEncoderTicksPerRotation(int encoder_ticks_per_rotation)
{
	encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
}
void TalonHWState::setFaults(unsigned int faults)
{
	faults_ = faults;
}
void TalonHWState::setStickyFaults(unsigned int sticky_faults)
{
	sticky_faults_ = sticky_faults;
}
void TalonHWState::setConversionFactor(double conversion_factor)
{
	conversion_factor_ = conversion_factor;
}
void TalonHWState::setEnableReadThread(bool enable_read_thread)
{
	enable_read_thread_ = enable_read_thread;
}

void TalonHWState::setFirmwareVersion(int firmware_version)
{
	firmware_version_ = firmware_version;
}

int TalonHWState::getFirmwareVersion(void) const
{
	return firmware_version_;
}

} // namespace
