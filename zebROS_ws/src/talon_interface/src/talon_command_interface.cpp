#include "talon_interface/talon_command_interface.h"

namespace hardware_interface
{

CustomProfilePoint::CustomProfilePoint(void) :
	mode(TalonMode_Position),
	pidSlot(0),
	setpoint(0.0),
	fTerm(0),
	duration(0),
	zeroPos(false)
{
}

// Sane? defaults
TrajectoryPoint::TrajectoryPoint(void)
	: position(0)
	, velocity(0)
	, headingRad(0)
	, arbFeedFwd(0)
	, auxiliaryPos(0)
	, auxiliaryVel(0)
	, auxiliaryArbFeedFwd(0)
	, profileSlotSelect0(0)
	, profileSlotSelect1(0)
	, isLastPoint(false)
	, zeroPos(false)
	, timeDur(0)
	, useAuxPID(false)
{
}

// Set up default values
// Set most of the changed_ vars to true
// to force a write of these values to the Talon
// That should put the talon in a known state
// rather than relying on them being setup to
// a certain state previously
TalonHWCommand::TalonHWCommand(void) :
	command_(0),
	command_changed_(true),
	mode_(TalonMode_Disabled),
	mode_changed_(false),
	demand1_type_(DemandType_Neutral),
	demand1_value_(0.0),
	demand1_changed_(true),
	pidf_slot_(0),
	pidf_slot_changed_(true),
	iaccum_(0.0),
	iaccum_changed_(false),
	invert_(false),
	sensor_phase_(false),
	invert_changed_(true),
	neutral_mode_(NeutralMode_Uninitialized),
	neutral_mode_changed_(false),
	neutral_output_(false),
	encoder_feedback_(FeedbackDevice_Uninitialized),
	feedback_coefficient_(1.0),
	encoder_feedback_changed_(false),
	remote_encoder_feedback_(RemoteFeedbackDevice_None),
	remote_encoder_feedback_changed_(false),
	encoder_ticks_per_rotation_(4096),
	remote_feedback_device_ids_{0, 0},
	remote_feedback_filters_{RemoteSensorSource_Off, RemoteSensorSource_Off},
	remote_feedback_filters_changed_(false),
	sensor_terms_{FeedbackDevice_QuadEncoder, FeedbackDevice_QuadEncoder, FeedbackDevice_QuadEncoder, FeedbackDevice_QuadEncoder},
	sensor_terms_changed_(false),

	//output shaping
	closed_loop_ramp_(0),
	open_loop_ramp_(0),
	peak_output_forward_(1.),
	peak_output_reverse_(-1.),
	nominal_output_forward_(0.),
	nominal_output_reverse_(0.),
	neutral_deadband_(41. / 1023.),
	output_shaping_changed_(true),

	// voltage compensation
	voltage_compensation_saturation_(12), //max voltage to apply to talons when command is 100%
	voltage_measurement_filter_(32), //number of samples in the average of voltage measurements
	voltage_compensation_enable_(true),
	voltage_compensation_changed_(true),

	velocity_measurement_period_(Period_100Ms),
	velocity_measurement_window_(64),
	velocity_measurement_changed_(true),

	sensor_position_value_(0.),
	sensor_position_changed_(false),

	// limit switches
	limit_switch_local_forward_source_(LimitSwitchSource_FeedbackConnector),
	limit_switch_local_forward_normal_(LimitSwitchNormal_Disabled),
	limit_switch_local_reverse_source_(LimitSwitchSource_FeedbackConnector),
	limit_switch_local_reverse_normal_(LimitSwitchNormal_Disabled),
	limit_switch_local_changed_(true),
	limit_switch_remote_forward_source_(RemoteLimitSwitchSource_Deactivated),
	limit_switch_remote_forward_normal_(LimitSwitchNormal_Disabled),
	limit_switch_remote_forward_id_(0),
	limit_switch_remote_reverse_source_(RemoteLimitSwitchSource_Deactivated),
	limit_switch_remote_reverse_normal_(LimitSwitchNormal_Disabled),
	limit_switch_remote_reverse_id_(0),
	limit_switch_remote_changed_(true),

	// soft limits
	softlimit_forward_threshold_(0.0),
	softlimit_forward_enable_(false),
	softlimit_reverse_threshold_(0.0),
	softlimit_reverse_enable_(false),
	softlimits_override_enable_(true),
	softlimit_changed_(true),

	// current limiting - TalonSRX only
	current_limit_peak_amps_(0),
	current_limit_peak_msec_(10), // see https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/README.md#motor-output-direction-is-incorrect-or-accelerates-when-current-limit-is-enabled
	current_limit_continuous_amps_(0),
	current_limit_enable_(false),
	current_limit_changed_(true),

	// current limiting - TalonFX / Falcon500
	supply_current_limit_(0),
	supply_current_trigger_threshold_current_(0),
	supply_current_trigger_threshold_time_(0),
	supply_current_limit_enable_(false),
	supply_current_limit_changed_(true),

	stator_current_limit_(0),
	stator_current_trigger_threshold_current_(0),
	stator_current_trigger_threshold_time_(0),
	stator_current_limit_enable_(false),
	stator_current_limit_changed_(true),

	motion_cruise_velocity_(0),
	motion_acceleration_(0),
	motion_s_curve_strength_(0),
	motion_cruise_changed_(true),

	motion_profile_clear_trajectories_(false),
	motion_profile_clear_has_underrun_(false),
	motion_profile_profile_trajectory_period_(0),
	motion_profile_profile_trajectory_period_changed_(true),

	clear_sticky_faults_(false),
	p_{0, 0, 0, 0},
	i_{0, 0, 0, 0},
	d_{0, 0, 0, 0},
	f_{0, 0, 0, 0},
	i_zone_{0, 0, 0, 0},
	allowable_closed_loop_error_{0, 0, 0, 0}, // need better defaults
	max_integral_accumulator_{0, 0, 0, 0},
	closed_loop_peak_output_{1, 1, 1, 1},
	closed_loop_period_{1, 1, 1, 1},
	pidf_changed_{true, true, true, true},
	aux_pid_polarity_(false),
	aux_pid_polarity_changed_(true),

	conversion_factor_(1.0),
	conversion_factor_changed_(true),

	motor_commutation_(hardware_interface::MotorCommutation::Trapezoidal),
	motor_commutation_changed_(true),

	absolute_sensor_range_(hardware_interface::Unsigned_0_to_360),
	absolute_sensor_range_changed_(true),

	sensor_initialization_strategy_(hardware_interface::BootToZero),
	sensor_initialization_strategy_changed_(true),

	custom_profile_disable_(false),
	custom_profile_run_(false),
	custom_profile_slot_(0),
	custom_profile_hz_(50.0),

	enable_read_thread_(true),
	enable_read_thread_changed_(false),
	mutex_(std::make_shared<std::mutex>())
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

	status_frame_periods_changed_[Status_1_General] = true;
	status_frame_periods_changed_[Status_2_Feedback0] = true;
	status_frame_periods_changed_[Status_3_Quadrature] = true;
	status_frame_periods_changed_[Status_4_AinTempVbat] = true;
	status_frame_periods_changed_[Status_6_Misc] = true;
	status_frame_periods_changed_[Status_7_CommStatus] = true;
	status_frame_periods_changed_[Status_8_PulseWidth] = true;
	status_frame_periods_changed_[Status_9_MotProfBuffer] = true;
	status_frame_periods_changed_[Status_10_MotionMagic] = true;
	status_frame_periods_changed_[Status_11_UartGadgeteer] = true;
	status_frame_periods_changed_[Status_12_Feedback1] = true;
	status_frame_periods_changed_[Status_14_Turn_PIDF1] = true;
	status_frame_periods_changed_[Status_15_FirmwareApiStatus] = true;

	control_frame_periods_[Control_3_General] = control_3_general_default;
	control_frame_periods_[Control_4_Advanced] = control_4_advanced_default;
	control_frame_periods_[Control_5_FeedbackOutputOverride] = control_5_feedbackoutputoverride_default;
	control_frame_periods_[Control_6_MotProfAddTrajPoint] = control_6_motprofaddtrajpoint_default;

	control_frame_periods_changed_[Control_3_General] = false;
	control_frame_periods_changed_[Control_4_Advanced] = false;
	control_frame_periods_changed_[Control_5_FeedbackOutputOverride] = false;
	control_frame_periods_changed_[Control_6_MotProfAddTrajPoint] = false;
}

TalonHWCommand::~TalonHWCommand()
{
}

// This gets the requested setpoint, not the
// status actually read from the controller
// Need to think about which makes the most
// sense to query...
bool TalonHWCommand::commandChanged(double &command)
{
	command = command_;
	if (!command_changed_)
		return false;
	command_changed_ = false;
	return true;
}
double TalonHWCommand::get(void) const
{
	return command_;
}

TalonMode TalonHWCommand::getMode(void) const
{
	return mode_;
}

void TalonHWCommand::setP(double oldP, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setP()");
		return;
	}
	if (fabs(oldP - p_[index]) > double_value_epsilon)
	{
		pidf_changed_[index] = true;
		p_[index] = oldP;
	}
}
double TalonHWCommand::getP(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getP()");
		return 0.0;
	}
	return p_[index];
}

void TalonHWCommand::setI(double ii, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setI()");
		return;
	}
	if (fabs(ii - i_[index]) > double_value_epsilon)
	{
		pidf_changed_[index] = true;
		i_[index] = ii;
	}
}
double TalonHWCommand::getI(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getI()");
		return 0.0;
	}
	return i_[index];
}

void TalonHWCommand::setD(double dd, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setD()");
		return;
	}
	if (fabs(dd - d_[index]) > double_value_epsilon)
	{
		pidf_changed_[index] = true;
		d_[index] = dd;
	}
}
double TalonHWCommand::getD(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getD()");
		return 0.0;
	}
	return d_[index];
}

void TalonHWCommand::setF(double ff, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setF()");
		return;
	}
	if (fabs(ff - f_[index]) > double_value_epsilon)
	{
		pidf_changed_[index] = true;
		f_[index] = ff;
	}
}
double TalonHWCommand::getF(size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getF()");
		return 0.0;
	}
	return f_[index];
}

void TalonHWCommand::setIZ(int i_zone, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setIZ()");
		return;
	}
	if (i_zone != i_zone_[index])
	{
		pidf_changed_[index] = true;
		i_zone_[index] = i_zone;
	}
}
int TalonHWCommand::getIZ(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getIZ()");
		return 0.0;
	}
	return i_zone_[index];
}

void TalonHWCommand::setAllowableClosedloopError(int allowable_closed_loop_error, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
		return;
	}
	if (allowable_closed_loop_error != allowable_closed_loop_error_[index])
	{
		pidf_changed_[index] = true;
		allowable_closed_loop_error_[index] = allowable_closed_loop_error;
	}
}
int TalonHWCommand::getAllowableClosedloopError(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
		return 0;
	}
	return allowable_closed_loop_error_[index];
}
void TalonHWCommand::setMaxIntegralAccumulator(int max_integral_accumulator, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setAllowableClosedLoopError()");
		return;
	}
	if (max_integral_accumulator != max_integral_accumulator_[index])
	{
		pidf_changed_[index] = true;
		max_integral_accumulator_[index] = max_integral_accumulator;
	}
}
int TalonHWCommand::getMaxIntegralAccumulator(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
		return 0.0;
	}
	return max_integral_accumulator_[index];
}
void TalonHWCommand::setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setClosedLoopPeakOutput()");
		return;
	}
	if (fabs(closed_loop_peak_output - closed_loop_peak_output_[index]) > double_value_epsilon)
	{
		pidf_changed_[index] = true;
		closed_loop_peak_output_[index] = closed_loop_peak_output;
	}
}
double TalonHWCommand::getClosedLoopPeakOutput(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getClosedLoopPeakOutput()");
		return 0.0;
	}
	return closed_loop_peak_output_[index];
}

void TalonHWCommand::setClosedLoopPeriod(int closed_loop_period, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::setClosedLoopPeriod()");
		return;
	}
	if (closed_loop_period != closed_loop_period_[index])
	{
		pidf_changed_[index] = true;
		closed_loop_period_[index] = closed_loop_period;
	}
}
int TalonHWCommand::getClosedLoopPeriod(size_t index) const
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::getClosedLoopPeriod()");
		return 0.0;
	}
	return closed_loop_period_[index];
}
bool TalonHWCommand::pidfChanged(double &p, double &i, double &d, double &f, int &iz, int &allowable_closed_loop_error, double &max_integral_accumulator, double &closed_loop_peak_output, int &closed_loop_period, size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::pidfChanged()");
		return false;
	}
	p = p_[index];
	i = i_[index];
	d = d_[index];
	f = f_[index];
	iz = i_zone_[index];
	allowable_closed_loop_error = allowable_closed_loop_error_[index];
	max_integral_accumulator = max_integral_accumulator_[index];
	closed_loop_peak_output = closed_loop_peak_output_[index];
	closed_loop_period = closed_loop_period_[index];
	if (!pidf_changed_[index])
		return false;
	pidf_changed_[index] = false;
	return true;
}
void TalonHWCommand::resetPIDF(size_t index)
{
	if (index >= TALON_PIDF_SLOTS)
	{
		ROS_WARN("Invalid index passed to TalonHWCommand::resetPIDF()");
		return;
	}
	pidf_changed_[index] = true;
}

void TalonHWCommand::setAuxPidPolarity(bool aux_pid_polarity)
{
	if (aux_pid_polarity_ != aux_pid_polarity)
	{
		aux_pid_polarity_ = aux_pid_polarity;
		aux_pid_polarity_changed_ = true;
	}
}
bool TalonHWCommand::getAuxPidPolarity(void) const
{
	return aux_pid_polarity_;
}
bool TalonHWCommand::auxPidPolarityChanged(bool &aux_pid_polarity)
{
	aux_pid_polarity = aux_pid_polarity_;
	if (!aux_pid_polarity_changed_)
		return false;
	aux_pid_polarity_changed_ = false;
	return true;
}
void TalonHWCommand::resetAuxPidPolarity(void)
{
	aux_pid_polarity_changed_ = true;
}

void TalonHWCommand::setIntegralAccumulator(double iaccum)
{
	iaccum_ = iaccum;
	iaccum_changed_ = true;
}
double TalonHWCommand::getIntegralAccumulator(void) const
{
	return iaccum_;
}
bool TalonHWCommand::integralAccumulatorChanged(double &iaccum)
{
	iaccum = iaccum_;
	if (!iaccum_changed_)
		return false;
	iaccum_changed_ = false;
	return true;
}
void TalonHWCommand::resetIntegralAccumulator(void)
{
	iaccum_changed_ = true;
}

void TalonHWCommand::set(double command)
{
	command_ = command;
	if (fabs(command_ - command) > double_value_epsilon)
		command_changed_ = true;
}

void TalonHWCommand::setMode(TalonMode mode)
{
	if ((mode <= TalonMode_First) || (mode >= TalonMode_Last))
	{
		ROS_WARN("Invalid mode passed to TalonHWCommand::setMode()");
		return;
	}
	if (mode != mode_)
	{
		mode_         = mode;
		mode_changed_ = true;
	}
}
// Check to see if mode changed since last call
// If so, return true and set mode to new desired
// talon mode
// If mode hasn't changed, return false
// Goal here is to prevent writes to the CAN
// bus to repeatedly set the mode to the same value.
// Instead, only send a setMode to a given Talon if
// the mode has actually changed.
bool TalonHWCommand::modeChanged(TalonMode &mode)
{
	mode = mode_;
	if (!mode_changed_)
		return false;
	mode_changed_ = false;
	return true;
}
void TalonHWCommand::resetMode(void)
{
	mode_changed_ = true;
}

void TalonHWCommand::setDemand1Type(DemandType demand_type)
{
	if ((demand_type <  DemandType_Neutral) ||
		(demand_type >= DemandType_Last))
	{
		ROS_WARN("Invalid mode passed to TalonHWCommand::setDemand1Type()");
		return;
	}
	if (demand1_type_ != demand_type)
	{
		demand1_type_    = demand_type;
		demand1_changed_ = true;
	}
}
DemandType TalonHWCommand::getDemand1Type(void) const
{
	return demand1_type_;
}

void TalonHWCommand::setDemand1Value(double demand_value)
{
	if (fabs(demand1_value_ - demand_value) > double_value_epsilon)
	{
		demand1_value_   = demand_value;
		demand1_changed_ = true;
	}
}

void TalonHWCommand::resetDemand1(void)
{
	demand1_changed_ = true;
}

double TalonHWCommand::getDemand1Value(void) const
{
	return demand1_value_;
}
bool TalonHWCommand::demand1Changed(DemandType &type, double &value)
{
	type  = demand1_type_;
	value = demand1_value_;
	if (!demand1_changed_)
		return false;
	demand1_changed_ = false;
	return true;
}

void TalonHWCommand::setNeutralMode(NeutralMode neutral_mode)
{
	if (neutral_mode == NeutralMode_Uninitialized)
		return; // Don't warn on this?
	else if ((neutral_mode < NeutralMode_Uninitialized) ||
			 (neutral_mode >= NeutralMode_Last))
	{
		ROS_WARN("Invalid neutral_mode passed to TalonHWCommand::setNeutralMode()");
		return;
	}
	if (neutral_mode != neutral_mode_)
	{
		neutral_mode_         = neutral_mode;
		neutral_mode_changed_ = true;
	}
}
bool TalonHWCommand::getNeutralMode(void)
{
	return neutral_mode_;
}
bool TalonHWCommand::neutralModeChanged(NeutralMode &neutral_mode)
{
	neutral_mode = neutral_mode_;
	if (!neutral_mode_changed_)
		return false;
	neutral_mode_changed_ = false;
	return true;
}

void TalonHWCommand::setNeutralOutput(void)
{
	neutral_output_ = true;
}
// Set motor controller to neutral output
// This should be a one-shot ... only
// write it to the motor controller once
bool TalonHWCommand::neutralOutputChanged(void)
{
	if (!neutral_output_)
		return false;
	neutral_output_ = false;
	return true;
}

void TalonHWCommand::setPidfSlot(int pidf_slot)
{
	if (pidf_slot != pidf_slot_)
	{
		pidf_slot_ = pidf_slot;
		pidf_slot_changed_ = true;
	}
}
int TalonHWCommand::getPidfSlot(void)const
{
	return pidf_slot_;
}
bool TalonHWCommand::slotChanged(int &newpidfSlot)
{
	newpidfSlot = pidf_slot_;
	if (!pidf_slot_changed_)
		return false;
	pidf_slot_changed_ = false;
	return true;
}
void TalonHWCommand::resetPidfSlot(void)
{
	pidf_slot_changed_ = true;
}

void TalonHWCommand::setInvert(bool invert)
{
	if (invert != invert_)
	{
		invert_ = invert;
		invert_changed_ = true;
	}
}
void TalonHWCommand::setSensorPhase(bool invert)
{
	if (invert != sensor_phase_)
	{
		sensor_phase_ = invert;
		invert_changed_ = true;
	}
}
bool TalonHWCommand::invertChanged(bool &invert, bool &sensor_phase)
{
	invert = invert_;
	sensor_phase = sensor_phase_;
	if (!invert_changed_)
		return false;
	invert_changed_ = false;
	return true;
}

FeedbackDevice TalonHWCommand::getEncoderFeedback(void) const
{
	return encoder_feedback_;
}
void TalonHWCommand::setEncoderFeedback(FeedbackDevice encoder_feedback)
{
	if ((encoder_feedback >= FeedbackDevice_Uninitialized) &&
		(encoder_feedback <  FeedbackDevice_Last) )
	{
		if (encoder_feedback != encoder_feedback_)
		{
			encoder_feedback_ = encoder_feedback;
			encoder_feedback_changed_ = true;
		}
	}
	else
		ROS_WARN_STREAM("Invalid feedback device requested");
}
double TalonHWCommand::getFeedbackCoefficient(void) const
{
	return feedback_coefficient_;
}
void TalonHWCommand::setFeedbackCoefficient(double feedback_coefficient)
{
	if (fabs(feedback_coefficient - feedback_coefficient_) > double_value_epsilon)
	{
		feedback_coefficient_ = feedback_coefficient;
		encoder_feedback_changed_ = true;
	}
}
bool TalonHWCommand::encoderFeedbackChanged(FeedbackDevice &encoder_feedback, double &feedback_coefficient)
{
	encoder_feedback = encoder_feedback_;
	feedback_coefficient = feedback_coefficient_;
	if (!encoder_feedback_changed_)
		return false;
	encoder_feedback_changed_ = false;
	return true;
}
void TalonHWCommand::resetEncoderFeedback(void)
{
	encoder_feedback_changed_ = true;
}

RemoteFeedbackDevice TalonHWCommand::getRemoteEncoderFeedback(void) const
{
	return remote_encoder_feedback_;
}
void TalonHWCommand::setRemoteEncoderFeedback(RemoteFeedbackDevice remote_encoder_feedback)
{
	if ((remote_encoder_feedback >= RemoteFeedbackDevice_SensorSum) &&
		(remote_encoder_feedback <  RemoteFeedbackDevice_Last) )
	{
		if (remote_encoder_feedback != remote_encoder_feedback_)
		{
			remote_encoder_feedback_ = remote_encoder_feedback;
			remote_encoder_feedback_changed_ = true;
		}
	}
	else
		ROS_WARN_STREAM("Invalid remote feedback device requested");
}
bool TalonHWCommand::remoteEncoderFeedbackChanged(RemoteFeedbackDevice &remote_encoder_feedback)
{
	remote_encoder_feedback = remote_encoder_feedback_;
	if (!remote_encoder_feedback_changed_)
		return false;
	remote_encoder_feedback_changed_ = false;
	return true;
}
void TalonHWCommand::resetRemoteEncoderFeedback(void)
{
	remote_encoder_feedback_changed_ = true;
}
int TalonHWCommand::getRemoteFeedbackDeviceId(size_t remote_ordinal) const
{
	if (remote_ordinal >= 2)
	{
		ROS_WARN("getRemoteFeedbackDeviceId: remote_ordinal too large");
		return -1;
	}
	return remote_feedback_device_ids_[remote_ordinal];
}
RemoteSensorSource TalonHWCommand::getRemoteFeedbackFilter(size_t remote_ordinal) const
{
	if (remote_ordinal >= 2)
	{
		ROS_WARN("getRemoteFeedbackFilter : remote_ordinal too large");
		return RemoteSensorSource_Last;
	}
	return remote_feedback_filters_[remote_ordinal];
}

void TalonHWCommand::setRemoteFeedbackDeviceId(int remote_feedback_device_id, size_t remote_ordinal)
{
	if (remote_ordinal >= 2)
	{
		ROS_WARN("setRemoteFeedbackFilter : remote_ordinal too large");
		return;
	}
	remote_feedback_device_ids_[remote_ordinal] = remote_feedback_device_id;
	remote_feedback_filters_changed_ = true;
}
void TalonHWCommand::setRemoteFeedbackFilter(RemoteSensorSource remote_sensor_source, size_t remote_ordinal)
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
	remote_feedback_filters_changed_ = true;
}
bool TalonHWCommand::remoteFeedbackFiltersChanged(std::array<int, 2> &remote_feedback_device_ids, std::array<RemoteSensorSource, 2> &remote_feedback_filters)
{
	remote_feedback_device_ids = remote_feedback_device_ids_;
	remote_feedback_filters = remote_feedback_filters_;
	if (!remote_feedback_filters_changed_)
		return false;
	remote_feedback_filters_changed_ = false;
	return true;
}
void TalonHWCommand::resetRemoteFeedbackFilters(void)
{
	remote_feedback_filters_changed_ = true;
}

FeedbackDevice TalonHWCommand::getSensorTerm(SensorTerm sensor_terms) const
{
	if (sensor_terms < SensorTerm_Last)
		return sensor_terms_[sensor_terms];
	ROS_WARN("getSensorTerm : sensor_terms index too large");
	return FeedbackDevice_Last;
}
void TalonHWCommand::setSensorTerm(FeedbackDevice feedback_device, SensorTerm sensor_terms)
{
	if (sensor_terms >= SensorTerm_Last)
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
	sensor_terms_[sensor_terms] = feedback_device;
	sensor_terms_changed_ = true;
}
bool TalonHWCommand::sensorTermsChanged(std::array<FeedbackDevice, SensorTerm_Last> &sensor_terms)
{
	sensor_terms = sensor_terms_;
	if (!sensor_terms_changed_)
		return false;
	sensor_terms_changed_ = false;
	return true;
}
void TalonHWCommand::resetSensorTerms(void)
{
	sensor_terms_changed_ = true;
}

int TalonHWCommand::getEncoderTicksPerRotation(void) const
{
	return encoder_ticks_per_rotation_;
}

void TalonHWCommand::setEncoderTicksPerRotation(int encoder_ticks_per_rotation)
{
	encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
}

//output shaping
void TalonHWCommand::setClosedloopRamp(double closed_loop_ramp)
{
	if (fabs(closed_loop_ramp_ - closed_loop_ramp) > double_value_epsilon)
	{
		closed_loop_ramp_ = closed_loop_ramp;
		output_shaping_changed_ = true;
	}
}
double TalonHWCommand::getClosedloopRamp(void) const
{
	return closed_loop_ramp_;
}
void TalonHWCommand::setOpenloopRamp(double open_loop_ramp)
{
	if (fabs(open_loop_ramp_ - open_loop_ramp) > double_value_epsilon)
	{
		open_loop_ramp_ = open_loop_ramp;
		output_shaping_changed_ = true;
	}
}
double TalonHWCommand::getOpenloopRamp(void) const
{
	return open_loop_ramp_;
}

void TalonHWCommand::setPeakOutputForward(double peak_output_forward)
{
	if (fabs(peak_output_forward - peak_output_forward_) > double_value_epsilon)
	{
		peak_output_forward_ = peak_output_forward;
		output_shaping_changed_ = true;
	}
}
double TalonHWCommand::getPeakOutputForward(void) const
{
	return peak_output_forward_;
}

void TalonHWCommand::setPeakOutputReverse(double peak_output_reverse)
{
	if (fabs(peak_output_reverse - peak_output_reverse_) > double_value_epsilon)
	{
		peak_output_reverse_ = peak_output_reverse;
		output_shaping_changed_ = true;
	}
}
double TalonHWCommand::getPeakOutputReverse(void) const
{
	return peak_output_reverse_;
}

void TalonHWCommand::setNominalOutputForward(double nominal_output_forward)
{
	if (fabs(nominal_output_forward - nominal_output_forward_) > double_value_epsilon)
	{
		nominal_output_forward_ = nominal_output_forward;
		output_shaping_changed_ = true;
	}
}
double TalonHWCommand::getNominalOutputForward(void) const
{
	return nominal_output_forward_;
}

void TalonHWCommand::setNominalOutputReverse(double nominal_output_reverse)
{
	if (fabs(nominal_output_reverse - nominal_output_reverse_) > double_value_epsilon)
	{
		nominal_output_reverse_ = nominal_output_reverse;
		output_shaping_changed_ = true;
	}
}
double TalonHWCommand::getNominalOutputReverse(void) const
{
	return nominal_output_reverse_;
}

void TalonHWCommand::setNeutralDeadband(double neutral_deadband)
{
	if (fabs(neutral_deadband - neutral_deadband_) > double_value_epsilon)
	{
		neutral_deadband_ = neutral_deadband;
		output_shaping_changed_ = true;
	}
}
double TalonHWCommand::getNeutralDeadband(void) const
{
	return neutral_deadband_;
}
bool TalonHWCommand::outputShapingChanged(double &closed_loop_ramp,
		double &open_loop_ramp,
		double &peak_output_forward,
		double &peak_output_reverse,
		double &nominal_output_forward,
		double &nominal_output_reverse,
		double &neutral_deadband)
{
	closed_loop_ramp = closed_loop_ramp_;
	open_loop_ramp = open_loop_ramp_;
	peak_output_forward = peak_output_forward_;
	peak_output_reverse = peak_output_reverse_;
	nominal_output_forward = nominal_output_forward_;
	nominal_output_reverse = nominal_output_reverse_;
	neutral_deadband = neutral_deadband_;
	if (!output_shaping_changed_)
		return false;
	output_shaping_changed_ = false;
	return true;
}
void TalonHWCommand::resetOutputShaping(void)
{
	output_shaping_changed_ = true;
}

void TalonHWCommand::setVoltageCompensationSaturation(double voltage)
{
	if (fabs(voltage - voltage_compensation_saturation_) > double_value_epsilon)
	{
		voltage_compensation_saturation_ = voltage;
		voltage_compensation_changed_ = true;
	}
}
double TalonHWCommand::getVoltageCompensationSaturation(void) const
{
	return voltage_compensation_saturation_;
}

void TalonHWCommand::setVoltageMeasurementFilter(int filterWindowSamples)
{
	if (filterWindowSamples != voltage_measurement_filter_)
	{
		voltage_measurement_filter_ = filterWindowSamples;
		voltage_compensation_changed_ = true;
	}
}
int TalonHWCommand::getVoltageMeasurementFilter(void) const
{
	return voltage_compensation_saturation_;
}

void TalonHWCommand::setVoltageCompensationEnable(bool enable)
{
	if (enable != voltage_compensation_enable_)
	{
		voltage_compensation_enable_ = enable;
		voltage_compensation_changed_ = true;
	}
}

bool TalonHWCommand::getEnableVoltageCompenation(void) const
{
	return voltage_compensation_enable_;
}

bool TalonHWCommand::voltageCompensationChanged(double &voltage_compensation_saturation,
		int &voltage_measurement_filter,
		bool &voltage_compensation_enable)
{
	voltage_compensation_saturation = voltage_compensation_saturation_;
	voltage_measurement_filter      = voltage_measurement_filter_;
	voltage_compensation_enable     = voltage_compensation_enable_;
	if (voltage_compensation_changed_)
	{
		voltage_compensation_changed_ = false;
		return true;
	}
	return false;
}
void TalonHWCommand::resetVoltageCompensation(void)
{
	voltage_compensation_changed_ = true;
}

void TalonHWCommand::setVelocityMeasurementPeriod(hardware_interface::VelocityMeasurementPeriod period)
{
	if (period != velocity_measurement_period_)
	{
		velocity_measurement_period_ = period;
		velocity_measurement_changed_ = true;
	}
}

bool TalonHWCommand::getVoltageMeasurementPeriod(void) const
{
	return velocity_measurement_period_;
}

void TalonHWCommand::setVelocityMeasurementWindow(int window)
{
	if (window != velocity_measurement_window_)
	{
		velocity_measurement_window_ = window;
		velocity_measurement_changed_ = true;
	}
}

bool TalonHWCommand::getVoltageMeasurementWindow(void) const
{
	return velocity_measurement_window_;
}

bool TalonHWCommand::velocityMeasurementChanged(hardware_interface::VelocityMeasurementPeriod &period,
		int &window)
{
	period = velocity_measurement_period_;
	window = velocity_measurement_window_;
	if (velocity_measurement_changed_)
	{
		velocity_measurement_changed_ = false;
		return true;
	}
	return false;
}
void TalonHWCommand::resetVelocityMeasurement(void)
{
	velocity_measurement_changed_ = true;
}

void TalonHWCommand::setSelectedSensorPosition(double position)
{
	sensor_position_value_ = position;
	sensor_position_changed_ = true;
}
double TalonHWCommand::getSelectedSensorPosition(void) const
{
	return sensor_position_value_;
}

bool TalonHWCommand::sensorPositionChanged(double &position)
{
	position = sensor_position_value_;
	if (!sensor_position_changed_)
		return false;
	sensor_position_changed_ = false;
	return true;
}
void TalonHWCommand::resetSensorPosition(void)
{
	sensor_position_changed_ = true;
}


void TalonHWCommand::setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
{
	if ((source != limit_switch_local_forward_source_) ||
		(normal != limit_switch_local_forward_normal_))
	{
		if ((source <= LimitSwitchSource_Uninitialized) ||
			(source >= LimitSwitchSource_Last))
		{
			ROS_WARN("Invalid source in setForwardLimitSwitchSource");
			return;
		}
		if ((normal <= LimitSwitchNormal_Uninitialized) ||
				(normal >= LimitSwitchNormal_Last))
		{
			ROS_WARN("Invalid normal in setForwardLimitSwitchSource");
			return;
		}
		if ((limit_switch_local_forward_source_ != source) ||
				(limit_switch_local_forward_normal_ != normal) )
		{
			limit_switch_local_forward_source_ = source;
			limit_switch_local_forward_normal_ = normal;
			limit_switch_local_changed_ = true;
		}
	}
}

void TalonHWCommand::getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
{
	source = limit_switch_local_forward_source_;
	normal = limit_switch_local_forward_normal_;
}

void TalonHWCommand::setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
{
	if ((source != limit_switch_local_reverse_source_) || (normal != limit_switch_local_reverse_normal_))
	{
		if ((source <= LimitSwitchSource_Uninitialized) ||
			(source >= LimitSwitchSource_Last))
		{
			ROS_WARN("Invalid source in setReverseLimitSwitchSource");
			return;
		}
		if ((normal <= LimitSwitchNormal_Uninitialized) ||
				(normal >= LimitSwitchNormal_Last))
		{
			ROS_WARN("Invalid normal in setReverseLimitSwitchSource");
			return;
		}
		if ((limit_switch_local_reverse_source_ != source) ||
				(limit_switch_local_reverse_normal_ != normal) )
		{
			limit_switch_local_reverse_source_ = source;
			limit_switch_local_reverse_normal_ = normal;
			limit_switch_local_changed_ = true;
		}
	}
}

void TalonHWCommand::getReverseLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
{
	source = limit_switch_local_reverse_source_;
	normal = limit_switch_local_reverse_normal_;
}

bool TalonHWCommand::limitSwitchesSourceChanged(LimitSwitchSource &forward_source, LimitSwitchNormal &forward_normal, LimitSwitchSource &reverse_source, LimitSwitchNormal &reverse_normal)
{
	forward_source = limit_switch_local_forward_source_;
	forward_normal = limit_switch_local_forward_normal_;
	reverse_source = limit_switch_local_reverse_source_;
	reverse_normal = limit_switch_local_reverse_normal_;
	if (!limit_switch_local_changed_)
		return false;
	limit_switch_local_changed_ = false;
	return true;
}
void TalonHWCommand::resetLimitSwitchesSource(void)
{
	limit_switch_local_changed_ = true;
}

void TalonHWCommand::setRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
{
	if ((source <= RemoteLimitSwitchSource_Uninitialized) ||
		(source >= RemoteLimitSwitchSource_Last))
	{
		ROS_WARN("Invalid source in setRemoteForwardLimitSwitchSource");
		return;
	}
	if ((normal <= LimitSwitchNormal_Uninitialized) ||
		(normal >= LimitSwitchNormal_Last))
	{
		ROS_WARN("Invalid normal in setRemoteForwardLimitSwitchSource");
		return;
	}
	if ((limit_switch_remote_forward_source_ != source) ||
		(limit_switch_remote_forward_normal_ != normal) ||
		(limit_switch_remote_forward_id_     != id    )  )
	{
		limit_switch_remote_forward_source_ = source;
		limit_switch_remote_forward_normal_ = normal;
		limit_switch_remote_forward_id_     = id;
		limit_switch_remote_changed_ = true;
	}
}

void TalonHWCommand::getRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
{
	source = limit_switch_remote_forward_source_;
	normal = limit_switch_remote_forward_normal_;
	id     = limit_switch_remote_forward_id_;
}

void TalonHWCommand::setRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
{
	if ((source <= RemoteLimitSwitchSource_Uninitialized) ||
		(source >= RemoteLimitSwitchSource_Last))
	{
		ROS_WARN("Invalid source in setRemoteReverseLimitSwitchSource");
		return;
	}
	if ((normal <= LimitSwitchNormal_Uninitialized) ||
		(normal >= LimitSwitchNormal_Last))
	{
		ROS_WARN("Invalid normal in setRemoteReverseLimitSwitchSource");
		return;
	}
	if ((limit_switch_remote_reverse_source_ != source) ||
		(limit_switch_remote_reverse_normal_ != normal) ||
		(limit_switch_remote_reverse_id_     != id    )  )
	{
		limit_switch_remote_reverse_source_ = source;
		limit_switch_remote_reverse_normal_ = normal;
		limit_switch_remote_reverse_id_     = id;
		limit_switch_remote_changed_ = true;
	}
}

void TalonHWCommand::getRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
{
	source = limit_switch_remote_reverse_source_;
	normal = limit_switch_remote_reverse_normal_;
	id     = limit_switch_remote_reverse_id_;
}

bool TalonHWCommand::remoteLimitSwitchesSourceChanged(RemoteLimitSwitchSource &forward_source, LimitSwitchNormal &forward_normal, unsigned int &forward_id, RemoteLimitSwitchSource &reverse_source, LimitSwitchNormal &reverse_normal, unsigned int &reverse_id)
{
	forward_source = limit_switch_remote_forward_source_;
	forward_normal = limit_switch_remote_forward_normal_;
	forward_id     = limit_switch_remote_forward_id_;
	reverse_source = limit_switch_remote_reverse_source_;
	reverse_normal = limit_switch_remote_reverse_normal_;
	reverse_id     = limit_switch_remote_reverse_id_;
	if (!limit_switch_remote_changed_)
		return false;
	limit_switch_remote_changed_ = false;
	return true;
}
void TalonHWCommand::resetRemoteLimitSwitchesSource(void)
{
	limit_switch_remote_changed_ = true;
}

// softlimits
void TalonHWCommand::setForwardSoftLimitThreshold(double threshold)
{
	if (fabs(threshold - softlimit_forward_threshold_) > double_value_epsilon)
	{
		softlimit_forward_threshold_ = threshold;
		softlimit_changed_ = true;
	}
}
double TalonHWCommand::getForwardSoftLimitThreshold(void) const
{
	return softlimit_forward_threshold_;
}

void TalonHWCommand::setForwardSoftLimitEnable(bool enable)
{
	if (enable != softlimit_forward_enable_)
	{
		softlimit_forward_enable_ = enable;
		softlimit_changed_ = true;
	}
}
bool TalonHWCommand::getForwardSoftLimitEnable(void) const
{
	return softlimit_forward_enable_;
}
void TalonHWCommand::setReverseSoftLimitThreshold(double threshold)
{
	if (fabs(threshold - softlimit_reverse_threshold_) > double_value_epsilon)
	{
		softlimit_reverse_threshold_ = threshold;
		softlimit_changed_ = true;
	}
}
double TalonHWCommand::getReverseSoftLimitThreshold(void) const
{
	return softlimit_reverse_threshold_;
}

void TalonHWCommand::setReverseSoftLimitEnable(bool enable)
{
	if (enable != softlimit_reverse_enable_)
	{
		softlimit_reverse_enable_ = enable;
		softlimit_changed_ = true;
	}
}
bool TalonHWCommand::getReverseSoftLimitEnable(void) const
{
	return softlimit_reverse_enable_;
}

void TalonHWCommand::setOverrideSoftLimitsEnable(bool enable)
{
	if (enable != softlimits_override_enable_)
	{
		softlimits_override_enable_ = enable;
		softlimit_changed_ = true;
	}
}
bool TalonHWCommand::getOverrideSoftsLimitEnable(void) const
{
	return softlimits_override_enable_;
}

bool TalonHWCommand::softLimitChanged(double &forward_threshold, bool &forward_enable, double &reverse_threshold, bool &reverse_enable, bool &override_enable)
{
	forward_threshold = softlimit_forward_threshold_;
	forward_enable = softlimit_forward_enable_;
	reverse_threshold = softlimit_reverse_threshold_;
	reverse_enable = softlimit_reverse_enable_;
	override_enable = softlimits_override_enable_;
	if (!softlimit_changed_)
		return false;
	softlimit_changed_ = false;
	return true;
}
void TalonHWCommand::resetSoftLimit(void)
{
	softlimit_changed_ = true;
}

// current limits - Talon SRX only
void TalonHWCommand::setPeakCurrentLimit(int amps)
{
	if (amps != current_limit_peak_amps_)
	{
		current_limit_peak_amps_ = amps;
		current_limit_changed_ = true;
	}
}
int TalonHWCommand::getPeakCurrentLimit(void) const
{
	return current_limit_peak_amps_;
}

void TalonHWCommand::setPeakCurrentDuration(int msec)
{
	if (msec != current_limit_peak_msec_)
	{
		current_limit_peak_msec_ = msec;
		current_limit_changed_ = true;
	}
}
int TalonHWCommand::getPeakCurrentDuration(void) const
{
	return current_limit_peak_msec_;
}
void TalonHWCommand::setContinuousCurrentLimit(int amps)
{
	if (amps != current_limit_continuous_amps_)
	{
		current_limit_continuous_amps_ = amps;
		current_limit_changed_ = true;
	}
}
int TalonHWCommand::getContinuousCurrentLimit(void) const
{
	return current_limit_continuous_amps_;
}
void TalonHWCommand::setCurrentLimitEnable(bool enable)
{
	if (enable != current_limit_enable_)
	{
		current_limit_enable_ = enable;
		current_limit_changed_ = true;
	}
}
bool TalonHWCommand::getCurrentLimitEnable(void) const
{
	return current_limit_enable_;
}

bool TalonHWCommand::currentLimitChanged(int &peak_amps, int &peak_msec, int &continuous_amps, bool &enable)
{
	peak_amps = current_limit_peak_amps_;
	peak_msec = current_limit_peak_msec_;
	continuous_amps = current_limit_continuous_amps_;
	enable = current_limit_enable_;
	if (!current_limit_changed_)
		return false;
	current_limit_changed_ = false;
	return true;
}
void TalonHWCommand::resetCurrentLimit(void)
{
	current_limit_changed_ = false;
}

// Current limits - TalonFX / Falcon500
void TalonHWCommand::setSupplyCurrentLimit(double supply_current_limit)
{
	if (supply_current_limit_ != supply_current_limit)
	{
		supply_current_limit_ = supply_current_limit;
		supply_current_limit_changed_ = true;
	}
}
double TalonHWCommand::getSupplyCurrentLimit(void) const
{
	return supply_current_limit_;
}
void TalonHWCommand::setSupplyCurrentTriggerThresholdCurrent(double supply_current_trigger_threshold_current)
{
	if (supply_current_trigger_threshold_current_ != supply_current_trigger_threshold_current)
	{
		supply_current_trigger_threshold_current_ = supply_current_trigger_threshold_current;
		supply_current_limit_changed_ = true;
	}
}
double TalonHWCommand::getSupplyCurrentTriggerThresholdCurrent(void) const
{
	return supply_current_trigger_threshold_current_;
}
void TalonHWCommand::setSupplyCurrentTriggerThresholdTime(double supply_current_trigger_threshold_time)
{
	if (supply_current_trigger_threshold_time_ != supply_current_trigger_threshold_time)
	{
		supply_current_trigger_threshold_time_ = supply_current_trigger_threshold_time;
		supply_current_limit_changed_ = true;
	}
}
double TalonHWCommand::getSupplyCurrentTriggerThresholdTime(void) const
{
	return supply_current_trigger_threshold_time_;
}
void TalonHWCommand::setSupplyCurrentLimitEnable(bool supply_current_limit_enable)
{
	if (supply_current_limit_enable_ != supply_current_limit_enable)
	{
		supply_current_limit_enable_ = supply_current_limit_enable;
		supply_current_limit_changed_ = true;
	}
}
bool TalonHWCommand::getSupplyCurrentLimitEnable(void) const
{
	return supply_current_limit_enable_;
}

bool TalonHWCommand::supplyCurrentLimitChanged(double &supply_current_limit,
		double &supply_current_trigger_threshold_current,
		double &supply_current_trigger_threshold_time,
		double &supply_current_limit_enable)
{
	supply_current_limit = supply_current_limit_;
	supply_current_trigger_threshold_current = supply_current_trigger_threshold_current_;
	supply_current_trigger_threshold_time = supply_current_trigger_threshold_time_;
	supply_current_limit_enable = supply_current_limit_enable_;
	const bool ret = supply_current_limit_changed_;
	supply_current_limit_changed_ = false;
	return ret;
}
void  TalonHWCommand::resetSupplyCurrentLimit(void)
{
	supply_current_limit_changed_ = true;
}

void TalonHWCommand::setStatorCurrentLimit(bool stator_current_limit)
{
	if (stator_current_limit_ != stator_current_limit)
	{
		stator_current_limit_ = stator_current_limit;
		stator_current_limit_changed_ = true;
	}
}
double TalonHWCommand::getStatorCurrentLimit(void) const
{
	return stator_current_limit_;
}
void TalonHWCommand::setStatorCurrentTriggerThresholdCurrent(double stator_current_trigger_threshold_current)
{
	if (stator_current_trigger_threshold_current_ != stator_current_trigger_threshold_current)
	{
		stator_current_trigger_threshold_current_ = stator_current_trigger_threshold_current;
		stator_current_limit_changed_ = true;
	}
}
double TalonHWCommand::getStatorCurrentTriggerThresholdCurrent(void) const
{
	return stator_current_trigger_threshold_current_;
}
void TalonHWCommand::setStatorCurrentTriggerThresholdTime(double stator_current_trigger_threshold_time)
{
	if (stator_current_trigger_threshold_time_ != stator_current_trigger_threshold_time)
	{
		stator_current_trigger_threshold_time_ = stator_current_trigger_threshold_time;
		stator_current_limit_changed_ = true;
	}
}
double TalonHWCommand::getStatorCurrentTriggerThresholdTime(void) const
{
	return stator_current_trigger_threshold_time_;
}
void TalonHWCommand::setStatorCurrentLimitEnable(bool stator_current_limit_enable)
{
	if (stator_current_limit_enable_ != stator_current_limit_enable)
	{
		stator_current_limit_enable_ = stator_current_limit_enable;
		stator_current_limit_changed_ = true;
	}
}
bool TalonHWCommand::getStatorCurrentLimitEnable(void) const
{
	return stator_current_limit_enable_;
}

bool TalonHWCommand::statorCurrentLimitChanged(double &stator_current_limit,
		double &stator_current_trigger_threshold_current,
		double &stator_current_trigger_threshold_time,
		double &stator_current_limit_enable)
{
	stator_current_limit = stator_current_limit_;
	stator_current_trigger_threshold_current = stator_current_trigger_threshold_current_;
	stator_current_trigger_threshold_time = stator_current_trigger_threshold_time_;
	stator_current_limit_enable = stator_current_limit_enable_;
	const bool ret = stator_current_limit_changed_;
	stator_current_limit_changed_ = false;
	return ret;
}
void  TalonHWCommand::resetStatorCurrentLimit(void)
{
	stator_current_limit_changed_ = true;
}

void TalonHWCommand::setMotionCruiseVelocity(double velocity)
{
	if (fabs(velocity - motion_cruise_velocity_) > double_value_epsilon)
	{
		motion_cruise_velocity_ = velocity;
		motion_cruise_changed_ = true;
	}
}
double TalonHWCommand::getMotionCruiseVelocity(void) const
{
	return motion_cruise_velocity_;
}
void TalonHWCommand::setMotionAcceleration(double acceleration)
{
	if (fabs(acceleration - motion_acceleration_) > double_value_epsilon)
	{
		motion_acceleration_ = acceleration;
		motion_cruise_changed_ = true;
	}
}
double TalonHWCommand::getMotionAcceleration(void) const
{
	return motion_acceleration_;
}
void TalonHWCommand::setMotionSCurveStrength(unsigned int s_curve_strength)
{
	if ((s_curve_strength != motion_s_curve_strength_) &&
		(s_curve_strength <= 8))
	{
		motion_s_curve_strength_ = s_curve_strength;
		motion_cruise_changed_ = true;
	}
}
unsigned int TalonHWCommand::getMotionSCurveStrength(void) const
{
	return motion_s_curve_strength_;
}

bool TalonHWCommand::motionCruiseChanged(double &velocity, double &acceleration, unsigned int &s_curve_strength)
{
	velocity = motion_cruise_velocity_;
	acceleration = motion_acceleration_;
	s_curve_strength = motion_s_curve_strength_;
	if (!motion_cruise_changed_)
		return false;
	motion_cruise_changed_ = false;
	return true;
}
void TalonHWCommand::resetMotionCruise(void)
{
	motion_cruise_changed_ = true;
}

// This is a one shot - when set, it needs to
// call the appropriate Talon function once
// then clear itself
void TalonHWCommand::setClearMotionProfileTrajectories(void)
{
	motion_profile_clear_trajectories_ = true;
}
bool TalonHWCommand::getClearMotionProfileTrajectories(void) const
{
	return motion_profile_clear_trajectories_;
}
bool TalonHWCommand::clearMotionProfileTrajectoriesChanged(void)
{
	if (!motion_profile_clear_trajectories_)
		return false;
	motion_profile_clear_trajectories_ = false;
	return true;
}
void TalonHWCommand::PushMotionProfileTrajectory(const TrajectoryPoint &traj_pt)
{
	motion_profile_trajectory_points_.push_back(traj_pt);
}
std::vector<TrajectoryPoint> TalonHWCommand::getMotionProfileTrajectories(void) const
{
	return motion_profile_trajectory_points_;
}
bool TalonHWCommand::motionProfileTrajectoriesChanged(std::vector<TrajectoryPoint> &points)
{
	if (motion_profile_trajectory_points_.size() != 0)
	{
		//ROS_WARN_STREAM("motionProfileTraectoriesChanged, mptp.size()=" << motion_profile_trajectory_points_.size());
		// Return up to 20 points at a time - too
		// many really slows down the hardware interface
		auto start = motion_profile_trajectory_points_.begin();
		auto end   = start + std::min((size_t)motion_profile_trajectory_points_.size(), (size_t)4000); //Intentionally very large
		points = std::vector<TrajectoryPoint>(start, end);
		motion_profile_trajectory_points_.erase(start, end);
		//ROS_WARN_STREAM("  returning points.size()=" << points.size());
		return true;
	}
	return false;
}

// This is a one shot - when set, it needs to
// call the appropriate Talon function once
// then clear itself
void TalonHWCommand::setClearMotionProfileHasUnderrun(void)
{
	motion_profile_clear_has_underrun_ = true;
}
bool TalonHWCommand::getClearMotionProfileHasUnderrun(void) const
{
	return motion_profile_clear_has_underrun_;
}
bool TalonHWCommand::clearMotionProfileHasUnderrunChanged(void)
{
	if (!motion_profile_clear_has_underrun_)
		return false;
	motion_profile_clear_has_underrun_ = false;
	return true;
}

void TalonHWCommand::setStatusFramePeriod(StatusFrame status_frame, uint8_t period)
{
	if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
	{
		if (status_frame_periods_[status_frame] != period)
		{
			status_frame_periods_[status_frame] = period;
			status_frame_periods_changed_[status_frame] = true;
		}
	}
	else
		ROS_ERROR("Invalid status_frame value passed to TalonHWCommand::setStatusFramePeriod()");
}

uint8_t TalonHWCommand::getStatusFramePeriod(StatusFrame status_frame) const
{
	if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
		return status_frame_periods_[status_frame];

	ROS_ERROR("Invalid status_frame value passed to TalonHWCommand::setStatusFramePeriod()");
	return 0;
}

bool TalonHWCommand::statusFramePeriodChanged(StatusFrame status_frame, uint8_t &period)
{
	if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
	{
		period = status_frame_periods_[status_frame];
		if (!status_frame_periods_changed_[status_frame])
			return false;
		status_frame_periods_changed_[status_frame] = false;
		return true;
	}

	ROS_ERROR("Invalid status_frame value passed to TalonHWCommand::setStatusFramePeriod()");
	return false;
}
void TalonHWCommand::resetStatusFramePeriod(StatusFrame status_frame)
{
	if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
	{
		status_frame_periods_changed_[status_frame] = true;
	}
	else
	{
		ROS_ERROR("Invalid status_frame value passed to TalonHWCommand::resetStatusFramePeriod()");
	}
}

void TalonHWCommand::setControlFramePeriod(ControlFrame control_frame, uint8_t period)
{
	if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
	{
		if (control_frame_periods_[control_frame] != period)
		{
			control_frame_periods_[control_frame] = period;
			control_frame_periods_changed_[control_frame] = true;
		}
	}
	else
		ROS_ERROR("Invalid control_frame value passed to TalonHWCommand::setControlFramePeriod()");
}

uint8_t TalonHWCommand::getControlFramePeriod(ControlFrame control_frame) const
{
	if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
		return control_frame_periods_[control_frame];

	ROS_ERROR("Invalid control_frame value passed to TalonHWCommand::setControlFramePeriod()");
	return 0;
}

bool TalonHWCommand::controlFramePeriodChanged(ControlFrame control_frame, uint8_t &period)
{
	if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
	{
		period = control_frame_periods_[control_frame];
		if (!control_frame_periods_changed_[control_frame])
			return false;
		control_frame_periods_changed_[control_frame] = false;
		return true;
	}

	ROS_ERROR("Invalid control_frame value passed to TalonHWCommand::setControlFramePeriod()");
	return false;
}
void TalonHWCommand::resetControlFramePeriod(ControlFrame control_frame)
{
	if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
		control_frame_periods_changed_[control_frame] = true;
	else
		ROS_ERROR("Invalid control_frame value passed to TalonHWCommand::resetControlFramePeriod()");
}

void TalonHWCommand::setMotionProfileTrajectoryPeriod(int msec)
{
	if (msec != motion_profile_profile_trajectory_period_)
	{
		motion_profile_profile_trajectory_period_ = msec;
		motion_profile_profile_trajectory_period_changed_ = true;
	}
}
int TalonHWCommand::getMotionProfileTrajectoryPeriod(void) const
{
	return motion_profile_profile_trajectory_period_;
}
bool TalonHWCommand::motionProfileTrajectoryPeriodChanged(int &msec)
{
	msec = motion_profile_profile_trajectory_period_;
	if (!motion_profile_profile_trajectory_period_changed_)
		return false;
	motion_profile_profile_trajectory_period_changed_ = false;
	return true;
}
void TalonHWCommand::resetMotionProfileTrajectoryPeriod(void)
{
	motion_profile_profile_trajectory_period_changed_ = true;
}

void TalonHWCommand::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
bool TalonHWCommand::getClearStickyFaults(void) const
{
	return clear_sticky_faults_;
}
bool TalonHWCommand::clearStickyFaultsChanged(void)
{
	if (!clear_sticky_faults_)
		return false;
	clear_sticky_faults_ = false;
	return true;
}

void TalonHWCommand::setConversionFactor(double conversion_factor)
{
	if (fabs(conversion_factor - conversion_factor_) > double_value_epsilon)
	{
		conversion_factor_ = conversion_factor;
		conversion_factor_changed_ = true;
	}
}
double TalonHWCommand::getConversionFactor(void) const
{
	return conversion_factor_;
}
bool TalonHWCommand::conversionFactorChanged(double &conversion_factor)
{
	conversion_factor = conversion_factor_;
	if (!conversion_factor_changed_)
		return false;
	conversion_factor_changed_ = false;
	return true;
}

//TalonFX only
void TalonHWCommand::setMotorCommutation(hardware_interface::MotorCommutation motor_commutation)
{
	if (motor_commutation_ != motor_commutation)
	{
		motor_commutation_ = motor_commutation;
		motor_commutation_changed_ = true;
	}
}
hardware_interface::MotorCommutation TalonHWCommand::getMotorCommutation(void) const
{
	return motor_commutation_;
}
bool TalonHWCommand::motorCommutationChanged(hardware_interface::MotorCommutation &motor_commutation)
{
	motor_commutation = motor_commutation_;
	const bool ret = motor_commutation_changed_;
	motor_commutation_changed_ = true;
	return ret;
}
void TalonHWCommand::resetMotorCommutation(void)
{
	motor_commutation_changed_ = false;
}

//TalonFX only
void TalonHWCommand::setAbsoluteSensorRange(hardware_interface::AbsoluteSensorRange absolute_sensor_range)
{
	if (absolute_sensor_range_ != absolute_sensor_range)
	{
		absolute_sensor_range_ = absolute_sensor_range;
		absolute_sensor_range_changed_ = true;
	}
}
hardware_interface::AbsoluteSensorRange TalonHWCommand::getAbsoluteSensorRange(void) const
{
	return absolute_sensor_range_;
}
bool TalonHWCommand::absoluteSensorRangeChanged(hardware_interface::AbsoluteSensorRange &absolute_sensor_range)
{
	absolute_sensor_range = absolute_sensor_range_;
	const bool ret = absolute_sensor_range_changed_;
	absolute_sensor_range_changed_ = true;
	return ret;
}
void TalonHWCommand::resetAbsoluteSensorRange(void)
{
	absolute_sensor_range_changed_ = false;
}

//TalonFX only
void TalonHWCommand::setSensorInitializationStrategy(hardware_interface::SensorInitializationStrategy sensor_initialization_strategy)
{
	if (sensor_initialization_strategy_ != sensor_initialization_strategy)
	{
		sensor_initialization_strategy_ = sensor_initialization_strategy;
		sensor_initialization_strategy_changed_ = true;
	}
}
hardware_interface::SensorInitializationStrategy TalonHWCommand::getSensorInitializationStrategy(void) const
{
	return sensor_initialization_strategy_;
}
bool TalonHWCommand::sensorInitializationStrategyChanged(hardware_interface::SensorInitializationStrategy &sensor_initialization_strategy)
{
	sensor_initialization_strategy = sensor_initialization_strategy_;
	const bool ret = sensor_initialization_strategy_changed_;
	sensor_initialization_strategy_changed_ = true;
	return ret;
}
void TalonHWCommand::resetSensorInitializationStrategy(void)
{
	sensor_initialization_strategy_changed_ = false;
}

void TalonHWCommand::setCustomProfileDisable(bool disable)
{
	custom_profile_disable_ = disable;
}

bool TalonHWCommand::getCustomProfileDisable(void) const
{
	return custom_profile_disable_;
}

std::vector<int> TalonHWCommand::getCustomProfileNextSlot(void) const
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfileNextSlot)");
		return std::vector<int>();
	}
	return custom_profile_next_slot_;
}
void TalonHWCommand::setCustomProfileNextSlot(const std::vector<int> &next_slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (setCustomProfileNextSlot)");
		return;
	}
	custom_profile_next_slot_ = next_slot;
}
double TalonHWCommand::getCustomProfileHz(void) const
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfileHz)");
		return -1;
	}
	return custom_profile_hz_;
}
void TalonHWCommand::setCustomProfileHz(const double &hz)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (setCustomProfileHz)");
		return;
	}
	custom_profile_hz_ = hz;
}
void TalonHWCommand::setCustomProfileRun(const bool &run)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (setCustomProfileRun)");
		return;
	}
	custom_profile_run_ = run;
}
bool TalonHWCommand::getCustomProfileRun(void)
{
	if (custom_profile_disable_)
	{
		// Don't print an error here since
		// this is used in the main write() loop for
		// status
		//ROS_ERROR("Custom profile disabled via param (getCustomProfileRun)");
		return false;
	}
	return custom_profile_run_;
}
void TalonHWCommand::setCustomProfileSlot(const int &slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (setCustomProfileSlot)");
		return;
	}
	custom_profile_slot_ = slot;
}
int TalonHWCommand::getCustomProfileSlot(void) const
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfileSlot)");
		return -1;
	}
	return custom_profile_slot_;
}

void TalonHWCommand::pushCustomProfilePoint(const CustomProfilePoint &point, size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (pushCustomProfilePoint)");
		return;
	}
	// Make sure there are enough slots allocated to
	// hold the point about to be added
	if (custom_profile_points_.size() <= slot)
		custom_profile_points_.resize(slot + 1);
	if (custom_profile_total_time_.size() <= slot)
		custom_profile_total_time_.resize(slot + 1);

	custom_profile_points_[slot].push_back(point);
	if (custom_profile_points_[slot].size() != 0)
	{
		custom_profile_total_time_[slot].push_back(custom_profile_total_time_[slot].back() + point.duration);
	}
	else
	{
		custom_profile_total_time_[slot].push_back(point.duration);
	}
	//ROS_INFO_STREAM("pushed point at slot: " << slot);
	while (custom_profile_points_changed_.size() <= slot)
		custom_profile_points_changed_.push_back(true);
	custom_profile_points_changed_[slot] = true;
}
void TalonHWCommand::pushCustomProfilePoints(const std::vector<CustomProfilePoint> &points, size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (pushCustomProfilePoints)");
		return;
	}

	if (!points.size())
		return;

	// Make sure there are enough slots allocated to
	// hold the point about to be added
	if (custom_profile_points_.size() <= slot)
		custom_profile_points_.resize(slot + 1);
	if (custom_profile_total_time_.size() <= slot)
		custom_profile_total_time_.resize(slot + 1);

	size_t prev_size = custom_profile_points_[slot].size();
	custom_profile_points_[slot].insert(custom_profile_points_[slot].end(), points.begin(), points.end());
	for (; prev_size < custom_profile_points_.size(); prev_size++)
	{
		if (prev_size != 0)
		{
			custom_profile_total_time_[slot].push_back(points[prev_size].duration + custom_profile_total_time_[slot][prev_size - 1]);
		}
		else
		{
			custom_profile_total_time_[slot].push_back(points[prev_size].duration);
		}
	}

	while (custom_profile_points_changed_.size() <= slot)
		custom_profile_points_changed_.push_back(true);
	custom_profile_points_changed_[slot] = true;
	//ROS_INFO_STREAM("pushed points at slot: " << slot);
}

void TalonHWCommand::overwriteCustomProfilePoints(const std::vector<CustomProfilePoint> &points, size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (overwriteCustomProfilePoints)");
		return;
	}

#if 0
	for (size_t i = 0; i < custom_profile_points_changed_.size(); i++)
	{
		ROS_INFO_STREAM("slot: " << i << " changed: " << custom_profile_points_changed_[i]);
	}
#endif

	// Make sure there are enough slots allocated to
	// hold the point about to be added
	if (custom_profile_points_.size() <= slot)
		custom_profile_points_.resize(slot + 1);
	if (custom_profile_total_time_.size() <= slot)
		custom_profile_total_time_.resize(slot + 1);

	custom_profile_points_[slot] = points;
	custom_profile_total_time_[slot].resize(points.size());

	for (size_t i = 0; i < points.size(); i++)
	{
		if (i != 0)
		{
			custom_profile_total_time_[slot][i] = points[i].duration + custom_profile_total_time_[slot][i - 1];
		}
		else
		{
			custom_profile_total_time_[slot][i] = points[i].duration;
		}
	}
	ROS_INFO_STREAM("override points at slot: " << slot);
	while (custom_profile_points_changed_.size() <= slot)
		custom_profile_points_changed_.push_back(true);
	custom_profile_points_changed_[slot] = true;
}

std::vector<CustomProfilePoint> TalonHWCommand::getCustomProfilePoints(size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfilePoints)");
		return std::vector<CustomProfilePoint>();
	}

	if (slot >= custom_profile_points_.size())
		return std::vector<CustomProfilePoint>();

	return custom_profile_points_[slot];
}

std::vector<bool> TalonHWCommand::getCustomProfilePointsTimesChanged(std::vector<std::vector<CustomProfilePoint>> &ret_points, std::vector<std::vector<double>> &ret_times)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfilePointsTimesChanged)");
		return std::vector<bool>();
	}
	std::vector<bool> returner = custom_profile_points_changed_;

	bool args_resized = false;
	// Make sure there are enough slots allocated to
	// hold the point about to be added
	const size_t slots = custom_profile_points_changed_.size();
	if (ret_points.size() != slots)
	{
		args_resized = true;
		ret_points.resize(slots);
	}
	if (ret_times.size() != slots)
	{
		args_resized = true;
		ret_times.resize(slots);
	}

	for (size_t i = 0; i < slots; i++)
	{
		if (args_resized || custom_profile_points_changed_[i])
		{
			if (args_resized)
				returner[i] = true;
			//ROS_INFO_STREAM("actually changed in interface " << custom_profile_points_changed_[i] << " slot: " << i);
			ret_points[i] = custom_profile_points_[i];
			ret_times[i]  = custom_profile_total_time_[i];
			custom_profile_points_changed_[i] = false;
		}
	}

	return returner;
}
std::vector<double> TalonHWCommand::getCustomProfileTime(size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfileTime)");
		return std::vector<double>();
	}

	if (slot >= custom_profile_total_time_.size())
		return std::vector<double>();

	return custom_profile_total_time_[slot];
}
double TalonHWCommand::getCustomProfileEndTime(size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfileEndTime)");
		return -1;
	}

	if (slot >= custom_profile_total_time_.size())
		return -1;

	return custom_profile_total_time_[slot].back();
}
// TODO : TimeCount and ProfileCount should always
// be the same?
size_t TalonHWCommand::getCustomProfileTimeCount(size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfileTimeCount)");
		return 0;
	}

	if (slot >= custom_profile_points_.size())
		return 0;

	return custom_profile_points_[slot].size();
}
size_t TalonHWCommand::getCustomProfileCount(size_t slot)
{
	if (custom_profile_disable_)
	{
		ROS_ERROR("Custom profile disabled via param (getCustomProfileCount)");
		return 0;
	}

	if (slot >= custom_profile_points_.size())
		return 0;

	return custom_profile_points_[slot].size();
}

void TalonHWCommand::setEnableReadThread(bool enable_read_thread)
{
	enable_read_thread_ = enable_read_thread;
	enable_read_thread_changed_ = true;
}
bool TalonHWCommand::getEnableReadThread(void) const
{
	return enable_read_thread_;
}

bool TalonHWCommand::enableReadThreadChanged(bool &enable_read_thread)
{
	enable_read_thread = enable_read_thread_;
	if (!enable_read_thread_changed_)
		return false;
	enable_read_thread_changed_ = false;
	return true;
}

void TalonHWCommand::lock(void)
{
	mutex_->lock();
}
bool TalonHWCommand::try_lock(void)
{
	return mutex_->try_lock();
}
void TalonHWCommand::unlock(void)
{
	mutex_->unlock();
}

} // namespace
