#pragma once

#include <mutex>
#include <string>
#include <talon_interface/talon_state_interface.h>

namespace hardware_interface
{

//Below struct contains all the info we want for our profile
//Later it might include some more complex settings (current limits?, peak output limits?,
//some f params to calc on the fly based on sensor data?)
struct CustomProfilePoint
{
	CustomProfilePoint() :
		mode(TalonMode_Position),
		pidSlot(0),
		setpoint(0.0),
		fTerm(0),
		duration(0),
		zeroPos(false)
	{
	}
	TalonMode mode;
	int pidSlot;
	double setpoint;
	double fTerm;
	double duration;
	bool zeroPos;
};

struct TrajectoryPoint
{
	// Sane? defaults
	TrajectoryPoint()
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
	double   position;
	double   velocity;
	double   headingRad;
	double   arbFeedFwd;
	double   auxiliaryPos;
	double   auxiliaryVel;
	double   auxiliaryArbFeedFwd;
	uint32_t profileSlotSelect0;
	uint32_t profileSlotSelect1;
	bool     isLastPoint;
	bool     zeroPos;
	int      timeDur;
	bool     useAuxPID;
};

// Class to buffer data needed to set the state of the
// Talon.  This should (eventually) include anything
// which might be set during runtime.  Config data
// which is set only once at startup can be handled
// in the hardware manager constructor/init rather than through
// this interface.
// Various controller code will set the member vars of
// this class depending on the needs of the motor
// being controlled
// Each pass through write() in the hardware interface
// will use this to re-configure (if necessary) and then
// update the setpoint on the associated Talon.
// The hardware_controller is responsible for keeping
// a master array of these classes - 1 entry per
// physical Talon controller in the robot
class TalonHWCommand
{
	public:
		// Set up default values
		// Set most of the changed_ vars to true
		// to force a write of these values to the Talon
		// That should put the talon in a known state
		// rather than relying on them being setup to
		// a certain state previously
		TalonHWCommand(void) :
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
			neutral_deadband_(41./1023.),
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

			// current limiting
			current_limit_peak_amps_(0),
			current_limit_peak_msec_(10), // see https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/README.md#motor-output-direction-is-incorrect-or-accelerates-when-current-limit-is-enabled
			current_limit_continuous_amps_(0),
			current_limit_enable_(false),
			current_limit_changed_(true),

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

		// This gets the requested setpoint, not the
		// status actually read from the controller
		// Need to think about which makes the most
		// sense to query...
		bool commandChanged(double &command)
		{
			command = command_;
			if (!command_changed_)
				return false;
			command_changed_ = false;
			return true;
		}
		double get(void) const
		{
			return command_;
		}

		TalonMode getMode(void) const
		{
			return mode_;
		}

		void setP(double oldP, size_t index)
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
		double getP(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getP()");
				return 0.0;
			}
			return p_[index];
		}

		void setI(double ii, size_t index)
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
		double getI(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getI()");
				return 0.0;
			}
			return i_[index];
		}

		void setD(double dd, size_t index)
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
		double getD(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getD()");
				return 0.0;
			}
			return d_[index];
		}

		void setF(double ff, size_t index)
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
		double getF(size_t index)
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getF()");
				return 0.0;
			}
			return f_[index];
		}

		void setIZ(int i_zone, size_t index)
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
		int getIZ(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getIZ()");
				return 0.0;
			}
			return i_zone_[index];
		}

		void setAllowableClosedloopError(int allowable_closed_loop_error, size_t index)
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
		int getAllowableClosedloopError(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
				return 0;
			}
			return allowable_closed_loop_error_[index];
		}
		void setMaxIntegralAccumulator(int max_integral_accumulator, size_t index)
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
		int getMaxIntegralAccumulator(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getAllowableClosedLoopErrro()");
				return 0.0;
			}
			return max_integral_accumulator_[index];
		}
		void setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index)
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
		double getClosedLoopPeakOutput(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getClosedLoopPeakOutput()");
				return 0.0;
			}
			return closed_loop_peak_output_[index];
		}

		void setClosedLoopPeriod(int closed_loop_period, size_t index)
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
		int getClosedLoopPeriod(size_t index) const
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::getClosedLoopPeriod()");
				return 0.0;
			}
			return closed_loop_period_[index];
		}
		bool pidfChanged(double &p, double &i, double &d, double &f, int &iz, int &allowable_closed_loop_error, double &max_integral_accumulator, double &closed_loop_peak_output, int &closed_loop_period, size_t index)
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
		void resetPIDF(size_t index)
		{
			if (index >= TALON_PIDF_SLOTS)
			{
				ROS_WARN("Invalid index passed to TalonHWCommand::resetPIDF()");
				return;
			}
			pidf_changed_[index] = true;
		}

		void setAuxPidPolarity(bool aux_pid_polarity)
		{
			if (aux_pid_polarity_ != aux_pid_polarity)
			{
				aux_pid_polarity_ = aux_pid_polarity;
				aux_pid_polarity_changed_ = true;
			}
		}
		bool getAuxPidPolarity(void) const
		{
			return aux_pid_polarity_;
		}
		bool auxPidPolarityChanged(bool &aux_pid_polarity)
		{
			aux_pid_polarity = aux_pid_polarity_;
			if (!aux_pid_polarity_changed_)
				return false;
			aux_pid_polarity_changed_ = false;
			return true;
		}
		void resetAuxPidPolarity(void)
		{
			aux_pid_polarity_changed_ = true;
		}

		void setIntegralAccumulator(double iaccum)
		{
			iaccum_ = iaccum;
			iaccum_changed_ = true;
		}
		double getIntegralAccumulator(void) const
		{
			return iaccum_;
		}
		bool integralAccumulatorChanged(double &iaccum)
		{
			iaccum = iaccum_;
			if (!iaccum_changed_)
				return false;
			iaccum_changed_ = false;
			return true;
		}
		void resetIntegralAccumulator(void)
		{
			iaccum_changed_ = true;
		}

		void set(double command)
		{
			command_ = command;
			if (fabs(command_ - command) > double_value_epsilon)
				command_changed_ = true;
		}

		void setMode(TalonMode mode)
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
		bool modeChanged(TalonMode &mode)
		{
			mode = mode_;
			if (!mode_changed_)
				return false;
			mode_changed_ = false;
			return true;
		}
		void resetMode(void)
		{
			mode_changed_ = true;
		}

		void setDemand1Type(DemandType demand_type)
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
		DemandType getDemand1Type(void) const
		{
			return demand1_type_;
		}

		void setDemand1Value(double demand_value)
		{
			if (fabs(demand1_value_ - demand_value) > double_value_epsilon)
			{
				demand1_value_   = demand_value;
				demand1_changed_ = true;
			}
		}

		void resetDemand1(void)
		{
			demand1_changed_ = true;
		}

		double getDemand1Value(void) const
		{
			return demand1_value_;
		}
		bool demand1Changed(DemandType &type, double &value)
		{
			type  = demand1_type_;
			value = demand1_value_;
			if (!demand1_changed_)
				return false;
			demand1_changed_ = false;
			return true;
		}

		void setNeutralMode(NeutralMode neutral_mode)
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
		bool getNeutralMode(void)
		{
			return neutral_mode_;
		}
		bool neutralModeChanged(NeutralMode &neutral_mode)
		{
			neutral_mode = neutral_mode_;
			if (!neutral_mode_changed_)
				return false;
			neutral_mode_changed_ = false;
			return true;
		}

		void setNeutralOutput(void)
		{
			neutral_output_ = true;
		}
		// Set motor controller to neutral output
		// This should be a one-shot ... only
		// write it to the motor controller once
		bool neutralOutputChanged(void)
		{
			if (!neutral_output_)
				return false;
			neutral_output_ = false;
			return true;
		}

		void setPidfSlot(int pidf_slot)
		{
			if (pidf_slot != pidf_slot_)
			{
				pidf_slot_ = pidf_slot;
				pidf_slot_changed_ = true;
			}
		}
		int getPidfSlot(void)const
		{
			return pidf_slot_;
		}
		bool slotChanged(int &newpidfSlot)
		{
			newpidfSlot = pidf_slot_;
			if (!pidf_slot_changed_)
				return false;
			pidf_slot_changed_ = false;
			return true;
		}
		void resetPidfSlot(void)
		{
			pidf_slot_changed_ = true;
		}

		void setInvert(bool invert)
		{
			if (invert != invert_)
			{
				invert_ = invert;
				invert_changed_ = true;
			}
		}
		void setSensorPhase(bool invert)
		{
			if (invert != sensor_phase_)
			{
				sensor_phase_ = invert;
				invert_changed_ = true;
			}
		}
		bool invertChanged(bool &invert, bool &sensor_phase)
		{
			invert = invert_;
			sensor_phase = sensor_phase_;
			if (!invert_changed_)
				return false;
			invert_changed_ = false;
			return true;
		}

		FeedbackDevice getEncoderFeedback(void) const
		{
			return encoder_feedback_;
		}
		void setEncoderFeedback(FeedbackDevice encoder_feedback)
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
		double getFeedbackCoefficient(void) const
		{
			return feedback_coefficient_;
		}
		void setFeedbackCoefficient(double feedback_coefficient)
		{
			if (fabs(feedback_coefficient - feedback_coefficient_) > double_value_epsilon)
			{
				feedback_coefficient_ = feedback_coefficient;
				encoder_feedback_changed_ = true;
			}
		}
		bool encoderFeedbackChanged(FeedbackDevice &encoder_feedback, double &feedback_coefficient)
		{
			encoder_feedback = encoder_feedback_;
			feedback_coefficient = feedback_coefficient_;
			if (!encoder_feedback_changed_)
				return false;
			encoder_feedback_changed_ = false;
			return true;
		}
		void resetEncoderFeedback(void)
		{
			encoder_feedback_changed_ = true;
		}

		RemoteFeedbackDevice getRemoteEncoderFeedback(void) const
		{
			return remote_encoder_feedback_;
		}
		void setRemoteEncoderFeedback(RemoteFeedbackDevice remote_encoder_feedback)
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
		bool remoteEncoderFeedbackChanged(RemoteFeedbackDevice &remote_encoder_feedback)
		{
			remote_encoder_feedback = remote_encoder_feedback_;
			if (!remote_encoder_feedback_changed_)
				return false;
			remote_encoder_feedback_changed_ = false;
			return true;
		}
		void resetRemoteEncoderFeedback(void)
		{
			remote_encoder_feedback_changed_ = true;
		}
		int getRemoteFeedbackDeviceId(size_t remote_ordinal) const
		{
			if (remote_ordinal >= 2)
			{
				ROS_WARN("getRemoteFeedbackDeviceId: remote_ordinal too large");
				return -1;
			}
			return remote_feedback_device_ids_[remote_ordinal];
		}
		RemoteSensorSource getRemoteFeedbackFilter(size_t remote_ordinal) const
		{
			if (remote_ordinal >= 2)
			{
				ROS_WARN("getRemoteFeedbackFilter : remote_ordinal too large");
				return RemoteSensorSource_Last;
			}
			return remote_feedback_filters_[remote_ordinal];
		}

		void setRemoteFeedbackDeviceId(int remote_feedback_device_id, size_t remote_ordinal)
		{
			if (remote_ordinal >= 2)
			{
				ROS_WARN("setRemoteFeedbackFilter : remote_ordinal too large");
				return;
			}
			remote_feedback_device_ids_[remote_ordinal] = remote_feedback_device_id;
			remote_feedback_filters_changed_ = true;
		}
		void setRemoteFeedbackFilter(RemoteSensorSource remote_sensor_source, size_t remote_ordinal)
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
		bool remoteFeedbackFiltersChanged(std::array<int, 2> &remote_feedback_device_ids, std::array<RemoteSensorSource, 2> &remote_feedback_filters)
		{
			remote_feedback_device_ids = remote_feedback_device_ids_;
			remote_feedback_filters = remote_feedback_filters_;
			if (!remote_feedback_filters_changed_)
				return false;
			remote_feedback_filters_changed_ = false;
			return true;
		}
		void resetRemoteFeedbackFilters(void)
		{
			remote_feedback_filters_changed_ = true;
		}

		FeedbackDevice getSensorTerm(SensorTerm sensor_terms) const
		{
			if (sensor_terms < SensorTerm_Last)
				return sensor_terms_[sensor_terms];
			ROS_WARN("getSensorTerm : sensor_terms index too large");
			return FeedbackDevice_Last;
		}
		void setSensorTerm(FeedbackDevice feedback_device, SensorTerm sensor_terms)
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
		bool sensorTermsChanged(std::array<FeedbackDevice, SensorTerm_Last> &sensor_terms)
		{
			sensor_terms = sensor_terms_;
			if (!sensor_terms_changed_)
				return false;
			sensor_terms_changed_ = false;
			return true;
		}
		void resetSensorTerms(void)
		{
			sensor_terms_changed_ = true;
		}

		int getEncoderTicksPerRotation(void) const
		{
			return encoder_ticks_per_rotation_;
		}

		void setEncoderTicksPerRotation(int encoder_ticks_per_rotation)
		{
			encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
		}

		//output shaping
		void setClosedloopRamp(double closed_loop_ramp)
		{
			if (fabs(closed_loop_ramp_ - closed_loop_ramp) > double_value_epsilon)
			{
				closed_loop_ramp_ = closed_loop_ramp;
				output_shaping_changed_ = true;
			}
		}
		double getClosedloopRamp(void) const
		{
			return closed_loop_ramp_;
		}
		void setOpenloopRamp(double open_loop_ramp)
		{
			if (fabs(open_loop_ramp_ - open_loop_ramp) > double_value_epsilon)
			{
				open_loop_ramp_ = open_loop_ramp;
				output_shaping_changed_ = true;
			}
		}
		double getOpenloopRamp(void) const
		{
			return open_loop_ramp_;
		}

		void setPeakOutputForward(double peak_output_forward)
		{
			if (fabs(peak_output_forward - peak_output_forward_) > double_value_epsilon)
			{
				peak_output_forward_ = peak_output_forward;
				output_shaping_changed_ = true;
			}
		}
		double getPeakOutputForward(void) const
		{
			return peak_output_forward_;
		}

		void setPeakOutputReverse(double peak_output_reverse)
		{
			if (fabs(peak_output_reverse - peak_output_reverse_) > double_value_epsilon)
			{
				peak_output_reverse_ = peak_output_reverse;
				output_shaping_changed_ = true;
			}
		}
		double getPeakOutputReverse(void) const
		{
			return peak_output_reverse_;
		}

		void setNominalOutputForward(double nominal_output_forward)
		{
			if (fabs(nominal_output_forward - nominal_output_forward_) > double_value_epsilon)
			{
				nominal_output_forward_ = nominal_output_forward;
				output_shaping_changed_ = true;
			}
		}
		double getNominalOutputForward(void) const
		{
			return nominal_output_forward_;
		}

		void setNominalOutputReverse(double nominal_output_reverse)
		{
			if (fabs(nominal_output_reverse - nominal_output_reverse_) > double_value_epsilon)
			{
				nominal_output_reverse_ = nominal_output_reverse;
				output_shaping_changed_ = true;
			}
		}
		double getNominalOutputReverse(void) const
		{
			return nominal_output_reverse_;
		}

		void setNeutralDeadband(double neutral_deadband)
		{
			if (fabs(neutral_deadband - neutral_deadband_) > double_value_epsilon)
			{
				neutral_deadband_ = neutral_deadband;
				output_shaping_changed_ = true;
			}
		}
		double getNeutralDeadband(void) const
		{
			return neutral_deadband_;
		}
		bool outputShapingChanged(double &closed_loop_ramp,
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
		void resetOutputShaping(void)
		{
			output_shaping_changed_ = true;
		}

		void setVoltageCompensationSaturation(double voltage)
		{
			if (fabs(voltage - voltage_compensation_saturation_) > double_value_epsilon)
			{
				voltage_compensation_saturation_ = voltage;
				voltage_compensation_changed_ = true;
			}
		}
		double getVoltageCompensationSaturation(void) const
		{
			return voltage_compensation_saturation_;
		}

		void setVoltageMeasurementFilter(int filterWindowSamples)
		{
			if (filterWindowSamples != voltage_measurement_filter_)
			{
				voltage_measurement_filter_ = filterWindowSamples;
				voltage_compensation_changed_ = true;
			}
		}
		int getVoltageMeasurementFilter(void) const
		{
			return voltage_compensation_saturation_;
		}

		void setVoltageCompensationEnable(bool enable)
		{
			if (enable != voltage_compensation_enable_)
			{
				voltage_compensation_enable_ = enable;
				voltage_compensation_changed_ = true;
			}
		}

		bool getEnableVoltageCompenation(void) const
		{
			return voltage_compensation_enable_;
		}

		bool voltageCompensationChanged(double &voltage_compensation_saturation,
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
		void resetVoltageCompensation(void)
		{
			voltage_compensation_changed_ = true;
		}

		void setVelocityMeasurementPeriod(hardware_interface::VelocityMeasurementPeriod period)
		{
			if (period != velocity_measurement_period_)
			{
				velocity_measurement_period_ = period;
				velocity_measurement_changed_ = true;
			}
		}

		bool getVoltageMeasurementPeriod(void) const
		{
			return velocity_measurement_period_;
		}

		void setVelocityMeasurementWindow(int window)
		{
			if (window != velocity_measurement_window_)
			{
				velocity_measurement_window_ = window;
				velocity_measurement_changed_ = true;
			}
		}

		bool getVoltageMeasurementWindow(void) const
		{
			return velocity_measurement_window_;
		}

		bool velocityMeasurementChanged(hardware_interface::VelocityMeasurementPeriod &period,
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
		void resetVelocityMeasurement(void)
		{
			velocity_measurement_changed_ = true;
		}

		void setSelectedSensorPosition(double position)
		{
			sensor_position_value_ = position;
			sensor_position_changed_ = true;
		}
		double getSelectedSensorPosition(void) const
		{
			return sensor_position_value_;
		}

		bool sensorPositionChanged(double &position)
		{
			position = sensor_position_value_;
			if (!sensor_position_changed_)
				return false;
			sensor_position_changed_ = false;
			return true;
		}
		void resetSensorPosition(void)
		{
			sensor_position_changed_ = true;
		}


		void setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
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

		void getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_forward_source_;
			normal = limit_switch_local_forward_normal_;
		}

		void setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
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

		void getReverseLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_reverse_source_;
			normal = limit_switch_local_reverse_normal_;
		}

		bool limitSwitchesSourceChanged(LimitSwitchSource &forward_source, LimitSwitchNormal &forward_normal, LimitSwitchSource &reverse_source, LimitSwitchNormal &reverse_normal)
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
		void resetLimitSwitchesSource(void)
		{
			limit_switch_local_changed_ = true;
		}

		void setRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
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

		void getRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
		{
			source = limit_switch_remote_forward_source_;
			normal = limit_switch_remote_forward_normal_;
			id     = limit_switch_remote_forward_id_;
		}

		void setRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
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

		void getRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
		{
			source = limit_switch_remote_reverse_source_;
			normal = limit_switch_remote_reverse_normal_;
			id     = limit_switch_remote_reverse_id_;
		}

		bool remoteLimitSwitchesSourceChanged(RemoteLimitSwitchSource &forward_source, LimitSwitchNormal &forward_normal, unsigned int &forward_id, RemoteLimitSwitchSource &reverse_source, LimitSwitchNormal &reverse_normal, unsigned int &reverse_id)
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
		void resetRemoteLimitSwitchesSource(void)
		{
			limit_switch_remote_changed_ = true;
		}

		// softlimits
		void setForwardSoftLimitThreshold(double threshold)
		{
			if (fabs(threshold - softlimit_forward_threshold_) > double_value_epsilon)
			{
				softlimit_forward_threshold_ = threshold;
				softlimit_changed_ = true;
			}
		}
		double getForwardSoftLimitThreshold(void) const
		{
			return softlimit_forward_threshold_;
		}

		void setForwardSoftLimitEnable(bool enable)
		{
			if (enable != softlimit_forward_enable_)
			{
				softlimit_forward_enable_ = enable;
				softlimit_changed_ = true;
			}
		}
		bool getForwardSoftLimitEnable(void) const
		{
			return softlimit_forward_enable_;
		}
		void setReverseSoftLimitThreshold(double threshold)
		{
			if (fabs(threshold - softlimit_reverse_threshold_) > double_value_epsilon)
			{
				softlimit_reverse_threshold_ = threshold;
				softlimit_changed_ = true;
			}
		}
		double getReverseSoftLimitThreshold(void) const
		{
			return softlimit_reverse_threshold_;
		}

		void setReverseSoftLimitEnable(bool enable)
		{
			if (enable != softlimit_reverse_enable_)
			{
				softlimit_reverse_enable_ = enable;
				softlimit_changed_ = true;
			}
		}
		bool getReverseSoftLimitEnable(void) const
		{
			return softlimit_reverse_enable_;
		}

		void setOverrideSoftLimitsEnable(bool enable)
		{
			if (enable != softlimits_override_enable_)
			{
				softlimits_override_enable_ = enable;
				softlimit_changed_ = true;
			}
		}
		bool getOverrideSoftsLimitEnable(void) const
		{
			return softlimits_override_enable_;
		}

		bool softLimitChanged(double &forward_threshold, bool &forward_enable, double &reverse_threshold, bool &reverse_enable, bool &override_enable)
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
		void resetSoftLimit(void)
		{
			softlimit_changed_ = true;
		}

		// current limits
		void setPeakCurrentLimit(int amps)
		{
			if (amps != current_limit_peak_amps_)
			{
				current_limit_peak_amps_ = amps;
				current_limit_changed_ = true;
			}
		}
		int getPeakCurrentLimit(void) const
		{
			return current_limit_peak_amps_;
		}

		void setPeakCurrentDuration(int msec)
		{
			if (msec != current_limit_peak_msec_)
			{
				current_limit_peak_msec_ = msec;
				current_limit_changed_ = true;
			}
		}
		int getPeakCurrentDuration(void) const
		{
			return current_limit_peak_msec_;
		}
		void setContinuousCurrentLimit(int amps)
		{
			if (amps != current_limit_continuous_amps_)
			{
				current_limit_continuous_amps_ = amps;
				current_limit_changed_ = true;
			}
		}
		int getContinuousCurrentLimit(void) const
		{
			return current_limit_continuous_amps_;
		}
		void setCurrentLimitEnable(bool enable)
		{
			if (enable != current_limit_enable_)
			{
				current_limit_enable_ = enable;
				current_limit_changed_ = true;
			}
		}
		bool getCurrentLimitEnable(void) const
		{
			return current_limit_enable_;
		}

		bool currentLimitChanged(int &peak_amps, int &peak_msec, int &continuous_amps, bool &enable)
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
		void resetCurrentLimit(void)
		{
			current_limit_changed_ = false;
		}

		void setMotionCruiseVelocity(double velocity)
		{
			if (fabs(velocity - motion_cruise_velocity_) > double_value_epsilon)
			{
				motion_cruise_velocity_ = velocity;
				motion_cruise_changed_ = true;
			}
		}
		double getMotionCruiseVelocity(void) const
		{
			return motion_cruise_velocity_;
		}
		void setMotionAcceleration(double acceleration)
		{
			if (fabs(acceleration - motion_acceleration_) > double_value_epsilon)
			{
				motion_acceleration_ = acceleration;
				motion_cruise_changed_ = true;
			}
		}
		double getMotionAcceleration(void) const
		{
			return motion_acceleration_;
		}
		void setMotionSCurveStrength(unsigned int s_curve_strength)
		{
			if ((s_curve_strength != motion_s_curve_strength_) &&
				(s_curve_strength <= 8))
			{
				motion_s_curve_strength_ = s_curve_strength;
				motion_cruise_changed_ = true;
			}
		}
		unsigned int getMotionSCurveStrength(void) const
		{
			return motion_s_curve_strength_;
		}

		bool motionCruiseChanged(double &velocity, double &acceleration, unsigned int &s_curve_strength)
		{
			velocity = motion_cruise_velocity_;
			acceleration = motion_acceleration_;
			s_curve_strength = motion_s_curve_strength_;
			if (!motion_cruise_changed_)
				return false;
			motion_cruise_changed_ = false;
			return true;
		}
		void resetMotionCruise(void)
		{
			motion_cruise_changed_ = true;
		}

		// This is a one shot - when set, it needs to
		// call the appropriate Talon function once
		// then clear itself
		void setClearMotionProfileTrajectories(void)
		{
			motion_profile_clear_trajectories_ = true;
		}
		bool getClearMotionProfileTrajectories(void) const
		{
			return motion_profile_clear_trajectories_;
		}
		bool clearMotionProfileTrajectoriesChanged(void)
		{
			if (!motion_profile_clear_trajectories_)
				return false;
			motion_profile_clear_trajectories_ = false;
			return true;
		}
		void PushMotionProfileTrajectory(const TrajectoryPoint &traj_pt)
		{
			motion_profile_trajectory_points_.push_back(traj_pt);
		}
		std::vector<TrajectoryPoint> getMotionProfileTrajectories(void) const
		{
			return motion_profile_trajectory_points_;
		}
		bool motionProfileTrajectoriesChanged(std::vector<TrajectoryPoint> &points)
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
		void setClearMotionProfileHasUnderrun(void)
		{
			motion_profile_clear_has_underrun_ = true;
		}
		bool getClearMotionProfileHasUnderrun(void) const
		{
			return motion_profile_clear_has_underrun_;
		}
		bool clearMotionProfileHasUnderrunChanged(void)
		{
			if (!motion_profile_clear_has_underrun_)
				return false;
			motion_profile_clear_has_underrun_ = false;
			return true;
		}

		void setStatusFramePeriod(StatusFrame status_frame, uint8_t period)
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

		uint8_t getStatusFramePeriod(StatusFrame status_frame) const
		{
			if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
				return status_frame_periods_[status_frame];

			ROS_ERROR("Invalid status_frame value passed to TalonHWCommand::setStatusFramePeriod()");
			return 0;
		}

		bool statusFramePeriodChanged(StatusFrame status_frame, uint8_t &period)
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
		void resetStatusFramePeriod(StatusFrame status_frame)
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

		void setControlFramePeriod(ControlFrame control_frame, uint8_t period)
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

		uint8_t getControlFramePeriod(ControlFrame control_frame) const
		{
			if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
				return control_frame_periods_[control_frame];

			ROS_ERROR("Invalid control_frame value passed to TalonHWCommand::setControlFramePeriod()");
			return 0;
		}

		bool controlFramePeriodChanged(ControlFrame control_frame, uint8_t &period)
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
		void resetControlFramePeriod(ControlFrame control_frame)
		{
			if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
				control_frame_periods_changed_[control_frame] = true;
			else
				ROS_ERROR("Invalid control_frame value passed to TalonHWCommand::resetControlFramePeriod()");
		}

		void setMotionProfileTrajectoryPeriod(int msec)
		{
			if (msec != motion_profile_profile_trajectory_period_)
			{
				motion_profile_profile_trajectory_period_ = msec;
				motion_profile_profile_trajectory_period_changed_ = true;
			}
		}
		int getMotionProfileTrajectoryPeriod(void) const
		{
			return motion_profile_profile_trajectory_period_;
		}
		bool motionProfileTrajectoryPeriodChanged(int &msec)
		{
			msec = motion_profile_profile_trajectory_period_;
			if (!motion_profile_profile_trajectory_period_changed_)
				return false;
			motion_profile_profile_trajectory_period_changed_ = false;
			return true;
		}
		void resetMotionProfileTrajectoryPeriod(void)
		{
			motion_profile_profile_trajectory_period_changed_ = true;
		}

		void setClearStickyFaults(void)
		{
			clear_sticky_faults_ = true;
		}
		bool getClearStickyFaults(void) const
		{
			return clear_sticky_faults_;
		}
		bool clearStickyFaultsChanged(void)
		{
			if (!clear_sticky_faults_)
				return false;
			clear_sticky_faults_ = false;
			return true;
		}

		void setConversionFactor(double conversion_factor)
		{
			if (fabs(conversion_factor - conversion_factor_) > double_value_epsilon)
			{
				conversion_factor_ = conversion_factor;
				conversion_factor_changed_ = true;
			}
		}
		double getConversionFactor(void) const
		{
			return conversion_factor_;
		}
		bool conversionFactorChanged(double &conversion_factor)
		{
			conversion_factor = conversion_factor_;
			if (!conversion_factor_changed_)
				return false;
			conversion_factor_changed_ = false;
			return true;
		}

		void setCustomProfileDisable(bool disable)
		{
			custom_profile_disable_ = disable;
		}

		bool getCustomProfileDisable(void) const
		{
			return custom_profile_disable_;
		}

		std::vector<int> getCustomProfileNextSlot(void) const
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (getCustomProfileNextSlot)");
				return std::vector<int>();
			}
			return custom_profile_next_slot_;
		}
		void setCustomProfileNextSlot(const std::vector<int> &next_slot)
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (setCustomProfileNextSlot)");
				return;
			}
			custom_profile_next_slot_ = next_slot;
		}
		double getCustomProfileHz(void) const
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (getCustomProfileHz)");
				return -1;
			}
			return custom_profile_hz_;
		}
		void setCustomProfileHz(const double &hz)
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (setCustomProfileHz)");
				return;
			}
			custom_profile_hz_ = hz;
		}
		void setCustomProfileRun(const bool &run)
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (setCustomProfileRun)");
				return;
			}
			custom_profile_run_ = run;
		}
		bool getCustomProfileRun(void)
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
		void setCustomProfileSlot(const int &slot)
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (setCustomProfileSlot)");
				return;
			}
			custom_profile_slot_ = slot;
		}
		int getCustomProfileSlot(void) const
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (getCustomProfileSlot)");
				return -1;
			}
			return custom_profile_slot_;
		}

		void pushCustomProfilePoint(const CustomProfilePoint &point, size_t slot)
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
			if(custom_profile_points_[slot].size() != 0)
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
		void pushCustomProfilePoints(const std::vector<CustomProfilePoint> &points, size_t slot)
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
			for(; prev_size < custom_profile_points_.size(); prev_size++)
			{
				if(prev_size != 0)
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

		void overwriteCustomProfilePoints(const std::vector<CustomProfilePoint> &points, size_t slot)
		{
			if (custom_profile_disable_)
			{
				ROS_ERROR("Custom profile disabled via param (overwriteCustomProfilePoints)");
				return;
			}

#if 0
			for(size_t i = 0; i < custom_profile_points_changed_.size(); i++)
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

			for(size_t i = 0; i < points.size(); i++)
			{
				if(i != 0)
				{
					custom_profile_total_time_[slot][i] = points[i].duration + custom_profile_total_time_[slot][i-1];
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

		std::vector<CustomProfilePoint> getCustomProfilePoints(size_t slot)
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

		std::vector<bool> getCustomProfilePointsTimesChanged(std::vector<std::vector<CustomProfilePoint>> &ret_points, std::vector<std::vector<double>> &ret_times)
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

			for(size_t i = 0; i < slots; i++)
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
		std::vector<double> getCustomProfileTime(size_t slot)
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
		double getCustomProfileEndTime(size_t slot)
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
		size_t getCustomProfileTimeCount(size_t slot)
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
		size_t getCustomProfileCount(size_t slot)
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

		void setEnableReadThread(bool enable_read_thread)
		{
			enable_read_thread_ = enable_read_thread;
			enable_read_thread_changed_ = true;
		}
		bool getEnableReadThread(void) const
		{
			return enable_read_thread_;
		}

		bool enableReadThreadChanged(bool &enable_read_thread)
		{
			enable_read_thread = enable_read_thread_;
			if (!enable_read_thread_changed_)
				return false;
			enable_read_thread_changed_ = false;
			return true;
		}

		void lock(void)
		{
			mutex_->lock();
		}
		bool try_lock(void)
		{
			return mutex_->try_lock();
		}
		void unlock(void)
		{
			mutex_->unlock();
		}

	private:
		double    command_; // motor setpoint - % vbus, velocity, position, etc
		bool      command_changed_;
		TalonMode mode_;         // talon mode - % vbus, close loop, motion profile, etc
		bool      mode_changed_; // set if mode needs to be updated on the talon hw
		DemandType demand1_type_;
		double    demand1_value_;
		bool      demand1_changed_;
		//RG: shouldn't there be a variable for the peak voltage limits?
		int       pidf_slot_; // index 0 or 1 of the active PIDF slot
		bool      pidf_slot_changed_; // set to true to trigger a write to PIDF select on Talon
		double    iaccum_;
		bool      iaccum_changed_;

		bool      invert_;
		bool      sensor_phase_;
		bool      invert_changed_;

		NeutralMode neutral_mode_;
		bool        neutral_mode_changed_;
		bool        neutral_output_;

		FeedbackDevice encoder_feedback_;
		double         feedback_coefficient_;
		bool           encoder_feedback_changed_;
		RemoteFeedbackDevice remote_encoder_feedback_;
		bool                 remote_encoder_feedback_changed_;
		int                  encoder_ticks_per_rotation_;
		std::array<int, 2>                remote_feedback_device_ids_;
		std::array<RemoteSensorSource, 2> remote_feedback_filters_;
		bool                              remote_feedback_filters_changed_;
		std::array<FeedbackDevice, SensorTerm_Last> sensor_terms_;
		bool                                        sensor_terms_changed_;

		//output shaping
		double closed_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;
		bool   output_shaping_changed_;

		double voltage_compensation_saturation_;
		int   voltage_measurement_filter_;
		bool  voltage_compensation_enable_;
		bool  voltage_compensation_changed_;

		hardware_interface::VelocityMeasurementPeriod velocity_measurement_period_;
		int velocity_measurement_window_;
		bool velocity_measurement_changed_;

		double sensor_position_value_;
		bool sensor_position_changed_;

		LimitSwitchSource limit_switch_local_forward_source_;
		LimitSwitchNormal limit_switch_local_forward_normal_;
		LimitSwitchSource limit_switch_local_reverse_source_;
		LimitSwitchNormal limit_switch_local_reverse_normal_;
		bool              limit_switch_local_changed_;
		RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		LimitSwitchNormal       limit_switch_remote_forward_normal_;
		unsigned int            limit_switch_remote_forward_id_;
		RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		LimitSwitchNormal       limit_switch_remote_reverse_normal_;
		unsigned int            limit_switch_remote_reverse_id_;
		bool                    limit_switch_remote_changed_;

		double softlimit_forward_threshold_;
		bool softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool softlimit_reverse_enable_;
		bool softlimits_override_enable_;
		bool softlimit_changed_;

		int current_limit_peak_amps_;
		int current_limit_peak_msec_;
		int current_limit_continuous_amps_;
		bool current_limit_enable_;
		bool current_limit_changed_;

		// Talon expects the next two in integral sensorUnitsPer100ms,
		// but at this level we're still dealing with
		// radians/sec (or /sec^2 for acceleration)
		double motion_cruise_velocity_;
		double motion_acceleration_;
		unsigned int motion_s_curve_strength_;
		bool motion_cruise_changed_;

		bool motion_profile_clear_trajectories_;
		bool motion_profile_clear_has_underrun_;
		std::vector<TrajectoryPoint> motion_profile_trajectory_points_;
		int motion_profile_profile_trajectory_period_;
		bool motion_profile_profile_trajectory_period_changed_;
		std::array<uint8_t, Status_Last> status_frame_periods_;
		std::array<bool, Status_Last> status_frame_periods_changed_;

		std::array<uint8_t, Control_Last> control_frame_periods_;
		std::array<bool, Control_Last> control_frame_periods_changed_;

		bool clear_sticky_faults_;

		// 2 entries in the Talon HW for each of these settings
		std::array<double, TALON_PIDF_SLOTS> p_;
		std::array<double, TALON_PIDF_SLOTS> i_;
		std::array<double, TALON_PIDF_SLOTS> d_;
		std::array<double, TALON_PIDF_SLOTS> f_;
		std::array<int, TALON_PIDF_SLOTS>    i_zone_;
		std::array<int, TALON_PIDF_SLOTS>    allowable_closed_loop_error_;
		std::array<double, TALON_PIDF_SLOTS> max_integral_accumulator_;
		std::array<double, TALON_PIDF_SLOTS> closed_loop_peak_output_;
		std::array<int, TALON_PIDF_SLOTS>    closed_loop_period_;
		std::array<bool, TALON_PIDF_SLOTS>   pidf_changed_;
		bool   aux_pid_polarity_;
		bool   aux_pid_polarity_changed_;

		double conversion_factor_;
		bool   conversion_factor_changed_;

		bool custom_profile_disable_;
		bool custom_profile_run_;
		int custom_profile_slot_;
		std::vector<int> custom_profile_next_slot_;
		double custom_profile_hz_;

		std::vector<std::vector<CustomProfilePoint>> custom_profile_points_;
		std::vector<std::vector<double>> custom_profile_total_time_;

		std::vector<bool> custom_profile_points_changed_;

		bool enable_read_thread_;
		bool enable_read_thread_changed_;

		// Normally the read-update-write process will lead to
		// sequential operation.  controller init happens asynchronously,
		// though, so lock the command entry for a given talon when
		// that talon's controller is being initialized
		std::shared_ptr<std::mutex> mutex_;

		static constexpr double double_value_epsilon = 0.0001;
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a Talon
class TalonCommandHandle: public TalonStateHandle
{
	public:
		TalonCommandHandle(void) :
			TalonStateHandle(),
			cmd_(0)
		{
		}

		TalonCommandHandle(const TalonStateHandle &js, TalonHWCommand *cmd) :
			TalonStateHandle(js),
			cmd_(cmd)
		{
			if (!cmd_)
				throw HardwareInterfaceException("Cannot create Talon handle '" + js.getName() + "'. command pointer is null.");
		}

		// Operator which allows access to methods from
		// the TalonHWCommand member var associated with this
		// handle
		// Note that we could create separate methods in
		// the handle class for every method in the HWState
		// class, e.g.
		//     double getFoo(void) const {assert(_state); return state_->getFoo();}
		// but if each of them just pass things unchanged between
		// the calling code and the HWState method there's no
		// harm in making a single method to do so rather than
		// dozens of getFoo() one-line methods
		//
		TalonHWCommand *operator->()
		{
			assert(cmd_);
			return cmd_;
		}

		// Get a pointer to the HW state associated with
		// this Talon.  Since CommandHandle is derived
		// from StateHandle, there's a state embedded
		// in each instance of a CommandHandle. Use
		// this method to access it.
		//
		// handle->state()->getCANID();
		//
		const TalonHWState *state(void) const
		{
			return TalonStateHandle::operator->();
		}

	private:
		TalonHWCommand *cmd_;
};

// Use ClaimResources here since we only want 1 controller
// to be able to access a given Talon at any particular time
class TalonCommandInterface : public HardwareResourceManager<TalonCommandHandle, ClaimResources> {};
}
