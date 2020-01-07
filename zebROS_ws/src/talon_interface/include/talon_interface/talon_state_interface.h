#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{
// These should mirror the modes listed in ControlModes.h
// Need a separate copy here though, since sim won't be
// including that header file - sim shouldn't require
// anything specifically from the actual motor controller
// hardware
enum TalonMode
{
	TalonMode_First = -1,
	TalonMode_PercentOutput,
	TalonMode_Position,      // CloseLoop
	TalonMode_Velocity,      // CloseLoop
	TalonMode_Current,       // CloseLoop
	TalonMode_Follower,
	TalonMode_MotionProfile,
	TalonMode_MotionMagic,
	TalonMode_MotionProfileArc,
	TalonMode_Disabled,
	TalonMode_Last
};

enum DemandType
{
	DemandType_Neutral,
	DemandType_AuxPID,
	DemandType_ArbitraryFeedForward,
	DemandType_Last
};

enum NeutralMode
{
	NeutralMode_Uninitialized,
	NeutralMode_EEPROM_Setting,
	NeutralMode_Coast,
	NeutralMode_Brake,
	NeutralMode_Last
};

enum FeedbackDevice
{
	FeedbackDevice_Uninitialized,
	FeedbackDevice_QuadEncoder,
	FeedbackDevice_IntegratedSensor,
	FeedbackDevice_Analog,
	FeedbackDevice_Tachometer,
	FeedbackDevice_PulseWidthEncodedPosition,
	FeedbackDevice_SensorSum,
	FeedbackDevice_SensorDifference,
	FeedbackDevice_RemoteSensor0,
	FeedbackDevice_RemoteSensor1,
	FeedbackDevice_None,
	FeedbackDevice_SoftwareEmulatedSensor,
	FeedbackDevice_CTRE_MagEncoder_Relative = FeedbackDevice_QuadEncoder,
	FeedbackDevice_CTRE_MagEncoder_Absolute = FeedbackDevice_PulseWidthEncodedPosition,
	FeedbackDevice_Last
};

enum RemoteFeedbackDevice
{
	RemoteFeedbackDevice_SensorSum,
	RemoteFeedbackDevice_SensorDifference,
	RemoteFeedbackDevice_RemoteSensor0,
	RemoteFeedbackDevice_RemoteSensor1,
	RemoteFeedbackDevice_None,
	RemoteFeedbackDevice_SoftwareEmulatedSensor,
	RemoteFeedbackDevice_Last
};

enum RemoteSensorSource
{
	RemoteSensorSource_Off,
	RemoteSensorSource_TalonSRX_SelectedSensor,
	RemoteSensorSource_Pigeon_Yaw,
	RemoteSensorSource_Pigeon_Pitch,
	RemoteSensorSource_Pigeon_Roll,
	RemoteSensorSource_CANifier_Quadrature,
	RemoteSensorSource_CANifier_PWMInput0,
	RemoteSensorSource_CANifier_PWMInput1,
	RemoteSensorSource_CANifier_PWMInput2,
	RemoteSensorSource_CANifier_PWMInput3,
	RemoteSensorSource_GadgeteerPigeon_Yaw,
	RemoteSensorSource_GadgeteerPigeon_Pitch,
	RemoteSensorSource_GadgeteerPigeon_Roll,
	RemoteSensorSource_Last
};

enum SensorTerm
{
	SensorTerm_Sum0,
	SensorTerm_Sum1,
	SensorTerm_Diff0,
	SensorTerm_Diff1,
	SensorTerm_Last
};

enum LimitSwitchSource
{
	LimitSwitchSource_Uninitialized,
	LimitSwitchSource_FeedbackConnector,
	LimitSwitchSource_RemoteTalonSRX,
	LimitSwitchSource_RemoteCANifier,
	LimitSwitchSource_Deactivated,
	LimitSwitchSource_Last
};

enum RemoteLimitSwitchSource
{
	RemoteLimitSwitchSource_Uninitialized,
	RemoteLimitSwitchSource_RemoteTalonSRX,
	RemoteLimitSwitchSource_RemoteCANifier,
	RemoteLimitSwitchSource_Deactivated,
	RemoteLimitSwitchSource_Last
};

enum LimitSwitchNormal
{
	LimitSwitchNormal_Uninitialized,
	LimitSwitchNormal_NormallyOpen,
	LimitSwitchNormal_NormallyClosed,
	LimitSwitchNormal_Disabled,
	LimitSwitchNormal_Last
};

enum VelocityMeasurementPeriod {
	Period_1Ms = 1,
	Period_2Ms = 2,
	Period_5Ms = 5,
	Period_10Ms = 10,
	Period_20Ms = 20,
	Period_25Ms = 25,
	Period_50Ms = 50,
	Period_100Ms = 100,
};

enum StatusFrame
{
	Status_1_General,
	Status_2_Feedback0,
	Status_3_Quadrature,
	Status_4_AinTempVbat,
	Status_6_Misc,
	Status_7_CommStatus,
	Status_8_PulseWidth,
	Status_9_MotProfBuffer,
	Status_10_MotionMagic,
	Status_10_Targets = Status_10_MotionMagic,
	Status_11_UartGadgeteer,
	Status_12_Feedback1,
	Status_13_Base_PIDF0,
	Status_14_Turn_PIDF1,
	Status_15_FirmwareApiStatus,
	Status_Last
};
static const uint8_t status_1_general_default = 10;
static const uint8_t status_2_feedback0_default = 20;
static const uint8_t status_3_quadrature_default = 160;
static const uint8_t status_4_aintempvbat_default = 160;
static const uint8_t status_6_misc_default = 0;
static const uint8_t status_7_commstatus_default = 0;
static const uint8_t status_8_pulsewidth_default = 160;
static const uint8_t status_9_motprofbuffer_default = 50;
static const uint8_t status_10_motionmagic_default = 160;
static const uint8_t status_11_uartgadgeteer_default = 250;
static const uint8_t status_12_feedback1_default = 250;
static const uint8_t status_13_base_pidf0_default = 160;
static const uint8_t status_14_turn_pidf1_default = 250;
static const uint8_t status_15_firmwareapistatus_default = 160;

enum ControlFrame
{
	Control_3_General,
	Control_4_Advanced,
	Control_5_FeedbackOutputOverride,
	Control_6_MotProfAddTrajPoint,
	Control_Last
};
// TODO : what should these defaults be?
static const uint8_t control_3_general_default = 0;
static const uint8_t control_4_advanced_default = 0;
static const uint8_t control_5_feedbackoutputoverride_default = 0;
static const uint8_t control_6_motprofaddtrajpoint_default = 0;

// Match up with CTRE Motion profile struct
enum SetValueMotionProfile
{
	Disable = 0, Enable = 1, Hold = 2,
};

struct MotionProfileStatus
{
	int  topBufferRem;
	int  topBufferCnt;
	int  btmBufferCnt;
	bool hasUnderrun;
	bool isUnderrun;
	bool activePointValid;
	bool isLast;
	int  profileSlotSelect0;
	int  profileSlotSelect1;
	SetValueMotionProfile outputEnable;
	int  timeDurMs;

	MotionProfileStatus(void):
		topBufferRem(0),
		topBufferCnt(0),
		btmBufferCnt(0),
		hasUnderrun(false),
		isUnderrun(false),
		activePointValid(false),
		isLast(false),
		profileSlotSelect0(0),
		profileSlotSelect1(0),
		outputEnable(Disable),
		timeDurMs(0)
	{
	}

};

struct CustomProfileStatus
{
	bool running;
	int slotRunning;
	std::vector<int> remainingPoints;
	double remainingTime;	//Should this be a ROS duration?
							//Note: will be set based on slotRunning
	bool outOfPoints;
	CustomProfileStatus():
		running(false),
		slotRunning(-1),
		remainingTime(0.0),
		outOfPoints(false)
	{
	}
};

constexpr size_t TALON_PIDF_SLOTS = 4;

// Class which contains state information
// about a given Talon SRX. This should include
// data about the mode the Talon is running in,
// current config and setpoint as well as data
// from the attached encoders, limit switches,
// etc.
// Each pass through read() in the low-level
// hardware interface should update the member
// vars of this class.
// The controllers can access the member variables
// as needed to make decisions in their update code
// The hardware_controller is responsible for keeping
// a master array of these classes - 1 entry per
// physical Talon controller in the robot
class TalonHWState
{
	public:
		TalonHWState(int can_id) :
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
			neutral_deadband_(41.0/1023.0),

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

		double getSetpoint(void) const
		{
			return setpoint_;
		}
		double getPosition(void) const
		{
			return position_;
		}
		double getSpeed(void) const
		{
			return speed_;
		}
		double getOutputVoltage(void) const
		{
			return output_voltage_;
		}
		int    getCANID(void) const
		{
			return can_id_;
		}
		double getOutputCurrent(void) const
		{
			return output_current_;
		}
		double getBusVoltage(void) const
		{
			return bus_voltage_;
		}
		double getMotorOutputPercent(void) const
		{
			return motor_output_percent_;
		}
		double getTemperature(void) const
		{
			return temperature_;
		}
		double getPidfP(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return pidf_p_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getPidfP. Must be < TALON_PIDF_SLOTS.");
				return 0.0;
			}
		}
		double getPidfI(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return pidf_i_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getPidfI. Must be < TALON_PIDF_SLOTS.");
				return 0.0;
			}
		}
		double getPidfD(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return pidf_d_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getPidfD. Must be < TALON_PIDF_SLOTS.");
				return 0.0;
			}
		}
		double getPidfF(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return pidf_f_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getPidfF. Must be < TALON_PIDF_SLOTS.");
				return 0.0;
			}
		}
		int getPidfIzone(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return pidf_izone_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getPidfIzone. Must be < TALON_PIDF_SLOTS.");
				return 0;
			}
		}
		int getAllowableClosedLoopError(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return allowable_closed_loop_error_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getAllowableClosedLoopError. Must be < TALON_PIDF_SLOTS.");
				return 0;
			}
		}
		double getMaxIntegralAccumulator(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return max_integral_accumulator_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getMaxIntegralAccumulator. Must be < TALON_PIDF_SLOTS.");
				return 0;
			}
		}
		double getClosedLoopPeakOutput(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return closed_loop_peak_output_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getClosedLoopPeakOutput. Must be < TALON_PIDF_SLOTS.");
				return 0;
			}
		}
		int getClosedLoopPeriod(size_t index) const
		{
			if (index < TALON_PIDF_SLOTS)
				return closed_loop_period_[index];
			else
			{
				ROS_WARN_STREAM("Invalid index in talon_state getClosedLoopPeriod. Must be < TALON_PIDF_SLOTS.");
				return 0;
			}
		}
		bool getAuxPidPolarity(void) const
		{
			return aux_pid_polarity_;
		}

		double getClosedLoopError(void) const
		{
			return closed_loop_error_;
		}
		double getIntegralAccumulator(void) const
		{
			return integral_accumulator_;
		}
		double getErrorDerivative(void) const
		{
			return error_derivative_;
		}
		double getClosedLoopTarget(void) const
		{
			return closed_loop_target_;
		}
		double getPTerm(void) const
		{
			return p_term_;
		}
		double getITerm(void) const
		{
			return i_term_;
		}
		double getDTerm(void) const
		{
			return d_term_;
		}
		double getFTerm(void) const
		{
			return f_term_;
		}
		double getActiveTrajectoryPosition(void) const
		{
			return active_trajectory_position_;
		}
		double getActiveTrajectoryVelocity(void) const
		{
			return active_trajectory_velocity_;
		}
		double getActiveTrajectoryArbitraryFeedForward(void) const
		{
			return active_trajectory_arbitrary_feed_forward_;
		}
		double getActiveTrajectoryHeading(void) const
		{
			return active_trajectory_heading_;
		}
		bool getForwardLimitSwitch(void) const
		{
			return forward_limit_switch_closed_;
		}
		bool getReverseLimitSwitch(void) const
		{
			return reverse_limit_switch_closed_;
		}
		bool getForwardSoftlimitHit(void) const
		{
			return forward_softlimit_hit_;
		}
		bool getReverseSoftlimitHit(void) const
		{
			return reverse_softlimit_hit_;
		}

		TalonMode getTalonMode(void) const
		{
			return talon_mode_;
		}
		DemandType getDemand1Type(void) const
		{
			return demand1_type_;
		}
		double getDemand1Value(void) const
		{
			return demand1_value_;
		}
		int  getSlot(void) const
		{
			return slot_;
		}
		bool getInvert(void) const
		{
			return invert_;
		}
		bool getSensorPhase(void) const
		{
			return sensor_phase_;
		}
		NeutralMode getNeutralMode(void) const
		{
			return neutral_mode_;
		}
		bool getNeutralOutput(void) const
		{
			return neutral_output_;
		}
		FeedbackDevice getEncoderFeedback(void) const
		{
			return encoder_feedback_;
		}
		RemoteFeedbackDevice getRemoteEncoderFeedback(void) const
		{
			return encoder_feedback_remote_;
		}
		double getFeedbackCoefficient(void) const
		{
			return feedback_coefficient_;
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
		FeedbackDevice getSensorTerm(SensorTerm sensor_term) const
		{
			if (sensor_term < SensorTerm_Last)
				return sensor_terms_[sensor_term];
			ROS_WARN("getSensorTerm : sensor_term index too large");
			return FeedbackDevice_Last;
		}
		int getEncoderTicksPerRotation(void) const
		{
			return encoder_ticks_per_rotation_;
		}

		unsigned int getFaults(void) const
		{
			return faults_;
		}
		unsigned int getStickyFaults(void) const
		{
			return sticky_faults_;
		}
		double getConversionFactor(void) const
		{
			return conversion_factor_;
		}
		bool getEnableReadThread(void) const
		{
			return enable_read_thread_;
		}
		void setSetpoint(double setpoint)
		{
			setpoint_ = setpoint;
		}
		void setPosition(double position)
		{
			position_ = position;
		}
		void setSpeed(double speed)
		{
			speed_ = speed;
		}
		void setOutputVoltage(double output_voltage)
		{
			output_voltage_ = output_voltage;
		}
		void setOutputCurrent(double output_current)
		{
			output_current_ = output_current;
		}
		void setBusVoltage(double bus_voltage)
		{
			bus_voltage_ = bus_voltage;
		}
		void setMotorOutputPercent(double motor_output_percent)
		{
			motor_output_percent_ = motor_output_percent;
		}
		void setTemperature(double temperature)
		{
			temperature_ = temperature;
		}

		//output shaping
		void setClosedloopRamp(double close_loop_ramp)
		{
			close_loop_ramp_ = close_loop_ramp;
		}
		double getClosedloopRamp(void) const
		{
			return close_loop_ramp_;
		}

		void setOpenloopRamp(double open_loop_ramp)
		{
			open_loop_ramp_ = open_loop_ramp;
		}
		double getOpenloopRamp(void) const
		{
			return open_loop_ramp_;
		}

		void setPeakOutputForward(double peak_output_forward)
		{
			peak_output_forward_ = peak_output_forward;
		}
		double getPeakOutputForward(void) const
		{
			return peak_output_forward_;
		}
		void setPeakOutputReverse(double peak_output_reverse)
		{
			peak_output_reverse_ = peak_output_reverse;
		}
		double getPeakOutputReverse(void) const
		{
			return peak_output_reverse_;
		}

		void setNominalOutputForward(double nominal_output_forward)
		{
			nominal_output_forward_ = nominal_output_forward;
		}
		double getNominalOutputForward(void) const
		{
			return nominal_output_forward_;
		}
		void setNominalOutputReverse(double nominal_output_reverse)
		{
			nominal_output_reverse_ = nominal_output_reverse;
		}
		double getNominalOutputReverse(void) const
		{
			return nominal_output_reverse_;
		}

		void setNeutralDeadband(double neutral_deadband)
		{
			neutral_deadband_ = neutral_deadband;
		}
		double getNeutralDeadband(void) const
		{
			return neutral_deadband_;
		}

		void setVoltageCompensationSaturation(double voltage_compensation_saturation)
		{
			voltage_compensation_saturation_ = voltage_compensation_saturation;
		}
		double getVoltageCompensationSaturation(void) const
		{
			return voltage_compensation_saturation_;
		}

		void setVoltageMeasurementFilter(int voltage_measurement_filter)
		{
			voltage_measurement_filter_ = voltage_measurement_filter;
		}
		int getVoltageMeasurementFilter(void) const
		{
			return voltage_measurement_filter_;
		}

		void setVoltageCompensationEnable(bool voltage_compensation_enable)
		{
			voltage_compensation_enable_ = voltage_compensation_enable;
		}
		bool getVoltageCompensationEnable(void) const
		{
			return voltage_compensation_enable_;
		}
		void setVelocityMeasurementPeriod(hardware_interface::VelocityMeasurementPeriod period)
		{
			velocity_measurement_period_ = period;
		}

		bool getVelocityMeasurementPeriod(void) const
		{
			return velocity_measurement_period_;
		}

		void setVelocityMeasurementWindow(int window)
		{
			velocity_measurement_window_ = window;
		}

		bool getVelocityMeasurementWindow(void) const
		{
			return velocity_measurement_window_;
		}

		void setForwardLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
		{
			limit_switch_local_forward_source_ = source;
			limit_switch_local_forward_normal_ = normal;
		}

		void getForwardLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_forward_source_;
			normal = limit_switch_local_forward_normal_;
		}

		void setReverseLimitSwitchSource(LimitSwitchSource source, LimitSwitchNormal normal)
		{
			limit_switch_local_reverse_source_ = source;
			limit_switch_local_reverse_normal_ = normal;
		}

		void getReverseLimitSwitchSource(LimitSwitchSource &source, LimitSwitchNormal &normal) const
		{
			source = limit_switch_local_reverse_source_;
			normal = limit_switch_local_reverse_normal_;
		}

		void setRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
		{
			limit_switch_remote_forward_source_ = source;
			limit_switch_remote_forward_normal_ = normal;
			limit_switch_remote_forward_id_     = id;
		}

		void getRemoteForwardLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
		{
			source = limit_switch_remote_forward_source_;
			normal = limit_switch_remote_forward_normal_;
			id     = limit_switch_remote_forward_id_;
		}

		void setRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource source, LimitSwitchNormal normal, unsigned int id)
		{
			limit_switch_remote_reverse_source_ = source;
			limit_switch_remote_reverse_normal_ = normal;
			limit_switch_remote_reverse_id_     = id;
		}

		void getRemoteReverseLimitSwitchSource(RemoteLimitSwitchSource &source, LimitSwitchNormal &normal, unsigned int &id) const
		{
			source = limit_switch_remote_reverse_source_;
			normal = limit_switch_remote_reverse_normal_;
			id     = limit_switch_remote_reverse_id_;
		}

		void setForwardSoftLimitThreshold(double threshold)
		{
			softlimit_forward_threshold_ = threshold;
		}
		double getForwardSoftLimitThreshold(void) const
		{
			return softlimit_forward_threshold_;
		}

		void setForwardSoftLimitEnable(bool enable)
		{
			softlimit_forward_enable_ = enable;
		}
		bool getForwardSoftLimitEnable(void) const
		{
			return softlimit_forward_enable_;
		}
		void setReverseSoftLimitThreshold(double threshold)
		{
			softlimit_reverse_threshold_ = threshold;
		}
		double getReverseSoftLimitThreshold(void) const
		{
			return softlimit_reverse_threshold_;
		}

		void setReverseSoftLimitEnable(bool enable)
		{
			softlimit_reverse_enable_ = enable;
		}
		bool getReverseSoftLimitEnable(void) const
		{
			return softlimit_reverse_enable_;
		}

		void setOverrideSoftLimitsEnable(bool enable)
		{
			softlimits_override_enable_ = enable;
		}
		bool getOverrideSoftLimitsEnable(void) const
		{
			return softlimits_override_enable_;
		}

		void setPeakCurrentLimit(int amps)
		{
			current_limit_peak_amps_ = amps;
		}
		int getPeakCurrentLimit(void) const
		{
			return current_limit_peak_amps_;
		}

		void setPeakCurrentDuration(int msec)
		{
			current_limit_peak_msec_ = msec;
		}
		int getPeakCurrentDuration(void) const
		{
			return current_limit_peak_msec_;
		}
		void setContinuousCurrentLimit(int amps)
		{
			current_limit_continuous_amps_ = amps;
		}
		int getContinuousCurrentLimit(void) const
		{
			return current_limit_continuous_amps_;
		}
		void setCurrentLimitEnable(bool enable)
		{
			current_limit_enable_ = enable;
		}
		bool getCurrentLimitEnable(void) const
		{
			return current_limit_enable_;
		}

		void setMotionCruiseVelocity(double velocity)
		{
			motion_cruise_velocity_ = velocity;
		}
		double getMotionCruiseVelocity(void) const
		{
			return motion_cruise_velocity_;
		}
		void setMotionAcceleration(double acceleration)
		{
			motion_acceleration_ = acceleration;
		}
		double getMotionAcceleration(void) const
		{
			return motion_acceleration_;
		}

		void setMotionSCurveStrength(unsigned int s_curve_strength)
		{
			if (s_curve_strength > 8)
			{
				ROS_ERROR("setMotionSCurveStrength out of range");
				return;
			}

			motion_s_curve_strength_ = s_curve_strength;
		}
		unsigned int getMotionSCurveStrength(void) const
		{
			return motion_s_curve_strength_;
		}

		void setMotionProfileTopLevelBufferCount(int count)
		{
			motion_profile_top_level_buffer_count_ = count;
		}
		int getMotionProfileTopLevelBufferCount(void) const
		{
			return motion_profile_top_level_buffer_count_;
		}
		void setMotionProfileTopLevelBufferFull(bool is_full)
		{
			motion_profile_top_level_buffer_full_ = is_full;
		}
		bool getMotionProfileTopLevelBufferFull(void) const
		{
			return motion_profile_top_level_buffer_full_;
		}
		void setMotionProfileStatus(const MotionProfileStatus &status)
		{
			motion_profile_status_ = status;
		}
		MotionProfileStatus getMotionProfileStatus(void) const
		{
			return motion_profile_status_;
		}

		void setStatusFramePeriod(StatusFrame status_frame, uint8_t period)
		{
			if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
				status_frame_periods_[status_frame] = period;
			else
				ROS_ERROR("Invalid status_frame value passed to TalonHWState::setStatusFramePeriod()");
		}

		uint8_t getStatusFramePeriod(StatusFrame status_frame) const
		{
			if ((status_frame >= Status_1_General) && (status_frame < Status_Last))
				return status_frame_periods_[status_frame];

			ROS_ERROR("Invalid status_frame value passed to TalonHWState::setStatusFramePeriod()");
			return 0;
		}

		void setControlFramePeriod(ControlFrame control_frame, uint8_t period)
		{
			if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
				control_frame_periods_[control_frame] = period;
			else
				ROS_ERROR("Invalid control_frame value passed to TalonHWState::setControlFramePeriod()");
		}

		uint8_t getControlFramePeriod(ControlFrame control_frame) const
		{
			if ((control_frame >= Control_3_General) && (control_frame < Control_Last))
				return control_frame_periods_[control_frame];

			ROS_ERROR("Invalid control_frame value passed to TalonHWState::setControlFramePeriod()");
			return 0;
		}

		void setMotionProfileTrajectoryPeriod(int msec)
		{
			motion_profile_trajectory_period_ = msec;
		}
		int getMotionProfileTrajectoryPeriod(void) const
		{
			return motion_profile_trajectory_period_;
		}
		CustomProfileStatus getCustomProfileStatus(void) const
		{
			return custom_profile_status_;
		}
		void setCustomProfileStatus(const CustomProfileStatus &status)
		{
			custom_profile_status_ = status;
		}
		void setPidfP(double pidf_p, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				pidf_p_[index] = pidf_p;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setPidfP. Must be < TALON_PIDF_SLOTS.");
		}
		void setPidfI(double pidf_i, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				pidf_i_[index] = pidf_i;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setPidfI. Must be < TALON_PIDF_SLOTS.");
		}
		void setPidfD(double pidf_d, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				pidf_d_[index] = pidf_d;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setPidfD. Must be < TALON_PIDF_SLOTS.");
		}
		void setPidfF(double pidf_f, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				pidf_f_[index] = pidf_f;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setPidfF. Must be < TALON_PIDF_SLOTS.");
		}
		void setPidfIzone(int pidf_izone, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				pidf_izone_[index] = pidf_izone;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setPidfIzone. Must be < TALON_PIDF_SLOTS.");
		}
		void setAllowableClosedLoopError(int allowable_closed_loop_error, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				allowable_closed_loop_error_[index] = allowable_closed_loop_error;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setAllowableClosedLoopError. Must be < TALON_PIDF_SLOTS.");
		}
		void setMaxIntegralAccumulator(double max_integral_accumulator, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				max_integral_accumulator_[index] = max_integral_accumulator;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setMaxIntegralAccumulator. Must be < TALON_PIDF_SLOTS.");
		}
		void setClosedLoopPeakOutput(double closed_loop_peak_output, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				closed_loop_peak_output_[index] = closed_loop_peak_output;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setClosedLoopPeakOutput. Must be < TALON_PIDF_SLOTS.");
		}
		void setClosedLoopPeriod(double closed_loop_period, size_t index)
		{
			if (index < TALON_PIDF_SLOTS)
				closed_loop_period_[index] = closed_loop_period;
			else
				ROS_WARN_STREAM("Invalid index in talon_state setClosedLoopPeriod. Must be < TALON_PIDF_SLOTS.");
		}
		void setAuxPidPolarity(bool aux_pid_polarity)
		{
			aux_pid_polarity_ = aux_pid_polarity;
		}

		void setClosedLoopError(double closed_loop_error)
		{
			closed_loop_error_ = closed_loop_error;
		}
		void setIntegralAccumulator(double integral_accumulator)
		{
			integral_accumulator_ = integral_accumulator;
		}
		void setErrorDerivative(double error_derivative)
		{
			error_derivative_ = error_derivative;
		}
		void setClosedLoopTarget(double closed_loop_target)
		{
			closed_loop_target_ = closed_loop_target;
		}
		void setPTerm(double p_term)
		{
			p_term_ = p_term;
		}
		void setITerm(double i_term)
		{
			i_term_ = i_term;
		}
		void setDTerm(double d_term)
		{
			d_term_ = d_term;
		}
		void setFTerm(double f_term)
		{
			f_term_ = f_term;
		}
		void setActiveTrajectoryPosition(double active_trajectory_position)
		{
			active_trajectory_position_ = active_trajectory_position;
		}
		void setActiveTrajectoryVelocity(double active_trajectory_velocity)
		{
			active_trajectory_velocity_ = active_trajectory_velocity;
		}
		void setActiveTrajectoryArbitraryFeedForward(double active_trajectory_arbitrary_feed_forward)
		{
			active_trajectory_arbitrary_feed_forward_ = active_trajectory_arbitrary_feed_forward;
		}
		void setActiveTrajectoryHeading(double active_trajectory_heading)
		{
			active_trajectory_heading_ = active_trajectory_heading;
		}
		void setForwardLimitSwitch(bool forward_limit_switch_closed)
		{
			forward_limit_switch_closed_ = forward_limit_switch_closed;
		}
		void setReverseLimitSwitch(bool reverse_limit_switch_closed)
		{
			reverse_limit_switch_closed_ = reverse_limit_switch_closed;
		}
		void setForwardSoftlimitHit(bool forward_softlimit_hit)
		{
			forward_softlimit_hit_ = forward_softlimit_hit;
		}
		void setReverseSoftlimitHit(bool reverse_softlimit_hit)
		{
			reverse_softlimit_hit_ = reverse_softlimit_hit;
		}

		void setTalonMode(TalonMode talon_mode)
		{
			if ((talon_mode_ > TalonMode_First) &&
				(talon_mode_ < TalonMode_Last) )
				talon_mode_ = talon_mode;
			else
				ROS_WARN("Invalid talon mode requested");
		}
		void setDemand1Type(DemandType demand_type)
		{
			if ((demand_type >= DemandType_Neutral) &&
				(demand_type < DemandType_Last))
				demand1_type_ = demand_type;
			else
				ROS_WARN("Invalid demand 1 type requested");
		}
		void setDemand1Value(double value)
		{
			demand1_value_ = value;
		}
		void setSlot(int slot)
		{
			slot_ = slot;
		}
		void setInvert(bool invert)
		{
			invert_ = invert;
		}
		void setSensorPhase(bool sensor_phase)
		{
			sensor_phase_ = sensor_phase;
		}
		void setNeutralMode(NeutralMode neutral_mode)
		{
			if ((neutral_mode_ >= NeutralMode_Uninitialized) &&
				(neutral_mode_ <  NeutralMode_Last) )
				neutral_mode_ = neutral_mode;
			else
				ROS_WARN_STREAM("Invalid neutral mode requested");
		}
		void setNeutralOutput(bool neutral_output)
		{
			neutral_output_ = neutral_output;
		}

		void setEncoderFeedback(FeedbackDevice encoder_feedback)
		{
			if ((encoder_feedback >= FeedbackDevice_Uninitialized) &&
				(encoder_feedback <  FeedbackDevice_Last) )
				encoder_feedback_ = encoder_feedback;
			else
				ROS_WARN_STREAM("Invalid feedback device requested");
		}
		void setRemoteEncoderFeedback(RemoteFeedbackDevice encoder_feedback_remote)
		{
			if ((encoder_feedback_remote >= RemoteFeedbackDevice_SensorSum) &&
				(encoder_feedback_remote <  RemoteFeedbackDevice_Last) )
				encoder_feedback_remote_ = encoder_feedback_remote;
			else
				ROS_WARN_STREAM("Invalid remote feedback device requested");
		}
		void setFeedbackCoefficient(double feedback_coefficient)
		{
			feedback_coefficient_ = feedback_coefficient;
		}
		void setRemoteFeedbackDeviceId(int device_id, size_t remote_ordinal)
		{
			if (remote_ordinal >= 2)
			{
				ROS_WARN("setRemoteFeedbackDeviceId : remote_ordinal too large");
				return;
			}
			remote_feedback_device_ids_[remote_ordinal] = device_id;
		}
		void setRemoteFeedbackDeviceIds(const std::array<int, 2> &remote_feedback_device_ids)
		{
			remote_feedback_device_ids_ = remote_feedback_device_ids;
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
		}
		void setRemoteFeedbackFilters(const std::array<RemoteSensorSource, 2> &remote_sensor_sources)
		{
			remote_feedback_filters_ = remote_sensor_sources;
		}
		void setSensorTerm(FeedbackDevice feedback_device, SensorTerm sensor_term)
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
		void setSensorTerms(const std::array<FeedbackDevice, SensorTerm_Last> &sensor_terms)
		{
			sensor_terms_ = sensor_terms;
		}
		void setEncoderTicksPerRotation(int encoder_ticks_per_rotation)
		{
			encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
		}
		void setFaults(unsigned int faults)
		{
			faults_ = faults;
		}
		void setStickyFaults(unsigned int sticky_faults)
		{
			sticky_faults_ = sticky_faults;
		}
		void setConversionFactor(double conversion_factor)
		{
			conversion_factor_ = conversion_factor;
		}
		void setEnableReadThread(bool enable_read_thread)
		{
			enable_read_thread_ = enable_read_thread;
		}

		void setFirmwareVersion(int firmware_version)
		{
			firmware_version_ = firmware_version;
		}

		int getFirmwareVersion(void) const
		{
			return firmware_version_;
		}

	private:
		double setpoint_;
		double position_;
		double speed_;
		double output_voltage_;
		double output_current_;
		double bus_voltage_;
		double motor_output_percent_;
		double temperature_;
		std::array<double, TALON_PIDF_SLOTS> pidf_p_;
		std::array<double, TALON_PIDF_SLOTS> pidf_i_;
		std::array<double, TALON_PIDF_SLOTS> pidf_d_;
		std::array<double, TALON_PIDF_SLOTS> pidf_f_;
		std::array<int, TALON_PIDF_SLOTS>    pidf_izone_;
		std::array<int, TALON_PIDF_SLOTS>    allowable_closed_loop_error_;
		std::array<double, TALON_PIDF_SLOTS> max_integral_accumulator_;
		std::array<double, TALON_PIDF_SLOTS> closed_loop_peak_output_;
		std::array<int, TALON_PIDF_SLOTS>    closed_loop_period_;
		bool   aux_pid_polarity_;
		double closed_loop_error_;
		double integral_accumulator_;
		double error_derivative_;
		double closed_loop_target_;
		double p_term_;
		double i_term_;
		double d_term_;
		double f_term_;
		double active_trajectory_position_;
		double active_trajectory_velocity_;
		double active_trajectory_arbitrary_feed_forward_;
		double active_trajectory_heading_;
		bool   forward_limit_switch_closed_;
		bool   reverse_limit_switch_closed_;
		bool   forward_softlimit_hit_;
		bool   reverse_softlimit_hit_;

		TalonMode  talon_mode_;
		DemandType demand1_type_;
		double     demand1_value_;

		int can_id_;

		int  slot_;
		bool invert_;
		bool sensor_phase_;

		NeutralMode neutral_mode_;
		bool        neutral_output_;

		FeedbackDevice encoder_feedback_;
		double feedback_coefficient_;
		RemoteFeedbackDevice encoder_feedback_remote_;
		int encoder_ticks_per_rotation_;
		std::array<int, 2> remote_feedback_device_ids_;
		std::array<RemoteSensorSource, 2> remote_feedback_filters_;
		std::array<FeedbackDevice, SensorTerm_Last> sensor_terms_;

		// output shaping
		double close_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;

		// voltage compensation
		double voltage_compensation_saturation_;
		int   voltage_measurement_filter_;
		bool  voltage_compensation_enable_;

		hardware_interface::VelocityMeasurementPeriod velocity_measurement_period_;
		int velocity_measurement_window_;

		LimitSwitchSource limit_switch_local_forward_source_;
		LimitSwitchNormal limit_switch_local_forward_normal_;
		LimitSwitchSource limit_switch_local_reverse_source_;
		LimitSwitchNormal limit_switch_local_reverse_normal_;
		RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		LimitSwitchNormal limit_switch_remote_forward_normal_;
		unsigned int limit_switch_remote_forward_id_;
		RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		LimitSwitchNormal limit_switch_remote_reverse_normal_;
		unsigned int limit_switch_remote_reverse_id_;

		// soft limits
		double softlimit_forward_threshold_;
		bool softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool softlimit_reverse_enable_;
		bool softlimits_override_enable_;

		// current limiting
		int current_limit_peak_amps_;
		int current_limit_peak_msec_;
		int current_limit_continuous_amps_;
		bool current_limit_enable_;

		// Talon expects these in integral sensorUnitsPer100ms,
		// but at this level we're still dealing with
		// radians/sec (or /sec^2 for acceleration)
		double motion_cruise_velocity_;
		double motion_acceleration_;

		unsigned int motion_s_curve_strength_;

		// Motion profiling
		int motion_profile_top_level_buffer_count_;
		bool motion_profile_top_level_buffer_full_;
		MotionProfileStatus motion_profile_status_;
		CustomProfileStatus custom_profile_status_;
		int motion_profile_trajectory_period_;

		std::array<uint8_t, Status_Last> status_frame_periods_;
		std::array<uint8_t, Control_Last> control_frame_periods_;

		unsigned int faults_;
		unsigned int sticky_faults_;

		double conversion_factor_;

		bool enable_read_thread_;

		int firmware_version_;
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef StateHandle<const TalonHWState> TalonStateHandle;
typedef StateHandle<TalonHWState> TalonWritableStateHandle;
class TalonStateInterface : public HardwareResourceManager<TalonStateHandle> {};

}

