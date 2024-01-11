#ifndef INC_TALONFXPRO_COMMAND_INTERFACE_H
#define INC_TALONFXPRO_COMMAND_INTERFACE_H

#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "ctre_interfaces/talonfxpro_state_types.h"
#include "state_handle/command_handle.h"

namespace hardware_interface::talonfxpro
{

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
class TalonFXProHWCommand
{
public:
	TalonFXProHWCommand(void);
	TalonFXProHWCommand(const TalonFXProHWCommand &) = delete;
	TalonFXProHWCommand(TalonFXProHWCommand &&) = delete;
	~TalonFXProHWCommand();

	TalonFXProHWCommand operator=(const TalonFXProHWCommand &) = delete;
	TalonFXProHWCommand &operator=(TalonFXProHWCommand &&) = delete;

	void setkP(const double kP, const size_t index);
	double getkP(const size_t index) const;

	void setkI(double kI, const size_t index);
	double getkI(size_t index) const;

	void setkD(double kD, const size_t index);
	double getkD(size_t index) const;

	void setkS(const double kS, const size_t index);
	double getkS(size_t index) const;

	void setkV(const double kV, const size_t index);
	double getkV(size_t index) const;

	void setkA(const double kA, const size_t index);
	double getkA(size_t index) const;

	void setkG(const double kG, const size_t index);
	double getkG(size_t index) const;

	void setGravityType(const GravityType gravity_type, const size_t index);
	GravityType getGravityType(const size_t index) const;

	bool slotChanged(double &kP,
					 double &kI,
					 double &kD,
					 double &kS,
					 double &kV,
					 double &kA,
					 double &kG,
					 GravityType &gravity_type,
					 size_t index) const;
	void resetSlot(size_t index);

	void setInvert(const Inverted invert);
	Inverted getInvert(void) const;

	void setNeutralMode(const NeutralMode neutral_mode);
	NeutralMode getNeutralMode(void) const;

	void setDutyCycleNeutralDeadband(const double duty_cycle_neutral_deadband);
	double getDutyCycleNeutralDeadband(void) const;
	void setPeakForwardDutyCycle(const double peak_forward_duty_cycle);
	double getPeakForwardDutyCycle(void) const;
	void setPeakReverseDutyCycle(const double peak_reverse_duty_cycle);
	double getPeakReverseDutyCycle(void) const;

	bool motorOutputConfigChanged(Inverted &invert,
								  NeutralMode &neutral_mode,
								  double &duty_cycle_neutral_deadband,
								  double &peak_forward_duty_cycle,
								  double &peak_reverse_duty_cycle) const;
	void resetMotorOutputConfig(void);

	void setStatorCurrentLimit(const double stator_current_limit);
	double getStatorCurrentLimit(void) const;
	void setStatorCurrentLimitEnable(const bool stator_current_limit_enable);
	bool getStatorCurrentLimitEnable(void) const;

	void setSupplyCurrentLimit(const double supply_current_limit);
	double getSupplyCurrentLimit(void) const;
	void setSupplyCurrentLimitEnable(const bool supply_current_limit_enable);
	bool getSupplyCurrentLimitEnable(void) const;
	bool currentLimitChanged(double &stator_current_limit,
							 bool &stator_current_limit_enable,
							 double &supply_current_limit,
							 bool &supply_current_limit_enable) const;
	void resetCurrentLimit(void);

	void setSupplyVoltageTimeConstant(const double supply_voltage_time_constant);
	double getSupplyVoltageTimeConstant(void) const;

	void setPeakForwardVoltage(const double peak_forward_voltage);
	double getPeakForwardVoltage(void) const;

	void setPeakReverseVoltage(const double peak_reverse_voltage);
	double getPeakReverseVoltage(void) const;

	bool voltageConfigsChanged(double &supply_voltage_time_constant,
							   double &peak_voltage_forward,
							   double &peak_voltage_reverse) const;
	void resetVoltageConfigs(void);

	void setPeakForwardTorqueCurrent(const double peak_forward_torque_current);
	double getPeakForwardTorqueCurrent(void) const;
	void setPeakReverseTorqueCurrent(const double peak_reverse_torque_current);
	double getPeakReverseTorqueCurrent(void) const;
	void setTorqueNeutralDeadband(const double torque_neutral_deadband);
	double getTorqueNeutralDeadband(void) const;

	bool torqueCurrentChanged(double &peak_forward_torque_current,
							  double &peak_reverse_torque_current,
							  double &torque_neutral_deadband) const;
	void resetTorqueCurrent(void);

	void setFeedbackRotorOffset(const double feedback_rotor_offset);
	double getFeedbackRotorOffset(void) const;

	void setSensorToMechanismRatio(const double sensor_to_mechanism_ratio);
	double getSensorToMechanismRatio(void) const;

	void setRotorToSensorRatio(const double rotor_to_sensor_ratio);
	double setRotorToSensorRatio(void) const;

	void setFeedbackSensorSource(const FeedbackSensorSource feedback_sensor_source);
	FeedbackSensorSource getFeedbackSensorSource(void) const;

	void setFeedbackRemoteSensorID(const int feedback_remote_sensor_id);
	int setFeedbackRemoteSensorID(void) const;

	bool feebackChanged(double &feedback_rotor_offset,
						double &sensor_to_mechanism_ratio,
						double &rotor_to_sensor_ratio,
						FeedbackSensorSource &feedback_sensor_source,
						int &feedback_remote_sensor_id) const;
	void resetFeedback(void);

	void setDifferentialSensorSource(const DifferentialSensorSource differential_sensor_source);
	DifferentialSensorSource getDifferentialSensorSource(void) const;

	void setDifferentialTalonFXSensorID(const int differential_talonfx_sensor_id);
	int  getDifferentialTalonFXSensorID(void) const;

	void setDifferentialRemoteSensorID(const int differential_remote_sensor_id);
	int  getDifferentialRemoteSensorID(void) const;

	bool differentialSensorsChanged(DifferentialSensorSource &differential_sensor_source,
									int &differential_talonfx_sensor_id,
									int &differential_remote_sensor_id) const;
	void resetDifferentialSensors(void);

	void setPeakDifferentialDutyCycle(const double peak_differential_duty_cycle);
	double getPeakDifferentialDutyCycle(void) const;

	void setPeakDifferentialVoltage(const double peak_differential_voltage);
	double getPeakDifferentialVoltage(void) const;

	void setPeakDifferentialTorqueCurrent(const double peak_differential_torque_current);
	double getPeakDifferentialTorqueCurrent(void) const;

	bool differentialConstantsChanged(double &peak_differential_duty_cycle,
									  double &peak_differential_voltage,
									  double &peak_differential_torque_current) const;
	void resetDifferentialConstants(void);

	void setDutyCycleOpenLoopRampPeriod(const double duty_cycle_open_loop_ramp_period);
	double getDutyCycleOpenLoopRampPeriod(void) const;

	void setVoltageOpenLoopRampPeriod(const double voltage_open_loop_ramp_period);
	double getVoltageOpenLoopRampPeriod(void) const;

	void setTorqueOpenLoopRampPeriod(const double torque_open_loop_ramp_period);
	double getTorqueOpenLoopRampPeriod(void) const;

	bool openLoopRampsChanged(double &duty_cycle_open_loop_ramp_period,
							  double &voltage_open_loop_ramp_period,
							  double &torque_open_loop_ramp_period) const;
	void resetOpenLoopRamps(void);

	void setDutyCycleClosedLoopRampPeriod(const double duty_cycle_closed_loop_ramp_period);
	double getDutyCycleClosedLoopRampPeriod(void) const;

	void setVoltageClosedLoopRampPeriod(const double voltage_closed_loop_ramp_period);
	double getVoltageClosedLoopRampPeriod(void) const;

	void setTorqueClosedLoopRampPeriod(const double torque_closed_loop_ramp_period);
	double getTorqueClosedLoopRampPeriod(void) const;

	bool closedLoopRampsChanged(double &duty_cycle_closed_loop_ramp_period,
								double &voltage_closed_loop_ramp_period,
								double &torque_closed_loop_ramp_period) const;
	void resetClosedLoopRamps(void);

	void setForwardLimitType(const LimitType forward_limit_type);
	LimitType getForwardLimitType(void) const;

	void setForwardLimitAutosetPositionEnable(const bool forward_limit_autoset_position_enable);
	bool getForwardLimitAutosetPositionEnable(void) const;

	void setForwardLimitAutosetPositionValue(const double forward_limit_autoset_position_value);
	double setForwardLimitAutosetPositionValue(void) const;

	void setForwardLimitEnable(const bool forward_limit_enable);
	bool getForwardLimitEnable(void) const;

	void setForwardLimitSource(const LimitSource forward_limit_source);
	LimitSource getForwardLimitSource(void) const;

	void setForwardLimitRemoteSensorID(const int forward_limit_remote_sensor_id);
	int getForwardLimitRemoteSensorID(void) const;

	void setReverseLimitType(const LimitType reverse_limit_type);
	LimitType getReverseLimitType(void) const;

	void setReverseLimitAutosetPositionEnable(const bool reverse_limit_autoset_position_enable);
	bool getReverseLimitAutosetPositionEnable(void) const;

	void setReverseLimitAutosetPositionValue(const double reverse_limit_autoset_position_value);
	double setReverseLimitAutosetPositionValue(void) const;

	void setReverseLimitEnable(const bool reverse_limit_enable);
	bool getReverseLimitEnable(void) const;

	void setReverseLimitSource(const LimitSource reverse_limit_source);
	LimitSource getReverseLimitSource(void) const;

	void setReverseLimitRemoteSensorID(const int reverse_limit_remote_sensor_id);
	int getReverseLimitRemoteSensorID(void) const;

	bool limitChanged(LimitType &forward_limit_type,
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
					  int &reverse_limit_remote_sensor_id) const;
	void resetLimit(void);

	void setBeepOnBoot(const bool beep_on_boot);
	bool getBeepOnBoot(void) const;
	void setBeepOnConfig(const bool beep_on_config);
	bool getBeepOnConfig(void) const;
	void setAllowMusicDurDisable(const bool allow_music_dur_disable);
	bool getAllowMusicDurDisable(void) const;
	bool audioChanged(bool &beep_on_boot, bool &beep_on_config, bool &allow_music_dur_disable) const;
	void resetAudio(void);

	void setForwardSoftLimitEnable(const bool enable);
	bool getForwardSoftLimitEnable(void) const;
	void setReverseSoftLimitEnable(const bool enable);
	bool getReverseSoftLimitEnable(void) const;

	void setForwardSoftLimitThreshold(const double threshold);
	double getForwardSoftLimitThreshold(void) const;
	void setReverseSoftLimitThreshold(const double threshold);
	double getReverseSoftLimitThreshold(void) const;

	bool softLimitChanged(bool &forward_enable, bool &reverse_enable, double &forward_threshold, double &reverse_threshold) const;
	void resetSoftLimit(void);

	void setMotionMagicCruiseVelocity(const double motion_magic_cruise_velocity);
	double getMotionMagicCruiseVelocity(void) const;
	void setMotionMagicAcceleration(const double motion_magic_acceleration);
	double getMotionMagicAcceleration(void) const;
	void setMotionMagicJerk(const double motion_magic_jerk);
	double getMotionMagicJerk(void) const;

	bool motionMagicChanged(double &motion_magic_cruise_velocity, double &motion_magic_acceleration, double &motion_magic_jerk) const;
	void resetMotionMagic(void);

	void setContinuousWrap(const bool continuous_wrap);
	bool getContinuousWrap(void) const;
	bool continuousWrapChanged(bool &continuous_wrap) const;
	void resetContinuousWrap(void);

	void setClearStickyFaults(void);
	bool getClearStickyFaults(void) const;
	bool clearStickyFaultsChanged(void) const;

	void setRotorPosition(const double position);
	double getRotorPosition(void) const;
	bool rotorPositionChanged(double &position) const;
	void resetRotorPosition(void);

	void setControlMode(const TalonMode mode);
	TalonMode getControlMode(void) const;

	void setControlOutput(const double control_output);
	double getControlOutput(void) const;

	void setControlPosition(const double control_position);
	double getControlPosition(void) const;

	void setControlVelocity(const double control_velocity);
	double getControlVelocity(void) const;

	void setControlAcceleration(const double control_acceleration);
	double getControlAcceleration(void) const;

	void setControlJerk(const double control_jerk);
	double getControlJerk(void) const;

	void setControlEnableFOC(const bool control_enable_foc);
	bool getControlEnableFOC(void) const;

	void setControlOverrideBrakeDurNeutral(const bool control_override_brake_dur_neutral);
	bool getControlOverrideBrakeDurNeutral(void) const;

	void setControlMaxAbsDutyCycle(const double control_max_abs_duty_cycle);
	double getControlMaxAbsDutyCycle(void) const;

	void setControlDeadband(const double control_deadband);
	double getControlDeadband(void) const;

	void setControlFeedforward(const double control_feedforward);
	double getControlFeedforward(void) const;

	void setControlSlot(const int control_slot);
	int getControlSlot(void) const;

	void setControlDifferentialPosition(const double control_differential_position);
	double getControlDifferentialPosition(void) const;

	void setControlDifferentialSlot(const int control_differential_slot);
	int getControlDifferentialSlot(void) const;

	void setControlOpposeMasterDirection(const bool oppose_master_direction);
	bool getControlOpposeMasterDirection(void) const;

	void setControlLimitForwardMotion(const bool limit_forward_direction);
	bool getControlLimitForwardMotion(void) const;

	void setControlLimitReverseMotion(const bool limit_reverse_direction);
	bool getControlLimitReverseMotion(void) const;

	bool controlChanged(TalonMode &control_mode,
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
						bool &oppose_master_direction) const;
	void resetControl(void);
						
	void setEnableReadThread(const bool enable_read_thread);
	bool getEnableReadThread(void) const;

private :
	// Slot0Configs ... Slot2Configs
	std::array<double, TALON_PIDF_SLOTS> kP_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kI_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kD_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kS_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kV_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kA_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kG_{0.0, 0.0, 0.0};
	std::array<GravityType, TALON_PIDF_SLOTS> gravity_type_{GravityType::Elevator_Static, GravityType::Elevator_Static, GravityType::Elevator_Static};
	mutable std::array<bool, TALON_PIDF_SLOTS> slot_changed_{true, true, true};
	
	// MotorOutputConfigs
	Inverted    invert_{Inverted::CounterClockwise_Positive};

	NeutralMode neutral_mode_{NeutralMode::Coast};

	double duty_cycle_neutral_deadband_{0.};
	double peak_forward_duty_cycle_{1.};
	double peak_reverse_duty_cycle_{-1.};
	mutable bool motor_output_config_changed_{true};

	double stator_current_limit_{0.};
	bool   stator_current_limit_enable_{false};
	double supply_current_limit_{0.};
	bool   supply_current_limit_enable_{false};
	mutable bool current_limit_changed_{true};

	double supply_voltage_time_constant_{0.};
	double peak_forward_voltage_{16.};
	double peak_reverse_voltage_{-16.};
	mutable bool voltage_configs_changed_{true};

	double peak_forward_torque_current_{800.};
	double peak_reverse_torque_current_{800.};
	double torque_neutral_deadband_{0.0};
	mutable bool   torque_current_changed_{true};

	double feedback_rotor_offset_{0.0};
	double sensor_to_mechanism_ratio_{1.0};
	double rotor_to_sensor_ratio_{1.0};
	FeedbackSensorSource feedback_sensor_source_{FeedbackSensorSource::RotorSensor};
	int    feedback_remote_sensor_id_{0};
	mutable bool feedback_changed_{true};

	DifferentialSensorSource differential_sensor_source_{DifferentialSensorSource::Disabled};
	int  differential_talonfx_sensor_id_{0};
	int  differential_remote_sensor_id_{0};
	mutable bool differential_sensors_changed_{true};

	double peak_differential_duty_cycle_{2.};
	double peak_differential_voltage_{32.};
	double peak_differential_torque_current_{1600.};
	mutable bool differential_constants_changed_{true};

	double duty_cycle_open_loop_ramp_period_{0.};
	double voltage_open_loop_ramp_period_{0.};
	double torque_open_loop_ramp_period_{0.};
	mutable bool open_loop_ramps_changed_{true};

	double duty_cycle_closed_loop_ramp_period_{0.};
	double voltage_closed_loop_ramp_period_{0.};
	double torque_closed_loop_ramp_period_{0.};
	mutable bool closed_loop_ramps_changed_{true};

	LimitType forward_limit_type_{LimitType::NormallyOpen};
	bool forward_limit_autoset_position_enable_{false};
	double forward_limit_autoset_position_value_{0.};
	bool forward_limit_enable_{true};
	LimitSource forward_limit_source_{LimitSource::LimitSwitchPin};
	int forward_limit_remote_sensor_id_{0};
	LimitType reverse_limit_type_{LimitType::NormallyOpen};
	bool reverse_limit_autoset_position_enable_{false};
	double reverse_limit_autoset_position_value_{0.};
	bool reverse_limit_enable_{true};
	LimitSource reverse_limit_source_{LimitSource::LimitSwitchPin};
	int reverse_limit_remote_sensor_id_{0};
	mutable bool limit_changed_{true};

	bool beep_on_boot_{true};
	bool beep_on_config_{true};
	bool allow_music_dur_disable_{false};
	mutable bool audio_changed_{true};

	bool   softlimit_forward_enable_{false};
	bool   softlimit_reverse_enable_{false};
	double softlimit_forward_threshold_{0.0};
	double softlimit_reverse_threshold_{0.0};
	mutable bool softlimit_changed_{true};

	double motion_magic_cruise_velocity_{0.0};
	double motion_magic_acceleration_{0.0};
	double motion_magic_jerk_{0.0};
	mutable bool motion_magic_changed_{true};

	bool continuous_wrap_{false};
	mutable bool continuous_wrap_changed_{true};

	mutable bool clear_sticky_faults_{false};

	double rotor_position_{0};
	mutable bool rotor_position_changed_{false};

	TalonMode control_mode_{TalonMode::CoastOut}; 
	double control_output_{0.0};
	double control_position_{0.0};
	double control_velocity_{0.0};
	double control_acceleration_{0.0};
	double control_jerk_{0.0};
	bool control_enable_foc_{true};
	bool control_override_brake_dur_neutral_{false};
	double control_max_abs_duty_cycle_{1.0};
	double control_deadband_{0.0};
	double control_feedforward_{0.0};
	int control_slot_{0};
	bool control_oppose_master_direction_{false};
	bool control_limit_forward_motion_{false};
	bool control_limit_reverse_motion_{false};
	double control_differential_position_{0.0};
	int control_differential_slot_{0};
	mutable bool control_changed_{false};

	bool enable_read_thread_{true};

	static constexpr double double_value_epsilon = 0.0001;
};

// Create a handle pointing to a type TalonFXProHWCommand / TalonFXProHWState pair
using TalonFXProCommandHandle = CommandHandle<TalonFXProHWCommand, TalonFXProHWState, TalonFXProStateHandle>;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given Talon at any particular time
class TalonFXProCommandInterface : public HardwareResourceManager<TalonFXProCommandHandle, ClaimResources> {};
} // namspace hardware_interface

#endif