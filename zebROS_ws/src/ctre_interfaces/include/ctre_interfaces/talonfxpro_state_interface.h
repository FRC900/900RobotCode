#ifndef INC_TALONFXPRO_STATE_INTERFACE__
#define INC_TALONFXPRO_STATE_INTERFACE__

#include <array>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include "ctre_interfaces/talonfxpro_state_types.h"
#include "state_handle/state_handle.h"

namespace hardware_interface::talonfxpro
{

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
class TalonFXProHWState
{
public:
	explicit TalonFXProHWState(const int can_id);
	TalonFXProHWState(const TalonFXProHWState &) = delete;
	TalonFXProHWState(TalonFXProHWState &&) = delete;
	~TalonFXProHWState();

	TalonFXProHWState operator=(const TalonFXProHWState &) = delete;
	TalonFXProHWState &operator=(TalonFXProHWState &&) = delete;
	
	int getCANID(void) const;

	void setkP(const double kP, const size_t index);
	double getkP(const size_t index) const;

	void setkI(const double kI, const size_t index);
	double getkI(const size_t index) const;

	void setkD(const double kD, const size_t index);
	double getkD(const size_t index) const;

	void setkS(double kS, size_t index);
	double getkS(size_t index) const;

	void setkV(const double kV, const size_t index);
	double getkV(const size_t index) const;

	void setkA(const double kA, const size_t index);
	double getkA(const size_t index) const;

	void setkG(const double kG, const size_t index);
	double getkG(const size_t index) const;

	void setGravityType(const GravityType gravity_type, const size_t index);
	GravityType getGravityType(const size_t index) const;

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

	void setStatorCurrentLimit(const double stator_current_limit);
	double getStatorCurrentLimit(void) const;
	void setStatorCurrentLimitEnable(const bool stator_current_limit_enable);
	bool getStatorCurrentLimitEnable(void) const;

	void setSupplyCurrentLimit(const double supply_current_limit);
	double getSupplyCurrentLimit(void) const;
	void setSupplyCurrentLimitEnable(const bool supply_current_limit_enable);
	bool getSupplyCurrentLimitEnable(void) const;

	void setSupplyVoltageTimeConstant(const double supply_voltage_time_constant);
	double getSupplyVoltageTimeConstant(void) const;

	void setPeakForwardVoltage(const double peak_forward_voltage);
	double getPeakForwardVoltage(void) const;

	void setPeakReverseVoltage(const double peak_reverse_voltage);
	double getPeakReverseVoltage(void) const;

	void setPeakForwardTorqueCurrent(const double peak_forward_torque_current);
	double getPeakForwardTorqueCurrent(void) const;
	void setPeakReverseTorqueCurrent(const double peak_reverse_torque_current);
	double getPeakReverseTorqueCurrent(void) const;
	void setTorqueNeutralDeadband(const double torque_neutral_deadband);
	double getTorqueNeutralDeadband(void) const;

	void setFeedbackRotorOffset(const double feedback_rotor_offset);
	double getFeedbackRotorOffset(void) const;

	void setSensorToMechanismRatio(const double sensor_to_mechanism_ratio);
	double getSensorToMechanismRatio(void) const;

	void setRotorToSensorRatio(const double rotor_to_sensor_ratio);
	double getRotorToSensorRatio(void) const;

	void setFeedbackSensorSource(const FeedbackSensorSource feedback_sensor_source);
	FeedbackSensorSource getFeedbackSensorSource(void) const;

	void setFeedbackRemoteSensorID(const int feedback_remote_sensor_id);
	int getFeedbackRemoteSensorID(void) const;

	void setDifferentialSensorSource(const DifferentialSensorSource differential_sensor_source);
	DifferentialSensorSource getDifferentialSensorSource(void) const;

	void setDifferentialTalonFXSensorID(const int differential_talonfx_sensor_id);
	int  getDifferentialTalonFXSensorID(void) const;

	void setDifferentialRemoteSensorID(const int differential_remote_sensor_id);
	int  getDifferentialRemoteSensorID(void) const;

	void setPeakDifferentialDutyCycle(const double peak_differential_duty_cycle);
	double getPeakDifferentialDutyCycle(void) const;

	void setPeakDifferentialVoltage(const double peak_differential_voltage);
	double getPeakDifferentialVoltage(void) const;

	void setPeakDifferentialTorqueCurrent(const double peak_differential_torque_current);
	double getPeakDifferentialTorqueCurrent(void) const;

	void setDutyCycleOpenLoopRampPeriod(const double duty_cycle_open_loop_ramp_period);
	double getDutyCycleOpenLoopRampPeriod(void) const;

	void setVoltageOpenLoopRampPeriod(const double voltage_open_loop_ramp_period);
	double getVoltageOpenLoopRampPeriod(void) const;

	void setTorqueOpenLoopRampPeriod(const double torque_open_loop_ramp_period);
	double getTorqueOpenLoopRampPeriod(void) const;

	void setDutyCycleClosedLoopRampPeriod(const double duty_cycle_closed_loop_ramp_period);
	double getDutyCycleClosedLoopRampPeriod(void) const;

	void setVoltageClosedLoopRampPeriod(const double voltage_closed_loop_ramp_period);
	double getVoltageClosedLoopRampPeriod(void) const;

	void setTorqueClosedLoopRampPeriod(const double torque_closed_loop_ramp_period);
	double getTorqueClosedLoopRampPeriod(void) const;

	void setForwardLimitType(const LimitType forward_limit_type);
	LimitType getForwardLimitType(void) const;

	void setForwardLimitAutosetPositionEnable(const bool forward_limit_autoset_position_enable);
	bool getForwardLimitAutosetPositionEnable(void) const;

	void setForwardLimitAutosetPositionValue(const double forward_limit_autoset_position_value);
	double getForwardLimitAutosetPositionValue(void) const;

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
	double getReverseLimitAutosetPositionValue(void) const;

	void setReverseLimitEnable(const bool reverse_limit_enable);
	bool getReverseLimitEnable(void) const;

	void setReverseLimitSource(const LimitSource reverse_limit_source);
	LimitSource getReverseLimitSource(void) const;

	void setReverseLimitRemoteSensorID(const int reverse_limit_remote_sensor_id);
	int getReverseLimitRemoteSensorID(void) const;

	void setBeepOnBoot(const bool beep_on_boot);
	bool getBeepOnBoot(void) const;
	void setBeepOnConfig(const bool beep_on_config);
	bool getBeepOnConfig(void) const;
	void setAllowMusicDurDisable(const bool allow_music_dur_disable);
	bool getAllowMusicDurDisable(void) const;

	void setForwardSoftLimitEnable(const bool enable);
	bool getForwardSoftLimitEnable(void) const;
	void setReverseSoftLimitEnable(const bool enable);
	bool getReverseSoftLimitEnable(void) const;

	void setForwardSoftLimitThreshold(const double threshold);
	double getForwardSoftLimitThreshold(void) const;
	void setReverseSoftLimitThreshold(const double threshold);
	double getReverseSoftLimitThreshold(void) const;

	void setMotionMagicCruiseVelocity(const double motion_magic_cruise_velocity);
	double getMotionMagicCruiseVelocity(void) const;
	void setMotionMagicAcceleration(const double motion_magic_acceleration);
	double getMotionMagicAcceleration(void) const;
	void setMotionMagicJerk(const double motion_magic_jerk);
	double getMotionMagicJerk(void) const;

	void setContinuousWrap(const bool continuous_wrap);
	bool getContinuousWrap(void) const;

	void setClearStickyFaults(void);
	bool getClearStickyFaults(void) const;

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

	void setControlJerk(const double control_acceleration);
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

	void setControlLimitForwardMotion(const bool limit_forward_direction);
	bool getControlLimitForwardMotion(void) const;

	void setControlLimitReverseMotion(const bool limit_reverse_direction);
	bool getControlLimitReverseMotion(void) const;

	void setControlDifferentialPosition(const double control_differential_position);
	double getControlDifferentialPosition(void) const;

	void setControlDifferentialSlot(const int control_differential_slot);
	int getControlDifferentialSlot(void) const;

	void setControlOpposeMasterDirection(const bool control_oppose_master_direction);
	bool getControlOpposeMasterDirection(void) const;

	void setEnableReadThread(const bool enable_read_thread);
	bool getEnableReadThread(void) const;

	void setHasResetOccurred(const bool has_reset_occurred);
	bool getHasResetOccurred(void) const;

	void setVersionMajor(const int version_major);
	int getVersionMajor(void) const;

	void setVersionMinor(const int version_minor);
	int getVersionMinor(void) const;

	void setVersionBugfix(const int version_bugfix);
	int getVersionBugfix(void) const;

	void setVersionBuild(const int version_build);
	int getVersionBuild(void) const;

	void setMotorVoltage(const double motor_voltage);
	double getMotorVoltage(void) const;

	void setForwardLimit(const bool forward_limit);
	bool getForwardLimit(void) const;

	void setReverseLimit(const bool reverse_limit);
	bool getReverseLimit(void) const;

	void setAppliedRotorPolarity(const Inverted applied_rotor_polarity);
	Inverted getAppliedRotorPolarity(void) const;

	void setDutyCycle(const double duty_cycle);
	double getDutyCycle(void) const;

	void setTorqueCurrent(const double torque_current);
	double getTorqueCurrent(void) const;

	void setStatorCurrent(const double stator_current);
	double getStatorCurrent(void) const;

	void setSupplyCurrent(const double supply_current);
	double getSupplyCurrent(void) const;

	void setSupplyVoltage(const double supply_voltage);
	double getSupplyVoltage(void) const;

	void setDeviceTemp(const double device_temp);
	double getDeviceTemp(void) const;

	void setProcessorTemp(const double processor_temp);
	double getProcessorTemp(void) const;

	void setRotorVelocity(const double rotor_velocity);
	double getRotorVelocity(void) const;

	void setRotorPosition(const double rotor_position);
	double getRotorPosition(void) const;

	void setVelocity(const double velocity);
	double getVelocity(void) const;

	void setPosition(const double position);
	double getPosition(void) const;

	void setAcceleration(const double acceleration);
	double getAcceleration(void) const;

	void setMotionMagicIsRunning(const bool motion_magic_is_running);
	bool getMotionMagicIsRunning(void) const;

	void setDeviceEnable(const bool device_enable);
	bool getDeviceEnable(void) const;

	void setDifferentialControlMode(const DifferentialControlMode differential_control_mode);
	DifferentialControlMode getDifferentialControlMode(void) const;

	void setDifferentialAverageVelocity(const double differential_average_velocity);
	double getDifferentialAverageVelocity(void) const;

	void setDifferentialAveragePosition(const double differential_average_position);
	double getDifferentialAveragePosition(void) const;

	void setDifferentialDifferenceVelocity(const double differential_difference_velocity);
	double getDifferentialDifferenceVelocity(void) const;

	void setDifferentialDifferencePosition(const double differential_difference_position);
	double getDifferentialDifferencePosition(void) const;

	void setBridgeOutput(const BridgeOutput bridge_output_value);
	BridgeOutput getBridgeOutput(void) const;

	void setFaultHardware(const bool fault_hardware);
	bool getFaultHardware(void) const;
	void setFaultProcTemp(const bool fault_proctemp);
	bool getFaultProcTemp(void) const;
	void setFaultDeviceTemp(const bool fault_devicetemp);
	bool getFaultDeviceTemp(void) const;
	void setFaultUndervoltage(const bool fault_undervoltage);
	bool getFaultUndervoltage(void) const;
	void setFaultBootDuringEnable(const bool fault_bootduringenable);
	bool getFaultBootDuringEnable(void) const;
	void setFaultBridgeBrownout(const bool fault_bridgebrownout);
	bool getFaultBridgeBrownout(void) const;
	void setFaultUnlicensedFeatureInUse(const bool fault_unlicensed_feature_in_use);
	bool getFaultUnlicensedFeatureInUse(void) const;
	void setFaultRemoteSensorReset(const bool fault_remotesensorreset);
	bool getFaultRemoteSensorReset(void) const;
	void setFaultMissingDifferentialFX(const bool fault_missingdifferentialfx);
	bool getFaultMissingDifferentialFX(void) const;
	void setFaultRemoteSensorPosOverfow(const bool fault_remotesensorposoverflow);
	bool getFaultRemoteSensorPosOverfow(void) const;
	void setFaultOverSupplyV(const bool fault_oversupplyv);
	bool getFaultOverSupplyV(void) const;
	void setFaultUnstableSupplyV(const bool fault_unstablesupplyv);
	bool getFaultUnstableSupplyV(void) const;
	void setFaultReverseHardLimit(const bool fault_reversehardlimit);
	bool getFaultReverseHardLimit(void) const;
	void setFaultForwardHardLimit(const bool fault_forwardhardlimit);
	bool getFaultForwardHardLimit(void) const;
	void setFaultReverseSoftLimit(const bool fault_reversesoftlimit);
	bool getFaultReverseSoftLimit(void) const;
	void setFaultForwardSoftLimit(const bool fault_forwardsoftlimit);
	bool getFaultForwardSoftLimit(void) const;
	void setFaultRemoteSensorDataInvalid(const bool fault_remotesensordatainvalid);
	bool getFaultRemoteSensorDataInvalid(void) const;
	void setFaultFusedSensorOutOfSync(const bool fault_fusedsensoroutofsync);
	bool getFaultFusedSensorOutOfSync(void) const;
	void setFaultStatorCurrLimit(const bool fault_statorcurrlimit);
	bool getFaultStatorCurrLimit(void) const;
	void setFaultSupplyCurrLimit(const bool fault_supplycurrlimit);
	bool getFaultSupplyCurrLimit(void) const;

	void setStickyFaultHardware(const bool sticky_fault_hardware);
	bool getStickyFaultHardware(void) const;
	void setStickyFaultProcTemp(const bool sticky_fault_proctemp);
	bool getStickyFaultProcTemp(void) const;
	void setStickyFaultDeviceTemp(const bool sticky_fault_devicetemp);
	bool getStickyFaultDeviceTemp(void) const;
	void setStickyFaultUndervoltage(const bool sticky_fault_undervoltage);
	bool getStickyFaultUndervoltage(void) const;
	void setStickyFaultBootDuringEnable(const bool sticky_fault_bootduringenable);
	bool getStickyFaultBootDuringEnable(void) const;
	void setStickyFaultBridgeBrownout(const bool sticky_fault_bridgebrownout);
	bool getStickyFaultBridgeBrownout(void) const;
	void setStickyFaultUnlicensedFeatureInUse(const bool sticky_fault_unlicensed_feature_in_use);
	bool getStickyFaultUnlicensedFeatureInUse(void) const;
	void setStickyFaultRemoteSensorReset(const bool sticky_fault_remotesensorreset);
	bool getStickyFaultRemoteSensorReset(void) const;
	void setStickyFaultMissingDifferentialFX(const bool sticky_fault_missingdifferentialfx);
	bool getStickyFaultMissingDifferentialFX(void) const;
	void setStickyFaultRemoteSensorPosOverfow(const bool sticky_fault_remotesensorposoverflow);
	bool getStickyFaultRemoteSensorPosOverfow(void) const;
	void setStickyFaultOverSupplyV(const bool sticky_fault_oversupplyv);
	bool getStickyFaultOverSupplyV(void) const;
	void setStickyFaultUnstableSupplyV(const bool sticky_fault_unstablesupplyv);
	bool getStickyFaultUnstableSupplyV(void) const;
	void setStickyFaultReverseHardLimit(const bool sticky_fault_reversehardlimit);
	bool getStickyFaultReverseHardLimit(void) const;
	void setStickyFaultForwardHardLimit(const bool sticky_fault_forwardhardlimit);
	bool getStickyFaultForwardHardLimit(void) const;
	void setStickyFaultReverseSoftLimit(const bool sticky_fault_reversesoftlimit);
	bool getStickyFaultReverseSoftLimit(void) const;
	void setStickyFaultForwardSoftLimit(const bool sticky_fault_forwardsoftlimit);
	bool getStickyFaultForwardSoftLimit(void) const;
	void setStickyFaultRemoteSensorDataInvalid(const bool sticky_fault_remotesensordatainvalid);
	bool getStickyFaultRemoteSensorDataInvalid(void) const;
	void setStickyFaultFusedSensorOutOfSync(const bool sticky_fault_fusedsensoroutofsync);
	bool getStickyFaultFusedSensorOutOfSync(void) const;
	void setStickyFaultStatorCurrLimit(const bool sticky_fault_statorcurrlimit);
	bool getStickyFaultStatorCurrLimit(void) const;
	void setStickyFaultSupplyCurrLimit(const bool sticky_fault_supplycurrlimit);
	bool getStickyFaultSupplyCurrLimit(void) const;

	void setClosedLoopProportionalOutput(const double closed_loop_proportional_output);
	double getClosedLoopProportionalOutput(void) const;

	void setClosedLoopIntegratedOutput(const double closed_loop_integrated_output);
	double getClosedLoopIntegratedOutput(void) const;

	void setClosedLoopFeedForward(const double closed_loop_feed_forward);
	double getClosedLoopFeedForward(void) const;

	void setClosedLoopDerivativeOutput(const double closed_loop_derivative_output);
	double getClosedLoopDerivativeOutput(void) const;

	void setClosedLoopOutput(const double closed_loop_output);
	double getClosedLoopOutput(void) const;

	void setClosedLoopReference(const double closed_loop_reference);
	double getClosedLoopReference(void) const;

	void setClosedLoopReferenceSlope(const double closed_loop_reference_slope);
	double getClosedLoopReferenceSlope(void) const;

	void setClosedLoopError(const double closed_loop_error);
	double getClosedLoopError(void) const;

	void setDifferentialOutput(const double differential_output);
	double getDifferentialOutput(void) const;

	void setDifferentialClosedLoopProportionalOutput(const double closed_loop_proportional_output);
	double getDifferentialClosedLoopProportionalOutput(void) const;

	void setDifferentialClosedLoopIntegratedOutput(const double closed_loop_integrated_output);
	double getDifferentialClosedLoopIntegratedOutput(void) const;

	void setDifferentialClosedLoopFeedForward(const double closed_loop_feed_forward);
	double getDifferentialClosedLoopFeedForward(void) const;

	void setDifferentialClosedLoopDerivativeOutput(const double closed_loop_derivative_output);
	double getDifferentialClosedLoopDerivativeOutput(void) const;

	void setDifferentialClosedLoopOutput(const double closed_loop_output);
	double getDifferentialClosedLoopOutput(void) const;

	void setDifferentialClosedLoopReference(const double closed_loop_reference);
	double getDifferentialClosedLoopReference(void) const;

	void setDifferentialClosedLoopReferenceSlope(const double closed_loop_reference_slope);
	double getDifferentialClosedLoopReferenceSlope(void) const;

	void setDifferentialClosedLoopError(const double closed_loop_error);
	double getDifferentialClosedLoopError(void) const;

private:
	int can_id_;
	// Slot0Configs ... Slot2Configs
	std::array<double, TALON_PIDF_SLOTS> kP_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kI_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kD_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kS_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kV_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kA_{0.0, 0.0, 0.0};
	std::array<double, TALON_PIDF_SLOTS> kG_{0.0, 0.0, 0.0};
	std::array<GravityType, TALON_PIDF_SLOTS> gravity_type_{GravityType::Elevator_Static};
	
	// MotorOutputConfigs
	Inverted    invert_{Inverted::CounterClockwise_Positive};

	NeutralMode neutral_mode_{NeutralMode::Coast};

	double duty_cycle_neutral_deadband_{0.};
	double peak_forward_duty_cycle_{1.};
	double peak_reverse_duty_cycle_{-1.};

	double stator_current_limit_{0.};
	bool   stator_current_limit_enable_{false};

	double supply_current_limit_{0.};
	bool   supply_current_limit_enable_{false};

	double supply_voltage_time_constant_{0.};
	double peak_forward_voltage_{16.};
	double peak_reverse_voltage_{-16.};

	double peak_forward_torque_current_{800.};
	double peak_reverse_torque_current_{800.};
	double torque_neutral_deadband_{0.0};

	double feedback_rotor_offset_{0.0};
	double sensor_to_mechanism_ratio_{1.0};
	double rotor_to_sensor_ratio_{1.0};
	FeedbackSensorSource feedback_sensor_source_{FeedbackSensorSource::RotorSensor};
	int feedback_remote_sensor_id_{0};

	DifferentialSensorSource differential_sensor_source_{DifferentialSensorSource::Disabled};
	int differential_talonfx_sensor_id_{0};
	int differential_remote_sensor_id_{0};

	double peak_differential_duty_cycle_{2.};
	double peak_differential_voltage_{32.};
	double peak_differential_torque_current_{1600.};

	double duty_cycle_open_loop_ramp_period_{0.};
	double voltage_open_loop_ramp_period_{0.};
	double torque_open_loop_ramp_period_{0.};

	double duty_cycle_closed_loop_ramp_period_{0.};
	double voltage_closed_loop_ramp_period_{0.};
	double torque_closed_loop_ramp_period_{0.};

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

	bool beep_on_boot_{true};
	bool beep_on_config_{true};
	bool allow_music_dur_disable_{false};

	bool   softlimit_forward_enable_{false};
	bool   softlimit_reverse_enable_{false};
	double softlimit_forward_threshold_{0.0};
	double softlimit_reverse_threshold_{0.0};

	double motion_magic_cruise_velocity_{0.0};
	double motion_magic_acceleration_{0.0};
	double motion_magic_jerk_{0.0};

	bool continuous_wrap_{true};

	bool clear_sticky_faults_{false};

	TalonMode control_mode_{TalonMode::Disabled}; 
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
	bool control_limit_forward_motion_{false};
	bool control_limit_reverse_motion_{false};
	double control_differential_position_{0.0};
	int control_differential_slot_{0};
	bool control_oppose_master_direction_{false};

	bool enable_read_thread_{true};

	// Values read from motor
	bool has_reset_occurred_{false};

	int version_major_{0};
	int version_minor_{0};
	int version_bugfix_{0};
	int version_build_{0};

	double motor_voltage_{0};

	bool forward_limit_{false};
	bool reverse_limit_{false};

	Inverted applied_rotor_polarity_{Inverted::CounterClockwise_Positive};

	double duty_cycle_{0};
	double torque_current_{0};
	double stator_current_{0};
	double supply_current_{0};
	double supply_voltage_{0};
	double device_temp_{0};
	double processor_temp_{0};

	double rotor_velocity_{0};
	double rotor_position_{0};
	double velocity_{0};
	double position_{0};
	double acceleration_{0};

	bool motion_magic_is_running_{false};

	bool device_enable_{false};

	DifferentialControlMode differential_control_mode_{DifferentialControlMode::DisabledOutput};
	double differential_average_velocity_{0};
	double differential_average_position_{0};
	double differential_difference_velocity_{0};
	double differential_difference_position_{0};

	BridgeOutput bridge_output_value_{BridgeOutput::Coast};

	bool fault_hardware_{false};
	bool fault_proctemp_{false};
	bool fault_devicetemp_{false};
	bool fault_undervoltage_{false};
	bool fault_bootduringenable_{false};
	bool fault_bridgebrownout_{false};
	bool fault_unlicensed_feature_in_use_{false};
	bool fault_remotesensorreset_{false};
	bool fault_missingdifferentialfx_{false};
	bool fault_remotesensorposoverflow_{false};
	bool fault_oversupplyv_{false};
	bool fault_unstablesupplyv_{false};
	bool fault_reversehardlimit_{false};
	bool fault_forwardhardlimit_{false};
	bool fault_reversesoftlimit_{false};
	bool fault_forwardsoftlimit_{false};
	bool fault_remotesensordatainvalid_{false};
	bool fault_fusedsensoroutofsync_{false};
	bool fault_statorcurrlimit_{false};
	bool fault_supplycurrlimit_{false};

	bool sticky_fault_hardware_{false};
	bool sticky_fault_proctemp_{false};
	bool sticky_fault_devicetemp_{false};
	bool sticky_fault_undervoltage_{false};
	bool sticky_fault_bootduringenable_{false};
	bool sticky_fault_bridgebrownout_{false};
	bool sticky_fault_unlicensed_feature_in_use_{false};
	bool sticky_fault_remotesensorreset_{false};
	bool sticky_fault_missingdifferentialfx_{false};
	bool sticky_fault_remotesensorposoverflow_{false};
	bool sticky_fault_oversupplyv_{false};
	bool sticky_fault_unstablesupplyv_{false};
	bool sticky_fault_reversehardlimit_{false};
	bool sticky_fault_forwardhardlimit_{false};
	bool sticky_fault_reversesoftlimit_{false};
	bool sticky_fault_forwardsoftlimit_{false};
	bool sticky_fault_remotesensordatainvalid_{false};
	bool sticky_fault_fusedsensoroutofsync_{false};
	bool sticky_fault_statorcurrlimit_{false};
	bool sticky_fault_supplycurrlimit_{false};

	double closed_loop_proportional_output_{0};
	double closed_loop_integrated_output_{0};
	double closed_loop_feed_forward_{0};
	double closed_loop_derivative_output_{0};
	double closed_loop_output_{0};
	double closed_loop_reference_{0};
	double closed_loop_reference_slope_{0};
	double closed_loop_error_{0};

	double differential_output_{0};
	double differential_closed_loop_proportional_output_{0};
	double differential_closed_loop_integrated_output_{0};
	double differential_closed_loop_feed_forward_{0};
	double differential_closed_loop_derivative_output_{0};
	double differential_closed_loop_output_{0};
	double differential_closed_loop_reference_{0};
	double differential_closed_loop_reference_slope_{0};
	double differential_closed_loop_error_{0};
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
using TalonFXProStateHandle = StateHandle<const TalonFXProHWState>;
class TalonFXProStateInterface : public HardwareResourceManager<TalonFXProStateHandle> {};

} // namespace

#endif