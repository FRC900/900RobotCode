#ifndef INC_TALONFXPRO_CONTROLLER_INTERFACE__
#define INC_TALONFXPRO_CONTROLLER_INTERFACE__

#include <atomic>

#define TALONCI_BACKWARDS_COMPATIBILITY
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
#include "ctre_interfaces/talon_state_types.h"
#endif
#include "ctre_interfaces/talonfxpro_command_interface.h"
#include "ddr_updater/ddr_updater.h"

// Define a function which reads and returns something from
// the talon's state buffer
#define STATE_PASSTHRU_FN(fn) \
    auto fn(void) const \
    { \
        return talon_.state()->fn(); \
    }

// Add a renamed copy of an existing talon fxpro_fn
// as a newly named function. Used to make an alias
// of an older talon CI function mapped to a new pro function
#define COPY_TALONFX_FN(fxpro_fn, talon_fn) \
    auto talon_fn(void) const \
    { \
        return fxpro_fn(); \
    }


/*
There are several paths used to set config values

1. read from ROS params on startup.  This happens in TalonFXProCIParams init. 
   Talon hw commands updated in TalonFXProControllerInterface
2. dynamic reconfig of the values.  This is handled using ddynamic_reconfigure
   in TalonFXProControllerInterface.  Callbacks from the ddr write are responsible
   for both updating the appropirate CIParams field as well as writing to the
   talon handle
3. explicit write from the controller via calls to TalonFXProControllerInterface 
   set* methods.  This is similar to step 2 in that each call needs to update 
   both the CIParams and call the talon controller interface function to actually
   send the config to the hardawre. Fianlly, the call must trigger a ddr update
   so dynamic reconfig vars are kept in sync with changes via code.

Steps 2 and 3 can be combined into one function by adding an "update ddr" bool flag
to the function.  Calls from path 2 would set this to false, and calls via controller
interface set calls would set it to true.  To make life easier, true would be the default
value for this so external calls wouldn't need to explicitly set the flag.
*/

namespace talonfxpro_controllers
{

class TalonFXProCIParams
{
public:
    TalonFXProCIParams() =default;
    TalonFXProCIParams(const TalonFXProCIParams &) = delete;
    TalonFXProCIParams(TalonFXProCIParams &&) noexcept = delete;
    ~TalonFXProCIParams() = default;

    TalonFXProCIParams& operator=(const TalonFXProCIParams &other);
    TalonFXProCIParams& operator=(TalonFXProCIParams &&other) noexcept = delete;

    bool readJointName(const ros::NodeHandle &n);
    bool readCloseLoopParams(const ros::NodeHandle &n);
    bool readInvert(const ros::NodeHandle &n);
    bool readNeutralMode(const ros::NodeHandle &n);
    bool readStatorCurrentLimits(const ros::NodeHandle &n);
    bool readSupplyCurrentLimits(const ros::NodeHandle &n);
    bool readVoltageLimits(const ros::NodeHandle &n);
    bool readTorqueLimits(const ros::NodeHandle &n);
    bool readFeedback(const ros::NodeHandle &n);
    bool readDifferentialConfig(const ros::NodeHandle &n);
    bool readDifferentialConstants(const ros::NodeHandle &n);
    bool readOpenLoopRamps(const ros::NodeHandle &n);
    bool readClosedLoopRamps(const ros::NodeHandle &n);
    bool readDutyCycleOutputShaping(const ros::NodeHandle &n);
    bool readLimitSwitches(const ros::NodeHandle &n);
    bool readAudio(const ros::NodeHandle &n);
    bool readSoftLimits(const ros::NodeHandle &n);
    bool readMotionMagic(const ros::NodeHandle &n);
    bool readContinuousWrap(const ros::NodeHandle &n);
    bool readControl(const ros::NodeHandle &n);

    bool readEnableReadThread(const ros::NodeHandle &n);

    const std::string &getJointName(void) const;

    std::string joint_name_;

    // Slot0Configs ... Slot2Configs
    std::array<std::atomic<double>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> kP_{0.0, 0.0, 0.0};
    std::array<std::atomic<double>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> kI_{0.0, 0.0, 0.0};
    std::array<std::atomic<double>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> kD_{0.0, 0.0, 0.0};
    std::array<std::atomic<double>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> kS_{0.0, 0.0, 0.0};
    std::array<std::atomic<double>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> kV_{0.0, 0.0, 0.0};
    std::array<std::atomic<double>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> kA_{0.0, 0.0, 0.0};
    std::array<std::atomic<double>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> kG_{0.0, 0.0, 0.0};
	std::array<std::atomic<hardware_interface::talonfxpro::GravityType>, hardware_interface::talonfxpro::TALON_PIDF_SLOTS> gravity_type_{
        hardware_interface::talonfxpro::GravityType::Elevator_Static,
        hardware_interface::talonfxpro::GravityType::Elevator_Static,
        hardware_interface::talonfxpro::GravityType::Elevator_Static,
        };

    std::atomic<hardware_interface::talonfxpro::Inverted> invert_{hardware_interface::talonfxpro::Inverted::CounterClockwise_Positive};
    std::atomic<hardware_interface::talonfxpro::NeutralMode> neutral_mode_{hardware_interface::talonfxpro::NeutralMode::Coast};

    std::atomic<double> duty_cycle_neutral_deadband_{0.};
    std::atomic<double> peak_forward_duty_cycle_{1.};
    std::atomic<double> peak_reverse_duty_cycle_{-1.};

    std::atomic<double> stator_current_limit_{0.};
    std::atomic<bool>   stator_current_limit_enable_{false};

    std::atomic<double> supply_current_limit_{0.};
    std::atomic<bool>   supply_current_limit_enable_{false};

    std::atomic<double> supply_voltage_time_constant_{0.};
    std::atomic<double> peak_forward_voltage_{16.};
    std::atomic<double> peak_reverse_voltage_{-16.};

    std::atomic<double> peak_forward_torque_current_{800.};
    std::atomic<double> peak_reverse_torque_current_{800.};
    std::atomic<double> torque_neutral_deadband_{0.0};

    std::atomic<double> feedback_rotor_offset_{0.0};
    std::atomic<double> sensor_to_mechanism_ratio_{1.0};
    std::atomic<double> rotor_to_sensor_ratio_{1.0};
    std::atomic<hardware_interface::talonfxpro::FeedbackSensorSource> feedback_sensor_source_{hardware_interface::talonfxpro::FeedbackSensorSource::RotorSensor};
    std::atomic<int> feedback_remote_sensor_id_{0};

	std::atomic<hardware_interface::talonfxpro::DifferentialSensorSource> differential_sensor_source_{hardware_interface::talonfxpro::DifferentialSensorSource::Disabled};
	std::atomic<int> differential_talonfx_sensor_id_{0};
	std::atomic<int> differential_remote_sensor_id_{0};

	std::atomic<double> peak_differential_duty_cycle_{2.};
	std::atomic<double> peak_differential_voltage_{32.};
	std::atomic<double> peak_differential_torque_current_{1600.};

    std::atomic<double> duty_cycle_open_loop_ramp_period_{0.};
    std::atomic<double> voltage_open_loop_ramp_period_{0.};
    std::atomic<double> torque_open_loop_ramp_period_{0.};

    std::atomic<double> duty_cycle_closed_loop_ramp_period_{0.};
    std::atomic<double> voltage_closed_loop_ramp_period_{0.};
    std::atomic<double> torque_closed_loop_ramp_period_{0.};

    std::atomic<hardware_interface::talonfxpro::LimitType> forward_limit_type_{hardware_interface::talonfxpro::LimitType::NormallyOpen};
    std::atomic<bool> forward_limit_autoset_position_enable_{false};
    std::atomic<double> forward_limit_autoset_position_value_{0.};
    std::atomic<bool> forward_limit_enable_{true};
    std::atomic<hardware_interface::talonfxpro::LimitSource> forward_limit_source_{hardware_interface::talonfxpro::LimitSource::LimitSwitchPin};
    std::atomic<int> forward_limit_remote_sensor_id_{0};

    std::atomic<hardware_interface::talonfxpro::LimitType> reverse_limit_type_{hardware_interface::talonfxpro::LimitType::NormallyOpen};
    std::atomic<bool> reverse_limit_autoset_position_enable_{false};
    std::atomic<double> reverse_limit_autoset_position_value_{0.};
    std::atomic<bool> reverse_limit_enable_{true};
    std::atomic<hardware_interface::talonfxpro::LimitSource> reverse_limit_source_{hardware_interface::talonfxpro::LimitSource::LimitSwitchPin};
    std::atomic<int> reverse_limit_remote_sensor_id_{0};

    std::atomic<bool> beep_on_boot_{true};
    std::atomic<bool> beep_on_config_{true};
    std::atomic<bool> allow_music_dur_disable_{false};

    std::atomic<bool> softlimit_forward_enable_{false};
    std::atomic<bool> softlimit_reverse_enable_{false};
    std::atomic<double> softlimit_forward_threshold_{0.0};
    std::atomic<double> softlimit_reverse_threshold_{0.0};

    std::atomic<double> motion_magic_cruise_velocity_{0.0};
    std::atomic<double> motion_magic_acceleration_{0.0};
    std::atomic<double> motion_magic_jerk_{0.0};

    std::atomic<bool>   control_enable_foc_{true};
    std::atomic<bool>   control_override_brake_dur_neutral_{false};
    std::atomic<double> control_max_abs_duty_cycle_{1.0};
    std::atomic<double> control_deadband_{0.0};
    std::atomic<double> control_feedforward_{0.0};
    std::atomic<int>    control_slot_{0};
    std::atomic<bool>   control_oppose_master_direction_{false};
    std::atomic<bool>   control_limit_forward_motion_{false};
    std::atomic<bool>   control_limit_reverse_motion_{false};
    std::atomic<int>    control_differential_slot_{0};

    std::atomic<bool> continuous_wrap_{false};
    std::atomic<bool> enable_read_thread_{true};

    std::atomic<double> set_position_{0};
};

class TalonFXProControllerInterface
{
public:
    TalonFXProControllerInterface();
    TalonFXProControllerInterface(const TalonFXProControllerInterface &) = delete;
    TalonFXProControllerInterface(TalonFXProControllerInterface &&) noexcept = delete;
    virtual ~TalonFXProControllerInterface() = default;

    TalonFXProControllerInterface& operator=(const TalonFXProControllerInterface &other) = delete;
    TalonFXProControllerInterface& operator=(TalonFXProControllerInterface &&other) noexcept = delete;
    // Read params from config file and use them to
    // initialize the Talon hardware
    virtual bool initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                              hardware_interface::talonfxpro::TalonFXProStateInterface * /*tsi*/,
                              ros::NodeHandle &n);

    // Same as above, except pass in an array of node handles. First entry is set up
    // as the leader talon and the rest are set in follower mode to follow the leader
    virtual bool initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                              hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                              std::vector<ros::NodeHandle> &n);

    // Init with XmlRpcValue instead of NodeHandle - XmlRpcValue will be either
    // a string or an array of strings of joints to load
    virtual bool initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                              hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                              ros::NodeHandle &controller_nh,
                              XmlRpc::XmlRpcValue param);

    // Same as above, just using a single C++ / C string to name the joint
    virtual bool initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                              hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                              ros::NodeHandle &controller_nh,
                              const std::string &talonfxpro_name);

    virtual bool initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                              hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                              ros::NodeHandle &controller_nh,
                              const char *talonfxpro_name);

    // Functions which are interfaces to TalonFXProCommandInterface control functions
    virtual void setControlMode(const hardware_interface::talonfxpro::TalonMode control_mode);
    virtual void setControlOutput(const double control_output);
    virtual void setControlPosition(const double control_position);
    virtual void setControlVelocity(const double control_velocity);
    virtual void setControlAcceleration(const double control_acceleration);
    virtual void setControlJerk(const double control_jerk);

    virtual void setControlDifferentialPosition(const double control_differential_position);

    // Functions which handle dynamic reconfigurable config vars
    void setkP(const double kP, const size_t index, const bool update_ddr = true);
    void setkI(const double kI, const size_t index, const bool update_ddr = true);
    void setkD(const double kD, const size_t index, const bool update_ddr = true);
    void setkS(const double kS, const size_t index, const bool update_ddr = true);
    void setkV(const double kV, const size_t index, const bool update_ddr = true);
    void setkA(const double kA, const size_t index, const bool update_ddr = true);
    void setkG(const double kG, const size_t index, const bool update_ddr = true);
    void setGravityType(const hardware_interface::talonfxpro::GravityType gravity_type, const size_t index, const bool update_ddr = true);
    void setInvert(const hardware_interface::talonfxpro::Inverted invert, const bool update_ddr = true);
    void setNeutralMode(const hardware_interface::talonfxpro::NeutralMode neutral_mode, const bool update_ddr = true);
    void setDutyCycleNeutralDeadband(const double duty_cycle_neutral_deadband, const bool update_ddr = true);
    void setPeakForwardDutyCycle(const double peak_forward_duty_cycle, const bool update_ddr = true);
    void setPeakReverseDutyCycle(const double peak_reverse_duty_cycle, const bool update_ddr = true);
    void setStatorCurrentLimit(const double stator_current_limit, const bool update_ddr = true);
    void setStatorCurrentLimitEnable(const bool stator_current_limit_enable, const bool update_ddr = true);
    void setSupplyCurrentLimit(const double supply_current_limit, const bool update_ddr = true);
    void setSupplyCurrentLimitEnable(const bool supply_current_limit_enable, const bool update_ddr = true);
    void setSupplyVoltageTimeConstant(const double supply_voltage_time_constant, const bool update_ddr = true);
    void setPeakForwardVoltage(const double peak_forward_voltage, const bool update_ddr = true);
    void setPeakReverseVoltage(const double peak_reverse_voltage, const bool update_ddr = true);
    void setPeakForwardTorqueCurrent(const double peak_forward_torque_current, const bool update_ddr = true);
    void setPeakReverseTorqueCurrent(const double peak_reverse_torque_current, const bool update_ddr = true);
    void setTorqueNeutralDeadband(const double torque_neutral_deadband, const bool update_ddr = true);
    void setFeedbackRotorOffset(const double feedback_rotor_offset, const bool update_ddr = true);
    void setSensorToMechanismRatio(const double sensor_to_mechanism_ratio, const bool update_ddr = true);
    void setRotorToSensorRatio(const double rotor_to_sensor_ratio, const bool update_ddr = true);
    void setDifferentialSensorSource(const hardware_interface::talonfxpro::DifferentialSensorSource differential_sensor_source, const bool update_ddr = true);
    void setDifferentialTalonFXSensorID(const int differential_talonfx_sensor_id, const bool update_ddr = true);
    void setDifferentialRemoteSensorID(const int differential_remote_sensor_id, const bool update_ddr = true);
    void setPeakDifferentialDutyCycle(const double peak_differential_duty_cycle, const bool update_ddr = true);
    void setPeakDifferentialVoltage(const double peak_differential_voltage, const bool update_ddr = true);
    void setPeakDifferentialTorqueCurrent(const double peak_differential_torque_current, const bool update_ddr = true);
    void setDutyCycleOpenLoopRampPeriod(const double duty_cycle_open_loop_ramp_period, const bool update_ddr = true);
    void setVoltageOpenLoopRampPeriod(const double voltage_open_loop_ramp_period, const bool update_ddr = true);
    void setTorqueOpenLoopRampPeriod(const double torque_open_loop_ramp_period, const bool update_ddr = true);
    void setDutyCycleClosedLoopRampPeriod(const double duty_cycle_closed_loop_ramp_period, const bool update_ddr = true);
    void setVoltageClosedLoopRampPeriod(const double voltage_closed_loop_ramp_period, const bool update_ddr = true);
    void setTorqueClosedLoopRampPeriod(const double torque_closed_loop_ramp_period, const bool update_ddr = true);
    void setForwardLimitType(const hardware_interface::talonfxpro::LimitType forward_limit_type, const bool update_ddr = true);
    void setForwardLimitAutosetPositionEnable(const bool forward_limit_autoset_position_enable, const bool update_ddr = true);
    void setForwardLimitAutosetPositionValue(const double forward_limit_autoset_position_value, const bool update_ddr = true);
    void setForwardLimitEnable(const bool forward_limit_enable, const bool update_ddr = true);
    void setForwardLimitSource(const hardware_interface::talonfxpro::LimitSource forward_limit_source, const bool update_ddr = true);
    void setForwardLimitRemoteSensorID(const int forward_limit_remote_sensor_id, const bool update_ddr = true);
    void setReverseLimitType(const hardware_interface::talonfxpro::LimitType reverse_limit_type, const bool update_ddr = true);
    void setReverseLimitAutosetPositionEnable(const bool reverse_limit_autoset_position_enable, const bool update_ddr = true);
    void setReverseLimitAutosetPositionValue(const double reverse_limit_autoset_position_value, const bool update_ddr = true);
    void setReverseLimitEnable(const bool reverse_limit_enable, const bool update_ddr = true);
    void setReverseLimitSource(const hardware_interface::talonfxpro::LimitSource reverse_limit_source, const bool update_ddr = true);
    void setReverseLimitRemoteSensorID(const int reverse_limit_remote_sensor_id, const bool update_ddr = true);
    void setBeepOnBoot(const bool beep_on_boot, const bool update_ddr = true);
    void setBeepOnConfig(const bool beep_on_config, const bool update_ddr = true);
    void setAllowMusicDurDisable(const bool allow_music_dur_disable, const bool update_ddr = true);
    void setForwardSoftLimitEnable(const bool forward_softlimit_enable, const bool update_ddr = true);
    void setReverseSoftLimitEnable(const bool reverse_softlimit_enable, const bool update_ddr = true);
    void setForwardSoftLimitThreshold(const double forward_softlimit_threshold, const bool update_ddr = true);
    void setReverseSoftLimitThreshold(const double reverse_softlimit_threshold, const bool update_ddr = true);
    void setMotionMagicCruiseVelocity(const double motion_magic_cruise_velocity, const bool update_ddr = true);
    void setMotionMagicAcceleration(const double motion_magic_acceleration, const bool update_ddr = true);
    void setMotionMagicJerk(const double motion_magic_jerk, const bool update_ddr = true);
    void setContinuousWrap(const bool continuous_wrap, const bool update_ddr = true);
    void setControlEnableFOC(const bool control_enable_foc, const bool update_ddr = true);
    void setControlOverrideBrakeDurNeutral(const bool control_override_brake_dur_neutral, const bool update_ddr = true);
    void setControlMaxAbsDutyCycle(const double control_max_abs_duty_cycle, const bool update_ddr = true);
    void setControlDeadband(const double control_deadband, const bool update_ddr = true);
    void setControlFeedforward(const double control_feedforward, const bool update_ddr = true);
    void setControlSlot(const int control_slot, const bool update_ddr = true);
    void setControlOpposeMasterDirection(const bool control_oppose_master_direction, const bool update_ddr = true);
    void setControlLimitForwardMotion(const bool control_limit_forward_motion, const bool update_ddr = true);
    void setControlLimitReverseMotion(const bool control_limit_reverse_motion, const bool update_ddr = true);
    void setControlDifferentialSlot(const int control_differential_slot, const bool update_ddr = true);
    void setEnableReadThread(const bool enable_read_thread, const bool update_ddr = true);
    void setRotorPosition(const double set_position, const bool update_ddr = true);

    // Long list of functions which allow access of state interface vars
	STATE_PASSTHRU_FN(getCANID);
    double getkP(const int index) const { return talon_.state()->getkP(index); }
    double getkI(const int index) const { return talon_.state()->getkI(index); }
    double getkD(const int index) const { return talon_.state()->getkD(index); }
    double getkS(const int index) const { return talon_.state()->getkS(index); }
    double getkV(const int index) const { return talon_.state()->getkV(index); }
    double getkA(const int index) const { return talon_.state()->getkA(index); }
    double getkG(const int index) const { return talon_.state()->getkG(index); }
    hardware_interface::talonfxpro::GravityType getGravityType(const size_t index) const { return talon_.state()->getGravityType(index); }
	STATE_PASSTHRU_FN(getInvert)
	STATE_PASSTHRU_FN(getNeutralMode)
	STATE_PASSTHRU_FN(getDutyCycleNeutralDeadband)
	STATE_PASSTHRU_FN(getPeakForwardDutyCycle)
	STATE_PASSTHRU_FN(getPeakReverseDutyCycle)
	STATE_PASSTHRU_FN(getStatorCurrentLimit)
	STATE_PASSTHRU_FN(getStatorCurrentLimitEnable)
	STATE_PASSTHRU_FN(getSupplyCurrentLimit)
	STATE_PASSTHRU_FN(getSupplyCurrentLimitEnable)
	STATE_PASSTHRU_FN(getSupplyVoltageTimeConstant)
	STATE_PASSTHRU_FN(getPeakForwardVoltage)
	STATE_PASSTHRU_FN(getPeakReverseVoltage)
	STATE_PASSTHRU_FN(getPeakForwardTorqueCurrent)
	STATE_PASSTHRU_FN(getPeakReverseTorqueCurrent)
	STATE_PASSTHRU_FN(getTorqueNeutralDeadband)
	STATE_PASSTHRU_FN(getFeedbackRotorOffset)
	STATE_PASSTHRU_FN(getSensorToMechanismRatio)
	STATE_PASSTHRU_FN(getRotorToSensorRatio)
	STATE_PASSTHRU_FN(getFeedbackSensorSource)
	STATE_PASSTHRU_FN(getFeedbackRemoteSensorID)
	STATE_PASSTHRU_FN(getDutyCycleOpenLoopRampPeriod)
	STATE_PASSTHRU_FN(getVoltageOpenLoopRampPeriod)
	STATE_PASSTHRU_FN(getTorqueOpenLoopRampPeriod)
	STATE_PASSTHRU_FN(getDutyCycleClosedLoopRampPeriod)
	STATE_PASSTHRU_FN(getVoltageClosedLoopRampPeriod)
	STATE_PASSTHRU_FN(getTorqueClosedLoopRampPeriod)
	STATE_PASSTHRU_FN(getForwardLimitType)
	STATE_PASSTHRU_FN(getForwardLimitAutosetPositionEnable)
	STATE_PASSTHRU_FN(getForwardLimitAutosetPositionValue)
	STATE_PASSTHRU_FN(getForwardLimitEnable)
	STATE_PASSTHRU_FN(getForwardLimitSource)
	STATE_PASSTHRU_FN(getForwardLimitRemoteSensorID)
	STATE_PASSTHRU_FN(getReverseLimitType)
	STATE_PASSTHRU_FN(getReverseLimitAutosetPositionEnable)
	STATE_PASSTHRU_FN(getReverseLimitAutosetPositionValue)
	STATE_PASSTHRU_FN(getReverseLimitEnable)
	STATE_PASSTHRU_FN(getReverseLimitSource)
	STATE_PASSTHRU_FN(getReverseLimitRemoteSensorID)
	STATE_PASSTHRU_FN(getBeepOnBoot)
	STATE_PASSTHRU_FN(getBeepOnConfig)
	STATE_PASSTHRU_FN(getAllowMusicDurDisable)
	STATE_PASSTHRU_FN(getForwardSoftLimitEnable)
	STATE_PASSTHRU_FN(getReverseSoftLimitEnable)
	STATE_PASSTHRU_FN(getForwardSoftLimitThreshold)
	STATE_PASSTHRU_FN(getReverseSoftLimitThreshold)
	STATE_PASSTHRU_FN(getMotionMagicCruiseVelocity)
	STATE_PASSTHRU_FN(getMotionMagicAcceleration)
	STATE_PASSTHRU_FN(getMotionMagicJerk)
	STATE_PASSTHRU_FN(getContinuousWrap)
	STATE_PASSTHRU_FN(getClearStickyFaults)
	STATE_PASSTHRU_FN(getControlMode)
	STATE_PASSTHRU_FN(getControlOutput)
	STATE_PASSTHRU_FN(getControlPosition)
	STATE_PASSTHRU_FN(getControlVelocity)
	STATE_PASSTHRU_FN(getControlAcceleration)
	STATE_PASSTHRU_FN(getControlEnableFOC)
	STATE_PASSTHRU_FN(getControlOverrideBrakeDurNeutral)
	STATE_PASSTHRU_FN(getControlMaxAbsDutyCycle)
	STATE_PASSTHRU_FN(getControlDeadband)
	STATE_PASSTHRU_FN(getControlFeedforward)
	STATE_PASSTHRU_FN(getControlSlot)
	STATE_PASSTHRU_FN(getControlOpposeMasterDirection)
    STATE_PASSTHRU_FN(getControlLimitForwardMotion);
    STATE_PASSTHRU_FN(getControlLimitReverseMotion);
    STATE_PASSTHRU_FN(getControlDifferentialPosition)
	STATE_PASSTHRU_FN(getControlDifferentialSlot)
	STATE_PASSTHRU_FN(getEnableReadThread)
	STATE_PASSTHRU_FN(getHasResetOccurred)
	STATE_PASSTHRU_FN(getVersionMajor)
	STATE_PASSTHRU_FN(getVersionMinor)
	STATE_PASSTHRU_FN(getVersionBugfix)
	STATE_PASSTHRU_FN(getVersionBuild)
	STATE_PASSTHRU_FN(getMotorVoltage)
	STATE_PASSTHRU_FN(getForwardLimit)
	STATE_PASSTHRU_FN(getReverseLimit)
	STATE_PASSTHRU_FN(getAppliedRotorPolarity)
	STATE_PASSTHRU_FN(getDutyCycle)
	STATE_PASSTHRU_FN(getTorqueCurrent)
	STATE_PASSTHRU_FN(getStatorCurrent)
	STATE_PASSTHRU_FN(getSupplyCurrent)
	STATE_PASSTHRU_FN(getSupplyVoltage)
    STATE_PASSTHRU_FN(getDeviceTemp)
    STATE_PASSTHRU_FN(getProcessorTemp)
    STATE_PASSTHRU_FN(getRotorVelocity)
    STATE_PASSTHRU_FN(getRotorPosition)
    STATE_PASSTHRU_FN(getVelocity)
    STATE_PASSTHRU_FN(getPosition)
    STATE_PASSTHRU_FN(getAcceleration)
    STATE_PASSTHRU_FN(getMotionMagicIsRunning)
    STATE_PASSTHRU_FN(getDeviceEnable)
    STATE_PASSTHRU_FN(getDifferentialControlMode)
    STATE_PASSTHRU_FN(getDifferentialAverageVelocity)
    STATE_PASSTHRU_FN(getDifferentialAveragePosition)
    STATE_PASSTHRU_FN(getDifferentialDifferenceVelocity)
    STATE_PASSTHRU_FN(getDifferentialDifferencePosition)
    STATE_PASSTHRU_FN(getBridgeOutput)
    STATE_PASSTHRU_FN(getFaultHardware)
    STATE_PASSTHRU_FN(getFaultProcTemp)
    STATE_PASSTHRU_FN(getFaultDeviceTemp)
    STATE_PASSTHRU_FN(getFaultUndervoltage)
    STATE_PASSTHRU_FN(getFaultBootDuringEnable)
    STATE_PASSTHRU_FN(getFaultBridgeBrownout)
    STATE_PASSTHRU_FN(getFaultUnlicensedFeatureInUse)
    STATE_PASSTHRU_FN(getFaultRemoteSensorReset)
    STATE_PASSTHRU_FN(getFaultMissingDifferentialFX)
    STATE_PASSTHRU_FN(getFaultRemoteSensorPosOverfow)
    STATE_PASSTHRU_FN(getFaultOverSupplyV)
    STATE_PASSTHRU_FN(getFaultUnstableSupplyV)
    STATE_PASSTHRU_FN(getFaultReverseHardLimit)
    STATE_PASSTHRU_FN(getFaultForwardHardLimit)
    STATE_PASSTHRU_FN(getFaultReverseSoftLimit)
    STATE_PASSTHRU_FN(getFaultForwardSoftLimit)
    STATE_PASSTHRU_FN(getFaultRemoteSensorDataInvalid)
    STATE_PASSTHRU_FN(getFaultFusedSensorOutOfSync)
    STATE_PASSTHRU_FN(getFaultStatorCurrLimit)
    STATE_PASSTHRU_FN(getFaultSupplyCurrLimit)
    STATE_PASSTHRU_FN(getStickyFaultHardware)
    STATE_PASSTHRU_FN(getStickyFaultProcTemp)
    STATE_PASSTHRU_FN(getStickyFaultDeviceTemp)
    STATE_PASSTHRU_FN(getStickyFaultUndervoltage)
    STATE_PASSTHRU_FN(getStickyFaultBootDuringEnable)
    STATE_PASSTHRU_FN(getStickyFaultBridgeBrownout)
    STATE_PASSTHRU_FN(getStickyFaultUnlicensedFeatureInUse)
    STATE_PASSTHRU_FN(getStickyFaultRemoteSensorReset)
    STATE_PASSTHRU_FN(getStickyFaultMissingDifferentialFX)
    STATE_PASSTHRU_FN(getStickyFaultRemoteSensorPosOverfow)
    STATE_PASSTHRU_FN(getStickyFaultOverSupplyV)
    STATE_PASSTHRU_FN(getStickyFaultUnstableSupplyV)
    STATE_PASSTHRU_FN(getStickyFaultReverseHardLimit)
    STATE_PASSTHRU_FN(getStickyFaultForwardHardLimit)
    STATE_PASSTHRU_FN(getStickyFaultReverseSoftLimit)
    STATE_PASSTHRU_FN(getStickyFaultForwardSoftLimit)
    STATE_PASSTHRU_FN(getStickyFaultRemoteSensorDataInvalid)
    STATE_PASSTHRU_FN(getStickyFaultFusedSensorOutOfSync)
    STATE_PASSTHRU_FN(getStickyFaultStatorCurrLimit)
    STATE_PASSTHRU_FN(getStickyFaultSupplyCurrLimit)
    STATE_PASSTHRU_FN(getClosedLoopProportionalOutput)
    STATE_PASSTHRU_FN(getClosedLoopIntegratedOutput)
    STATE_PASSTHRU_FN(getClosedLoopFeedForward)
    STATE_PASSTHRU_FN(getClosedLoopDerivativeOutput)
    STATE_PASSTHRU_FN(getClosedLoopOutput)
    STATE_PASSTHRU_FN(getClosedLoopReference)
    STATE_PASSTHRU_FN(getClosedLoopReferenceSlope)
    STATE_PASSTHRU_FN(getClosedLoopError)
    STATE_PASSTHRU_FN(getDifferentialOutput)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopProportionalOutput)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopIntegratedOutput)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopFeedForward)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopDerivativeOutput)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopOutput)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopReference)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopReferenceSlope)
    STATE_PASSTHRU_FN(getDifferentialClosedLoopError)

// Shim functions to hook up V5 calls from controllers to updated V6 functions
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    COPY_TALONFX_FN(getForwardLimit, getForwardLimitSwitch)
    COPY_TALONFX_FN(getReverseLimit, getReverseLimitSwitch)
    COPY_TALONFX_FN(getVelocity, getSpeed)

    void setDemand1Type(const hardware_interface::DemandType demand_type) const;
    void setDemand1Value(const double demand_value);
    void setCommand(const double command);
    void setMotionCruiseVelocity(const double motion_cruise_velocity);
    void setMotionAcceleration(const double motion_acceleration);
    void setMotionSCurveStrength(const double motion_s_curve_strength);
    void setPIDFSlot(const int slot);
    void setSelectedSensorPosition(const double sensor_position);
    void setNeutralMode(const hardware_interface::NeutralMode neutral_mode);

    hardware_interface::TalonMode getMode(void) const;
    void setMode(const hardware_interface::TalonMode mode);
#endif

private:
    bool init(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
              ros::NodeHandle &n,
              hardware_interface::talonfxpro::TalonFXProCommandHandle &talon,
              bool follower);
    bool readParams(const ros::NodeHandle &n, TalonFXProCIParams &params) const;
    void writeParamsToHW(TalonFXProCIParams &params, hardware_interface::talonfxpro::TalonFXProCommandHandle &talon) const;

    std::unique_ptr<ddr_updater::DDRUpdater> ddr_updater_;

protected:
    TalonFXProCIParams params_;
    // The talon being controlled via this controller interface
    hardware_interface::talonfxpro::TalonFXProCommandHandle talon_;

    // List of follower talons associated with the master
    // listed above
    std::vector<hardware_interface::talonfxpro::TalonFXProCommandHandle> follower_talons_;

    virtual bool setInitialControlMode(void);

    static inline const std::map<std::string, int> limit_type_enum_map_ {
        {"NormallyOpen", static_cast<int>(hardware_interface::talonfxpro::LimitType::NormallyOpen)},
        {"NormallyClosed", static_cast<int>(hardware_interface::talonfxpro::LimitType::NormallyClosed)}
    };
    static inline const std::map<std::string, int> limit_source_enum_map_ {
        {"LimitSwitchPin", static_cast<int>(hardware_interface::talonfxpro::LimitSource::LimitSwitchPin)},
        {"RemoteTalonFX", static_cast<int>(hardware_interface::talonfxpro::LimitSource::RemoteTalonFX)},
        {"RemoteCANifier", static_cast<int>(hardware_interface::talonfxpro::LimitSource::RemoteCANifier)},
        {"RemoteCANcoder", static_cast<int>(hardware_interface::talonfxpro::LimitSource::RemoteCANcoder)},
        {"Disabled", static_cast<int>(hardware_interface::talonfxpro::LimitSource::Disabled)}
    };
};

// A derived class which disables mode switching. Any single-mode CI class should derive from this class
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
class TalonFXProFixedModeControllerInterface : public TalonFXProControllerInterface
{
public:
    TalonFXProFixedModeControllerInterface() : TalonFXProControllerInterface() {}
    TalonFXProFixedModeControllerInterface(const TalonFXProFixedModeControllerInterface &) = delete; 
    TalonFXProFixedModeControllerInterface(TalonFXProFixedModeControllerInterface &&) noexcept = delete;
    ~TalonFXProFixedModeControllerInterface() override = default;

    TalonFXProFixedModeControllerInterface &operator=(const TalonFXProFixedModeControllerInterface &other) = delete;
    TalonFXProFixedModeControllerInterface &operator=(TalonFXProFixedModeControllerInterface &&other) noexcept = delete;

    void setControlMode(const hardware_interface::talonfxpro::TalonMode mode) override;
    void setControlJerk(const double control_jerk) override;

protected:
    // Disable changing mode for controllers derived from this class
    bool setInitialControlMode(void) override;
};

// A derived class which emits a warning on potentially invalid control output calls
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
class TalonFXProPositionControllerInterface : public TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>
{
public:
    TalonFXProPositionControllerInterface() : TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>() {}
    TalonFXProPositionControllerInterface(const TalonFXProPositionControllerInterface &) = delete; 
    TalonFXProPositionControllerInterface(TalonFXProPositionControllerInterface &&) noexcept = delete;
    virtual ~TalonFXProPositionControllerInterface() = default;

    TalonFXProPositionControllerInterface &operator=(const TalonFXProPositionControllerInterface &other) = delete;
    TalonFXProPositionControllerInterface &operator=(TalonFXProPositionControllerInterface &&other) noexcept = delete;

    void setControlMode(const hardware_interface::talonfxpro::TalonMode mode) override;
    void setControlOutput(const double control_output) override;
    void setControlAcceleration(const double control_acceleration) override;
};

// A derived class which emits a warning on potentially invalid control output calls
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
class TalonFXProVelocityControllerInterface : public TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>
{
public:
    TalonFXProVelocityControllerInterface() : TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>() {}
    TalonFXProVelocityControllerInterface(const TalonFXProVelocityControllerInterface &) = delete; 
    TalonFXProVelocityControllerInterface(TalonFXProVelocityControllerInterface &&) noexcept = delete;
    virtual ~TalonFXProVelocityControllerInterface() = default;

    TalonFXProVelocityControllerInterface &operator=(const TalonFXProVelocityControllerInterface &other) = delete;
    TalonFXProVelocityControllerInterface &operator=(TalonFXProVelocityControllerInterface &&other) noexcept = delete;

    void setControlOutput(const double control_output) override;
    void setControlPosition(const double control_position) override;
};

// Derived classes which emits a warning on potentially invalid control output calls
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
class TalonFXProMotionMagicControllerInterface : public TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>
{
public:
    TalonFXProMotionMagicControllerInterface() : TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>() {}
    TalonFXProMotionMagicControllerInterface(const TalonFXProMotionMagicControllerInterface &) = delete; 
    TalonFXProMotionMagicControllerInterface(TalonFXProMotionMagicControllerInterface &&) noexcept = delete;
    virtual ~TalonFXProMotionMagicControllerInterface() = default;

    TalonFXProMotionMagicControllerInterface &operator=(const TalonFXProMotionMagicControllerInterface &other) = delete;
    TalonFXProMotionMagicControllerInterface &operator=(TalonFXProMotionMagicControllerInterface &&other) noexcept = delete;

    void setControlOutput(const double control_output) override;
    void setControlVelocity(const double control_velocity) override;
    void setControlAcceleration(const double control_acceleration) override;
};

template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
class TalonFXProMotionMagicVelocityControllerInterface : public TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>
{
public:
    TalonFXProMotionMagicVelocityControllerInterface() : TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>() {}
    TalonFXProMotionMagicVelocityControllerInterface(const TalonFXProMotionMagicVelocityControllerInterface &) = delete; 
    TalonFXProMotionMagicVelocityControllerInterface(TalonFXProMotionMagicVelocityControllerInterface &&) noexcept = delete;
    virtual ~TalonFXProMotionMagicVelocityControllerInterface() = default;

    TalonFXProMotionMagicVelocityControllerInterface &operator=(const TalonFXProMotionMagicVelocityControllerInterface &other) = delete;
    TalonFXProMotionMagicVelocityControllerInterface &operator=(TalonFXProMotionMagicVelocityControllerInterface &&other) noexcept = delete;

    void setControlOutput(const double control_output) override;
    void setControlPosition(const double control_position) override;
    void setControlAcceleration(const double control_acceleration) override;
};

template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
class TalonFXProDynamicMotionMagicControllerInterface : public TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>
{
public:
    TalonFXProDynamicMotionMagicControllerInterface() : TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>() {}
    TalonFXProDynamicMotionMagicControllerInterface(const TalonFXProDynamicMotionMagicControllerInterface &) = delete; 
    TalonFXProDynamicMotionMagicControllerInterface(TalonFXProDynamicMotionMagicControllerInterface &&) noexcept = delete;
    virtual ~TalonFXProDynamicMotionMagicControllerInterface() = default;

    TalonFXProDynamicMotionMagicControllerInterface &operator=(const TalonFXProDynamicMotionMagicControllerInterface &other) = delete;
    TalonFXProDynamicMotionMagicControllerInterface &operator=(TalonFXProDynamicMotionMagicControllerInterface &&other) noexcept = delete;

    void setControlOutput(const double control_output) override;
    void setControlJerk(const double control_jerk) override;
};

extern const char DUTY_CYCLE_NAME[];
extern const char TORQUE_CURRENT_FOC_NAME[];
extern const char VOLTAGE_NAME[];
extern const char POSITION_DUTY_CYCLE_NAME[];
extern const char POSITION_VOLTAGE_NAME[];
extern const char POSITION_TORQUE_CURRENT_FOC_NAME[];
extern const char VELOCITY_DUTY_CYCLE_NAME[];
extern const char VELOCITY_VOLTAGE_NAME[];
extern const char VELOCITY_TORQUE_CURRENT_FOC_NAME[];
extern const char MOTION_MAGIC_DUTY_CYCLE_NAME[];
extern const char MOTION_MAGIC_VOLTAGE_NAME[];
extern const char MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME[];
extern const char MOTION_MAGIC_EXPO_DUTY_CYCLE_NAME[];
extern const char MOTION_MAGIC_EXPO_VOLTAGE_NAME[];
extern const char MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC_NAME[];
extern const char MOTION_MAGIC_VELOCITY_DUTY_CYCLE_NAME[];
extern const char MOTION_MAGIC_VELOCITY_VOLTAGE_NAME[];
extern const char MOTION_MAGIC_VELOCITY_TORQUE_CURRENT_FOC_NAME[];
extern const char DYNAMIC_MOTION_MAGIC_DUTY_CYCLE_NAME[];
extern const char DYNAMIC_MOTION_MAGIC_VOLTAGE_NAME[];
extern const char DYNAMIC_MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME[];
using TalonFXProDutyCycleOutControllerInterface = TalonFXProFixedModeControllerInterface<hardware_interface::talonfxpro::TalonMode::DutyCycleOut, DUTY_CYCLE_NAME>;
using TalonFXProTorqueCurrentFOCControllerInterface = TalonFXProFixedModeControllerInterface<hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC, TORQUE_CURRENT_FOC_NAME>;
using TalonFXProVoltageOutControllerInterface = TalonFXProFixedModeControllerInterface<hardware_interface::talonfxpro::TalonMode::VoltageOut, VOLTAGE_NAME>;
using TalonFXProPositionDutyCycleControllerInterface = TalonFXProPositionControllerInterface<hardware_interface::talonfxpro::TalonMode::PositionDutyCycle, POSITION_DUTY_CYCLE_NAME>;
using TalonFXProPositionVoltageControllerInterface = TalonFXProPositionControllerInterface<hardware_interface::talonfxpro::TalonMode::PositionVoltage, POSITION_VOLTAGE_NAME>;
using TalonFXProPositionTorqueCurrentFOCControllerInterface = TalonFXProPositionControllerInterface<hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC, POSITION_TORQUE_CURRENT_FOC_NAME>;
using TalonFXProVelocityDutyCycleControllerInterface = TalonFXProVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle, VELOCITY_DUTY_CYCLE_NAME>;
using TalonFXProVelocityVoltageControllerInterface = TalonFXProVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::VelocityVoltage, VELOCITY_VOLTAGE_NAME>;
using TalonFXProVelocityTorqueCurrentFOCControllerInterface = TalonFXProVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC, VELOCITY_TORQUE_CURRENT_FOC_NAME>;
using TalonFXProMotionMagicDutyCycleControllerInterface = TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle, MOTION_MAGIC_DUTY_CYCLE_NAME>;
using TalonFXProMotionMagicVoltageControllerInterface = TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage, MOTION_MAGIC_VOLTAGE_NAME>;
using TalonFXProMotionMagicTorqueCurrentFOCControllerInterface = TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC, MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME>;
using TalonFXProMotionMagicExpoDutyCycleControllerInterface = TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicExpoDutyCycle, MOTION_MAGIC_EXPO_DUTY_CYCLE_NAME>;
using TalonFXProMotionMagicExpoVoltageControllerInterface = TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage, MOTION_MAGIC_EXPO_VOLTAGE_NAME>;
using TalonFXProMotionMagicExpoTorqueCurrentFOCControllerInterface = TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicExpoTorqueCurrentFOC, MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC_NAME>;
using TalonFXProMotionMagicVelocityDutyCycleControllerInterface = TalonFXProMotionMagicVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle, MOTION_MAGIC_VELOCITY_DUTY_CYCLE_NAME>;
using TalonFXProMotionMagicVelocityVoltageControllerInterface = TalonFXProMotionMagicVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage, MOTION_MAGIC_VELOCITY_VOLTAGE_NAME>;
using TalonFXProMotionMagicVelocityTorqueCurrentFOCControllerInterface = TalonFXProMotionMagicVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC, MOTION_MAGIC_VELOCITY_TORQUE_CURRENT_FOC_NAME>;
using TalonFXProDynamicMotionMagicDutyCycleControllerInterface = TalonFXProDynamicMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle, DYNAMIC_MOTION_MAGIC_DUTY_CYCLE_NAME>;
using TalonFXProDynamicMotionMagicVoltageControllerInterface = TalonFXProDynamicMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage, DYNAMIC_MOTION_MAGIC_VOLTAGE_NAME>;
using TalonFXProDynamicMotionMagicTorqueCurrentFOCControllerInterface = TalonFXProDynamicMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC, DYNAMIC_MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME>;

template <bool STRICT>
class TalonFXProFollowerControllerInterfaceBase : public TalonFXProControllerInterface
{
public:
    TalonFXProFollowerControllerInterfaceBase()  : TalonFXProControllerInterface() {}
    TalonFXProFollowerControllerInterfaceBase(const TalonFXProFollowerControllerInterfaceBase &) = delete;
    TalonFXProFollowerControllerInterfaceBase(TalonFXProFollowerControllerInterfaceBase &&) noexcept = delete;
    ~TalonFXProFollowerControllerInterfaceBase() override = default;

    TalonFXProFollowerControllerInterfaceBase &operator=(const TalonFXProFollowerControllerInterfaceBase &other) = delete;
    TalonFXProFollowerControllerInterfaceBase &operator=(TalonFXProFollowerControllerInterfaceBase &&other) noexcept = delete;
    bool initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                      hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                      ros::NodeHandle &n) override;

    // Disable changing mode for controllers derived from this class
    void setControlMode(const hardware_interface::talonfxpro::TalonMode /*mode*/) override;

    // Disable all of these since followers can't set their control output
    void setControlOutput(const double /*control_output*/) override;
    void setControlPosition(const double /*control_position*/) override;
    void setControlVelocity(const double /*control_velocity*/) override;
    void setControlAcceleration(const double /*control_acceleration*/) override;
};

using TalonFXProFollowerControllerInterface = TalonFXProFollowerControllerInterfaceBase<false>;
using TalonFXProStrictFollowerControllerInterface = TalonFXProFollowerControllerInterfaceBase<true>;

// Not adding NeutralOut, CoastOut or StaticBrake - if we want a controller which only
// does nothing, why is there a motor there in the first place? Those modes will be used
// on a variable mode controller only.
}

#endif