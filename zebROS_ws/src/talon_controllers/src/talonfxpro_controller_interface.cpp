#include "talon_controllers/talonfxpro_controller_interface.h"
#include "ctre_interfaces/talon_state_types.h"

namespace talonfxpro_controllers
{
const char DUTY_CYCLE_NAME[] = "DutyCycleOut";
const char TORQUE_CURRENT_FOC_NAME[] = "TorqueCurrentFOC";
const char VOLTAGE_NAME[] = "VoltageOut";
const char POSITION_DUTY_CYCLE_NAME[] = "PositionDutyCycle";
const char POSITION_VOLTAGE_NAME[] = "PositionVoltage";
const char POSITION_TORQUE_CURRENT_FOC_NAME[] = "PositionTorqueCurrentFOC";
const char VELOCITY_DUTY_CYCLE_NAME[] = "VelocityDutyCycle";
const char VELOCITY_VOLTAGE_NAME[] = "VelocityVoltage";
const char VELOCITY_TORQUE_CURRENT_FOC_NAME[] = "VelocityTorqueCurrentFOC";
const char MOTION_MAGIC_DUTY_CYCLE_NAME[] = "MotionMagicDutyCycle";
const char MOTION_MAGIC_VOLTAGE_NAME[] = "MotionMagicVoltage";
const char MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME[] = "MotionMagicTorqueCurrentFOC";
const char MOTION_MAGIC_EXPO_DUTY_CYCLE_NAME[] = "MotionMagicExpoDutyCycle";
const char MOTION_MAGIC_EXPO_VOLTAGE_NAME[] = "MotionMagicExpoVoltage";
const char MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC_NAME[] = "MotionMagicExpoTorqueCurrentFOC";
const char MOTION_MAGIC_VELOCITY_DUTY_CYCLE_NAME[] = "MotionMagicVelocityDutyCycle";
const char MOTION_MAGIC_VELOCITY_VOLTAGE_NAME[] = "MotionMagicVelocityVoltage";
const char MOTION_MAGIC_VELOCITY_TORQUE_CURRENT_FOC_NAME[] = "MotionMagicVelocityTorqueCurrentFOC";
const char DYNAMIC_MOTION_MAGIC_DUTY_CYCLE_NAME[] = "DynamicMotionMagicDutyCycle";
const char DYNAMIC_MOTION_MAGIC_VOLTAGE_NAME[] = "DynamicMotionMagicVoltage";
const char DYNAMIC_MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME[] = "DynamicMotionMagicTorqueCurrentFOC";

template <typename T>
static bool readIntoScalar(const ros::NodeHandle &n, const char *name, std::atomic<T> &scalar)
{
    if (T val; n.getParam(name, val))
    {
        scalar = val;
        return true;
    }
    return false;
}

// Read a double named <param_type> from the array/map
// in params
bool findFloatParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, std::atomic<double> &val)
{
    if (!params.hasMember(param_name))
    {
        return false;
    }
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
    {
        throw std::runtime_error(param_name + " was not a valid double type");
    }
    if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        val.store(static_cast<double>(param));
        return true;
    }
    else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        val.store(static_cast<int>(param));
        return true;
    }
    else
    {
        throw std::runtime_error("A non-double value was passed for " + param_name);
    }

    return false;
}

// Read an integer named <param_name> from the array/map
// in params
bool findIntParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, std::atomic<int> &val)
{
    if (!params.hasMember(param_name))
    {
        return false;
    }
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
    {
        throw std::runtime_error(param_name + " was not a valid int type");
    }
    if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        val.store(static_cast<int>(param));
    }
    else
    {
        throw std::runtime_error("A non-int value was passed for " + param_name);
    }
    return false;
}

bool stringToLimitType(const std::string &str,
                       hardware_interface::talonfxpro::LimitType &limit_type) 
{
    if (str == "NormallyOpen")
        limit_type = hardware_interface::talonfxpro::LimitType::NormallyOpen;
    else if (str == "NormallyClosed")
        limit_type = hardware_interface::talonfxpro::LimitType::NormallyClosed;
    else
    {
        ROS_ERROR_STREAM("Invalid limit switch type: " << str);
        return false;
    }
    return true;
}

bool stringToLimitSource(const std::string &str,
                       hardware_interface::talonfxpro::LimitSource &limit_source)
{
    if (str == "LimitSwitchPin")
        limit_source = hardware_interface::talonfxpro::LimitSource::LimitSwitchPin;
    else if (str == "RemoteTalonFX")
        limit_source = hardware_interface::talonfxpro::LimitSource::RemoteTalonFX;
    else if (str == "RemoteCANifier")
        limit_source = hardware_interface::talonfxpro::LimitSource::RemoteCANifier;
    else if (str == "RemoteCANcoder")
        limit_source = hardware_interface::talonfxpro::LimitSource::RemoteCANcoder;
    else if (str == "Disabled")
        limit_source = hardware_interface::talonfxpro::LimitSource::Disabled;
    else
    {
        ROS_ERROR_STREAM("Invalid limit switch type: " << str << ", expecting LimitSwitchPin, RemoteTalonFX, RemoteCANifier, RemoteCANcoder, or Disabled");
        return false;
    }
    return true;
}

TalonFXProCIParams::TalonFXProCIParams(TalonFXProCIParams &&other) noexcept 
{ 
    for (size_t i = 0; i < hardware_interface::talonfxpro::TALON_PIDF_SLOTS; i++)
    {
        kP_[i].exchange(other.kP_[i]);
        kI_[i].exchange(other.kI_[i]);
        kD_[i].exchange(other.kD_[i]);
        kS_[i].exchange(other.kS_[i]);
        kV_[i].exchange(other.kV_[i]);
        kA_[i].exchange(other.kA_[i]);
        kG_[i].exchange(other.kG_[i]);
        gravity_type_[i].exchange(other.gravity_type_[i]);
    }

    invert_.exchange(other.invert_);
    neutral_mode_.exchange(other.neutral_mode_);

    duty_cycle_neutral_deadband_.exchange(other.duty_cycle_neutral_deadband_);
    peak_forward_duty_cycle_.exchange(other.peak_forward_duty_cycle_);
    peak_reverse_duty_cycle_.exchange(other.peak_reverse_duty_cycle_);

    stator_current_limit_.exchange(other.stator_current_limit_);
    stator_current_limit_enable_.exchange(other.stator_current_limit_enable_);

    supply_current_limit_.exchange(other.supply_current_limit_);
    supply_current_limit_enable_.exchange(other.supply_current_limit_enable_);
    supply_current_threshold_.exchange(other.supply_current_threshold_);
    supply_time_threshold_.exchange(other.supply_time_threshold_);

    supply_voltage_time_constant_.exchange(other.supply_voltage_time_constant_);
    peak_forward_voltage_.exchange(other.peak_forward_voltage_);
    peak_reverse_voltage_.exchange(other.peak_reverse_voltage_);

    peak_forward_torque_current_.exchange(other.peak_forward_torque_current_);
    peak_reverse_torque_current_.exchange(other.peak_reverse_torque_current_);
    torque_neutral_deadband_.exchange(other.torque_neutral_deadband_);

    feedback_rotor_offset_.exchange(other.feedback_rotor_offset_);
    sensor_to_mechanism_ratio_.exchange(other.sensor_to_mechanism_ratio_);
    rotor_to_sensor_ratio_.exchange(other.rotor_to_sensor_ratio_);
    feedback_sensor_source_.exchange(other.feedback_sensor_source_);
    feedback_remote_sensor_id_.exchange(other.feedback_remote_sensor_id_);

    differential_sensor_source_.exchange(other.differential_sensor_source_);
    differential_talonfx_sensor_id_.exchange(other.differential_talonfx_sensor_id_);
    differential_remote_sensor_id_.exchange(other.differential_remote_sensor_id_);

    peak_differential_duty_cycle_.exchange(other.peak_differential_duty_cycle_);
    peak_differential_voltage_.exchange(other.peak_differential_voltage_);
    peak_differential_torque_current_.exchange(other.peak_differential_torque_current_);

    duty_cycle_open_loop_ramp_period_.exchange(other.duty_cycle_open_loop_ramp_period_);
    voltage_open_loop_ramp_period_.exchange(other.voltage_open_loop_ramp_period_);
    torque_open_loop_ramp_period_.exchange(other.torque_open_loop_ramp_period_);

    duty_cycle_closed_loop_ramp_period_.exchange(other.duty_cycle_closed_loop_ramp_period_);
    voltage_closed_loop_ramp_period_.exchange(other.voltage_closed_loop_ramp_period_);
    torque_closed_loop_ramp_period_.exchange(other.torque_closed_loop_ramp_period_);

    forward_limit_type_.exchange(other.forward_limit_type_);
    forward_limit_autoset_position_enable_.exchange(other.forward_limit_autoset_position_enable_);
    forward_limit_autoset_position_value_.exchange(other.forward_limit_autoset_position_value_);
    forward_limit_enable_.exchange(other.forward_limit_enable_);
    forward_limit_source_.exchange(other.forward_limit_source_);
    forward_limit_remote_sensor_id_.exchange(other.forward_limit_remote_sensor_id_);

    reverse_limit_type_.exchange(other.reverse_limit_type_);
    reverse_limit_autoset_position_enable_.exchange(other.reverse_limit_autoset_position_enable_);
    reverse_limit_autoset_position_value_.exchange(other.reverse_limit_autoset_position_value_);
    reverse_limit_enable_.exchange(other.reverse_limit_enable_);
    reverse_limit_source_.exchange(other.reverse_limit_source_);
    reverse_limit_remote_sensor_id_.exchange(other.reverse_limit_remote_sensor_id_);

    beep_on_boot_.exchange(other.beep_on_boot_);
    beep_on_config_.exchange(other.beep_on_config_);
    allow_music_dur_disable_.exchange(other.allow_music_dur_disable_);

    softlimit_forward_enable_.exchange(other.softlimit_forward_enable_);
    softlimit_reverse_enable_.exchange(other.softlimit_reverse_enable_);
    softlimit_forward_threshold_.exchange(other.softlimit_forward_threshold_);
    softlimit_reverse_threshold_.exchange(other.softlimit_reverse_threshold_);

    motion_magic_cruise_velocity_.exchange(other.motion_magic_cruise_velocity_);
    motion_magic_acceleration_.exchange(other.motion_magic_acceleration_);
    motion_magic_jerk_.exchange(other.motion_magic_jerk_);
    motion_magic_expo_kV_.exchange(other.motion_magic_expo_kV_);
    motion_magic_expo_kA_.exchange(other.motion_magic_expo_kA_);

    control_enable_foc_.exchange(other.control_enable_foc_);
    control_override_brake_dur_neutral_.exchange(other.control_override_brake_dur_neutral_);
    control_max_abs_duty_cycle_.exchange(other.control_max_abs_duty_cycle_);
    control_deadband_.exchange(other.control_deadband_);
    control_feedforward_.exchange(other.control_feedforward_);
    control_slot_.exchange(other.control_slot_);
    control_oppose_master_direction_.exchange(other.control_oppose_master_direction_);
    //does not appear to exist in the params struct but keeping here anyway
    //control_limit_forward_motion_.exchange(other.control_limit_forward_motion_);
    //control_limit_reverse_motion_.exchange(other.control_limit_reverse_motion_);
    control_differential_slot_.exchange(other.control_differential_slot_);

    continuous_wrap_.exchange(other.continuous_wrap_);
    enable_read_thread_.exchange(other.enable_read_thread_);

    set_position_.exchange(other.set_position_);
    type_string_control_mode_.exchange(other.type_string_control_mode_);
} 

TalonFXProCIParams& TalonFXProCIParams::operator=(const TalonFXProCIParams &other)
{
    if (this != &other)
    {
        joint_name_ = other.joint_name_;

        // Slot0Configs ... Slot2Configs
        for (size_t i = 0; i < hardware_interface::talonfxpro::TALON_PIDF_SLOTS; i++)
        {
            kP_[i].store(other.kP_[i].load());
            kI_[i].store(other.kI_[i].load());
            kD_[i].store(other.kD_[i].load());
            kS_[i].store(other.kS_[i].load());
            kV_[i].store(other.kV_[i].load());
            kA_[i].store(other.kA_[i].load());
            kG_[i].store(other.kG_[i].load());
            gravity_type_[i].store(other.gravity_type_[i].load());
        }
        invert_.store(other.invert_.load());
        neutral_mode_.store(other.neutral_mode_.load());

        duty_cycle_neutral_deadband_.store(other.duty_cycle_neutral_deadband_.load());
        peak_forward_duty_cycle_.store(other.peak_forward_duty_cycle_.load());
        peak_reverse_duty_cycle_.store(other.peak_reverse_duty_cycle_.load());

        stator_current_limit_.store(other.stator_current_limit_.load());
        stator_current_limit_enable_.store(other.stator_current_limit_enable_.load());

        supply_current_limit_.store(other.supply_current_limit_.load());
        supply_current_limit_enable_.store(other.supply_current_limit_enable_.load());
        supply_current_threshold_.store(other.supply_current_threshold_.load());
        supply_time_threshold_.store(other.supply_time_threshold_.load());

        supply_voltage_time_constant_.store(other.supply_voltage_time_constant_.load());
        peak_forward_voltage_.store(other.peak_forward_voltage_.load());
        peak_reverse_voltage_.store(other.peak_reverse_voltage_.load());

        peak_forward_torque_current_.store(other.peak_forward_torque_current_.load());
        peak_reverse_torque_current_.store(other.peak_reverse_torque_current_.load());
        torque_neutral_deadband_.store(other.torque_neutral_deadband_.load());

        feedback_rotor_offset_.store(other.feedback_rotor_offset_.load());
        sensor_to_mechanism_ratio_.store(other.sensor_to_mechanism_ratio_.load());
        rotor_to_sensor_ratio_.store(other.rotor_to_sensor_ratio_.load());
        feedback_sensor_source_.store(other.feedback_sensor_source_.load());
        feedback_remote_sensor_id_.store(other.feedback_remote_sensor_id_.load());

        differential_sensor_source_.store(other.differential_sensor_source_.load());
        differential_talonfx_sensor_id_.store(differential_talonfx_sensor_id_.load());
        differential_remote_sensor_id_.store(differential_remote_sensor_id_.load());

        peak_differential_duty_cycle_.store(other.peak_differential_duty_cycle_.load());
        peak_differential_voltage_.store(other.peak_differential_voltage_.load());
        peak_differential_torque_current_.store(other.peak_differential_torque_current_.load());

        duty_cycle_open_loop_ramp_period_.store(other.duty_cycle_open_loop_ramp_period_.load());
        voltage_open_loop_ramp_period_.store(other.voltage_open_loop_ramp_period_.load());
        torque_open_loop_ramp_period_.store(other.torque_open_loop_ramp_period_.load());

        duty_cycle_closed_loop_ramp_period_.store(other.duty_cycle_closed_loop_ramp_period_.load());
        voltage_closed_loop_ramp_period_.store(other.voltage_closed_loop_ramp_period_.load());
        torque_closed_loop_ramp_period_.store(other.torque_closed_loop_ramp_period_.load());

        forward_limit_type_.store(other.forward_limit_type_.load());
        forward_limit_autoset_position_enable_.store(other.forward_limit_autoset_position_enable_.load());
        forward_limit_autoset_position_value_.store(other.forward_limit_autoset_position_value_.load());
        forward_limit_enable_.store(other.forward_limit_enable_.load());
        forward_limit_source_.store(other.forward_limit_source_.load());
        forward_limit_remote_sensor_id_.store(other.forward_limit_remote_sensor_id_.load());

        reverse_limit_type_.store(other.reverse_limit_type_.load());
        reverse_limit_autoset_position_enable_.store(other.reverse_limit_autoset_position_enable_.load());
        reverse_limit_autoset_position_value_.store(other.reverse_limit_autoset_position_value_.load());
        reverse_limit_enable_.store(other.reverse_limit_enable_.load());
        reverse_limit_source_.store(other.reverse_limit_source_.load());
        reverse_limit_remote_sensor_id_.store(other.reverse_limit_remote_sensor_id_.load());

        beep_on_boot_.store(other.beep_on_boot_.load());
        beep_on_config_.store(other.beep_on_config_.load());
        allow_music_dur_disable_.store(other.allow_music_dur_disable_.load());

        softlimit_forward_enable_.store(other.softlimit_forward_enable_.load());
        softlimit_reverse_enable_.store(other.softlimit_reverse_enable_.load());
        softlimit_forward_threshold_.store(other.softlimit_forward_threshold_.load());
        softlimit_reverse_threshold_.store(other.softlimit_reverse_threshold_.load());

        motion_magic_cruise_velocity_.store(other.motion_magic_cruise_velocity_.load());
        motion_magic_acceleration_.store(other.motion_magic_acceleration_.load());
        motion_magic_jerk_.store(other.motion_magic_jerk_.load());
        motion_magic_expo_kV_.store(other.motion_magic_expo_kV_.load());
        motion_magic_expo_kA_.store(other.motion_magic_expo_kA_.load());

        control_enable_foc_.store(other.control_enable_foc_.load());
        control_override_brake_dur_neutral_.store(other.control_override_brake_dur_neutral_.load());
        control_max_abs_duty_cycle_.store(other.control_max_abs_duty_cycle_.load());
        control_deadband_.store(other.control_deadband_.load());
        control_feedforward_.store(other.control_feedforward_.load());
        control_slot_.store(other.control_slot_.load());
        control_oppose_master_direction_.store(other.control_oppose_master_direction_.load());
        control_limit_forward_motion_.store(other.control_limit_forward_motion_.load());
        control_limit_reverse_motion_.store(other.control_limit_reverse_motion_.load());
        control_differential_slot_.store(other.control_differential_slot_.load());

        continuous_wrap_.store(other.continuous_wrap_.load());
        enable_read_thread_.store(other.enable_read_thread_.load());
        set_position_.store(other.set_position_.load()); // TODO - this might not matter since it isn't a persistent config item?
        type_string_control_mode_.store(other.type_string_control_mode_.load());
    }
    return *this;
}

// Read a joint name from the given nodehandle's params
// This needs to be present for the code to work, so 
// return true / false for success / failure
bool TalonFXProCIParams::readJointName(const ros::NodeHandle &n)
{
    if (!n.getParam("joint", joint_name_))
    {
        ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
        return false;
    }
    return true;
}
bool TalonFXProCIParams::readCloseLoopParams(const ros::NodeHandle &n)
{
    XmlRpc::XmlRpcValue pid_param_list;

    if (!n.getParam("close_loop_values", pid_param_list))
    {
        return true;
    }
    if (pid_param_list.size() <= static_cast<int>(hardware_interface::talonfxpro::TALON_PIDF_SLOTS))
    {
        for (int i = 0; i < pid_param_list.size(); i++)
        {
            XmlRpc::XmlRpcValue &pidparams = pid_param_list[i];

#ifdef TALONCI_BACKWARDS_COMPATIBILITY
            // Read old versions of these first
            findFloatParam("p", pidparams, kP_[i]);
            findFloatParam("i", pidparams, kI_[i]);
            findFloatParam("d", pidparams, kD_[i]);
            findFloatParam("f", pidparams, kV_[i]);
#endif

            // Then the new ones. Hopefully we won't see both
            // in any particular config, but this setup will
            // favor using the new names over the old ones
            findFloatParam("kP", pidparams, kP_[i]);
            findFloatParam("kI", pidparams, kI_[i]);
            findFloatParam("kD", pidparams, kD_[i]);
            findFloatParam("kS", pidparams, kS_[i]);
            findFloatParam("kV", pidparams, kV_[i]);
            findFloatParam("kA", pidparams, kA_[i]);
            findFloatParam("kG", pidparams, kG_[i]);
            if (std::string str; n.getParam("gravity_type", str))
            {
                if (str == "elevator_static")
                {
                    gravity_type_[i] = hardware_interface::talonfxpro::GravityType::Elevator_Static;
                }
                else if (str == "arm_cosine")
                {
                    gravity_type_[i] = hardware_interface::talonfxpro::GravityType::Arm_Cosine;
                }
                else
                {
                    ROS_ERROR_STREAM("Invalid gravity_type for joint " << joint_name_ << " closed loop values entry " << i << " : " << str);
                    return false;
                }
            }
        }
        return true;
    }
    else
    {
        throw std::runtime_error("Too many pid_param values");
    }
    return false;
}

bool TalonFXProCIParams::readInvert(const ros::NodeHandle &n)
{
    if (std::string str; n.getParam("invert", str))
    {
        if (str == "counterclockwise_positive")
        {
            invert_ = hardware_interface::talonfxpro::Inverted::CounterClockwise_Positive;
        }
        else if (str == "clockwise_positive")
        {
            invert_ = hardware_interface::talonfxpro::Inverted::Clockwise_Positive;
        }
        else
        {
            ROS_ERROR_STREAM("Invalid option for invert in joint " << joint_name_ << " : options are 'counterclockwise_positive' and 'clockwise_positive'");
            return false;
        }
    }
    return true;
}

bool TalonFXProCIParams::readNeutralMode(const ros::NodeHandle &n)
{
    if (std::string mode_string; n.getParam("neutral_mode", mode_string))
    {
        if (mode_string == "Coast")
            neutral_mode_ = hardware_interface::talonfxpro::NeutralMode::Coast;
        else if (mode_string == "Brake")
            neutral_mode_ = hardware_interface::talonfxpro::NeutralMode::Brake;
        else
        {
            ROS_ERROR_STREAM("Invalid neutral mode " << mode_string << 
                            " namespace: " << n.getNamespace() <<
                            " joint_name " << joint_name_);
            return false;
        }
    }
    return true;
}

bool TalonFXProCIParams::readDutyCycleOutputShaping(const ros::NodeHandle &n)
{
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    readIntoScalar(n, "peak_output_forward", peak_forward_duty_cycle_);
    readIntoScalar(n, "peak_output_reverse", peak_reverse_duty_cycle_);
    readIntoScalar(n, "neutral_deadband", duty_cycle_neutral_deadband_);
#endif
    readIntoScalar(n, "peak_forward_duty_cycle", peak_forward_duty_cycle_);
    readIntoScalar(n, "peak_reverse_duty_cycle", peak_reverse_duty_cycle_);
    readIntoScalar(n, "duty_cycle_neutral_deadband", duty_cycle_neutral_deadband_);
    return true;
}

bool TalonFXProCIParams::readSupplyCurrentLimits(const ros::NodeHandle &n)
{
    const bool current_limit_read = readIntoScalar(n, "supply_current_limit", supply_current_limit_);
    if (readIntoScalar(n, "supply_current_limit_enable", supply_current_limit_enable_) &&
        supply_current_limit_enable_ && !current_limit_read)
    {
        ROS_WARN_STREAM("Supply current limit not set for " << joint_name_
                        << " before enabling - using defaults might not work as expected");
    }
    readIntoScalar(n, "supply_current_threshold", supply_current_threshold_);
    readIntoScalar(n, "supply_time_threshold", supply_time_threshold_);
    return true;
}

bool TalonFXProCIParams::readStatorCurrentLimits(const ros::NodeHandle &n)
{
    const bool current_limit_read = readIntoScalar(n, "stator_current_limit", stator_current_limit_);
    if (readIntoScalar(n, "stator_current_limit_enable", stator_current_limit_enable_) &&
        stator_current_limit_enable_ && !current_limit_read)
    {
        ROS_WARN_STREAM("Stator current limit not set for " << joint_name_
                        << " before enabling - using defaults might not work as expected");
    }
    return true;
}

bool TalonFXProCIParams::readVoltageLimits(const ros::NodeHandle &n)
{
    // Perhaps map to voltage compensation value?
    readIntoScalar(n, "supply_voltage_time_constant", supply_voltage_time_constant_);
    readIntoScalar(n, "peak_forward_voltage", peak_forward_voltage_);
    readIntoScalar(n, "peak_reverse_voltage", peak_reverse_voltage_);
    return true;
}

bool TalonFXProCIParams::readTorqueLimits(const ros::NodeHandle &n)
{
    readIntoScalar(n, "peak_forward_torque_current", peak_forward_torque_current_);
    readIntoScalar(n, "peak_reverse_torque_current", peak_reverse_torque_current_);
    readIntoScalar(n, "torque_neutral_deadband", torque_neutral_deadband_);
    return true;
}

bool TalonFXProCIParams::readFeedback(const ros::NodeHandle &n)
{
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    double conversion_factor{};
    if (n.getParam("conversion_factor", conversion_factor))
    {
        sensor_to_mechanism_ratio_ = 1.0 / conversion_factor;
    }
#endif
    readIntoScalar(n, "feedback_rotor_offset", feedback_rotor_offset_);
    readIntoScalar(n, "sensor_to_mechanism_ratio", sensor_to_mechanism_ratio_);
    readIntoScalar(n, "rotor_to_sensor_ratio", rotor_to_sensor_ratio_);

    std::string str;
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    if (n.getParam("remote_feedback_type", str))
    {
        if (str == "RemoteSensor0")
        {
            feedback_sensor_source_ = hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANCoder;
        }
    }
    readIntoScalar(n, "remote_feedback_device_id0", feedback_remote_sensor_id_);
#endif
    if (n.getParam("feedback_sensor_source", str))
    {
        if (str == "RotorSensor")
        {
            feedback_sensor_source_ = hardware_interface::talonfxpro::FeedbackSensorSource::RotorSensor;
        }
        else if (str == "RemoteCANCoder")
        {
            feedback_sensor_source_ = hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANCoder;
        }
        else if (str == "FusedCANcoder")
        {
            feedback_sensor_source_ = hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder;
        }
        else
        {
            ROS_ERROR_STREAM("Invalid remote feedback device name " << str << " for joint " << joint_name_);
            return false;
        }
    }
    readIntoScalar(n, "feedback_remote_sensor_id", feedback_remote_sensor_id_);

    return true;
}

bool TalonFXProCIParams::readDifferentialConfig(const ros::NodeHandle &n)
{
    std::string str;
    if (n.getParam("differential_sensor_source", str))
    {
        if (str == "disabled")
        {
            differential_sensor_source_.store(hardware_interface::talonfxpro::DifferentialSensorSource::Disabled);
        }
        else if (str == "remotetalonfx_diff")
        {
            differential_sensor_source_.store(hardware_interface::talonfxpro::DifferentialSensorSource::RemoteTalonFX_Diff);
        }
        else if (str == "remotepigeon2_yaw")
        {
            differential_sensor_source_.store(hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Yaw);
        }
        else if (str == "remotepigeon2_pitch")
        {
            differential_sensor_source_.store(hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Pitch);
        }
        else if (str == "remotepigeon2_roll")
        {
            differential_sensor_source_.store(hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Roll);
        }
        else if (str == "remotecancode")
        {
            differential_sensor_source_.store(hardware_interface::talonfxpro::DifferentialSensorSource::RemoteCANcoder);
        }
        else
        {

            ROS_ERROR_STREAM("Invalid differential sensor source " << str << " for joint " << joint_name_);
        }
    }
    readIntoScalar(n, "differential_talonfx_sensor_id_", differential_talonfx_sensor_id_);
    readIntoScalar(n, "differential_remote_sensor_id_", differential_remote_sensor_id_);
    return true;
}

bool TalonFXProCIParams::readDifferentialConstants(const ros::NodeHandle &n)
{
    readIntoScalar(n, "peak_differential_duty_cycle", peak_differential_duty_cycle_);
    readIntoScalar(n, "peak_differential_voltage", peak_differential_voltage_);
    readIntoScalar(n, "peak_differential_torque_current", peak_differential_torque_current_);
    return true;
}

bool TalonFXProCIParams::readOpenLoopRamps(const ros::NodeHandle &n)
{
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    readIntoScalar(n, "open_loop_ramp", duty_cycle_open_loop_ramp_period_);
    readIntoScalar(n, "open_loop_ramp", voltage_open_loop_ramp_period_);
    readIntoScalar(n, "open_loop_ramp", torque_open_loop_ramp_period_);
#endif
    readIntoScalar(n, "duty_cycle_open_loop_ramp_period", duty_cycle_open_loop_ramp_period_);
    readIntoScalar(n, "voltage_open_loop_ramp_period", voltage_open_loop_ramp_period_);
    readIntoScalar(n, "torque_open_loop_ramp_period", torque_open_loop_ramp_period_);
    return true;
}

bool TalonFXProCIParams::readClosedLoopRamps(const ros::NodeHandle &n)
{
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    readIntoScalar(n, "closed_loop_ramp", duty_cycle_closed_loop_ramp_period_);
    readIntoScalar(n, "closed_loop_ramp", voltage_closed_loop_ramp_period_);
    readIntoScalar(n, "closed_loop_ramp", torque_closed_loop_ramp_period_);
#endif
    readIntoScalar(n, "duty_cycle_closed_loop_ramp_period", duty_cycle_closed_loop_ramp_period_);
    readIntoScalar(n, "voltage_closed_loop_ramp_period", voltage_closed_loop_ramp_period_);
    readIntoScalar(n, "torque_closed_loop_ramp_period", torque_closed_loop_ramp_period_);
    return true;
}

bool TalonFXProCIParams::readLimitSwitches(const ros::NodeHandle &n)
{
    // For now, looks like source is just local?
    std::string str;
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    if (n.getParam("limit_switch_local_forward_normal", str))
    {
        hardware_interface::talonfxpro::LimitType forward_limit_type;
        if (!stringToLimitType(str, forward_limit_type))
        {
            return false;
        }
        forward_limit_type_ = forward_limit_type;
    }
#endif
    if (n.getParam("forward_limit_switch_type", str))
    {
        hardware_interface::talonfxpro::LimitType forward_limit_type;
        if (!stringToLimitType(str, forward_limit_type))
        {
            return false;
        }
        forward_limit_type_ = forward_limit_type;
    }
    readIntoScalar(n, "forward_limit_autoset_position_enable", forward_limit_autoset_position_enable_);
    readIntoScalar(n, "forward_limit_autoset_position_value", forward_limit_autoset_position_value_);
    readIntoScalar(n, "forward_limit_enable", forward_limit_enable_);
    readIntoScalar(n, "forward_limit_remote_sensor_id_", forward_limit_remote_sensor_id_);
    if (n.getParam("forward_limit_switch_source", str))
    {
        hardware_interface::talonfxpro::LimitSource forward_limit_source;
        if (!stringToLimitSource(str, forward_limit_source))
        {
            return false;
        }
        forward_limit_source_ = forward_limit_source;
    }

#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    if (n.getParam("limit_switch_local_reverse_normal", str))
    {
        hardware_interface::talonfxpro::LimitType reverse_limit_type;
        if (!stringToLimitType(str, reverse_limit_type))
        {
            return false;
        }
        reverse_limit_type_ = reverse_limit_type;
    }
#endif
    if (n.getParam("reverse_limit_switch_type", str))
    {
        hardware_interface::talonfxpro::LimitType reverse_limit_type;
        if (!stringToLimitType(str, reverse_limit_type))
        {
            return false;
        }
        reverse_limit_type_ = reverse_limit_type;
    }
    readIntoScalar(n, "reverse_limit_autoset_position_enable", reverse_limit_autoset_position_enable_);
    readIntoScalar(n, "reverse_limit_autoset_position_value", reverse_limit_autoset_position_value_);
    readIntoScalar(n, "reverse_limit_enable", reverse_limit_enable_);
    readIntoScalar(n, "reverse_limit_remote_sensor_id_", reverse_limit_remote_sensor_id_);
    if (n.getParam("reverse_limit_switch_source", str))
    {
        hardware_interface::talonfxpro::LimitSource reverse_limit_source;
        if (!stringToLimitSource(str, reverse_limit_source))
        {
            return false;
        }
        reverse_limit_source_ = reverse_limit_source;
    }

    return true;
}

bool TalonFXProCIParams::readAudio(const ros::NodeHandle &n)
{
    readIntoScalar(n, "beep_on_boot", beep_on_boot_);
    readIntoScalar(n, "beep_on_config", beep_on_config_);
    readIntoScalar(n, "allow_music_dur_disable", allow_music_dur_disable_);
    return true;
}

bool TalonFXProCIParams::readSoftLimits(const ros::NodeHandle &n)
{
    const bool forward_threshold_set = readIntoScalar(n, "softlimit_forward_threshold", softlimit_forward_threshold_);
    if (readIntoScalar(n, "softlimit_forward_enable", softlimit_forward_enable_) &&
        softlimit_forward_enable_ && !forward_threshold_set)
    {
        ROS_WARN_STREAM("Enabling forward softlimits for " << joint_name_ << " without setting threshold");
    }
    const bool reverse_threshold_set = readIntoScalar(n, "softlimit_reverse_threshold", softlimit_reverse_threshold_);
    if (readIntoScalar(n, "softlimit_reverse_enable", softlimit_reverse_enable_) &&
        softlimit_reverse_enable_ && !reverse_threshold_set)
    {
        ROS_WARN_STREAM("Enabling reverse softlimits for " << joint_name_ << " without setting threshold");
    }
    return true;
}

bool TalonFXProCIParams::readMotionMagic(const ros::NodeHandle &n)
{
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    readIntoScalar(n, "motion_cruise_velocity", motion_magic_cruise_velocity_);
    readIntoScalar(n, "motion_acceleration", motion_magic_acceleration_);
#endif
    readIntoScalar(n, "motion_magic_cruise_velocity", motion_magic_cruise_velocity_);
    readIntoScalar(n, "motion_magic_acceleration", motion_magic_acceleration_);
    readIntoScalar(n, "motion_magic_jerk", motion_magic_jerk_);
    readIntoScalar(n, "motion_magic_expo_kV", motion_magic_expo_kV_);
    readIntoScalar(n, "motion_magic_expo_kA", motion_magic_expo_kA_);
    return true;
}
bool TalonFXProCIParams::readControl(const ros::NodeHandle &n)
{
#ifdef TALONCI_BACKWARDS_COMPATIBILITY
// TODO : maybe demand1 value into feedforward?
    readIntoScalar(n, "neutral_deadband", control_deadband_);
#endif
    readIntoScalar(n, "control_enable_foc", control_enable_foc_);
    readIntoScalar(n, "control_override_brake_dur_neutral", control_override_brake_dur_neutral_);
    readIntoScalar(n, "control_max_abs_duty_cycle", control_max_abs_duty_cycle_);
    readIntoScalar(n, "control_deadband", control_deadband_);
    readIntoScalar(n, "control_feedforward", control_feedforward_);
    readIntoScalar(n, "control_slot", control_slot_);
    readIntoScalar(n, "control_oppose_master_direction", control_oppose_master_direction_);
    readIntoScalar(n, "control_limit_forward_motion", control_limit_forward_motion_);
    readIntoScalar(n, "control_limit_reverse_motion", control_limit_reverse_motion_);
    readIntoScalar(n, "control_differential_slot", control_differential_slot_);
    return true;
}
bool TalonFXProCIParams::readContinuousWrap(const ros::NodeHandle &n)
{
    readIntoScalar(n, "continuous_wrap", continuous_wrap_);
    return true;
}

bool TalonFXProCIParams::readEnableReadThread(const ros::NodeHandle &n)
{
    readIntoScalar(n, "enable_read_thread", enable_read_thread_);
    return true;
}

bool TalonFXProCIParams::readTypeStringControlMode(const ros::NodeHandle &n)
{
    if (std::string type_str; n.getParam("type", type_str))
    {
        // Controller set to base class - no default type
        if (type_str == "talonfxpro_controllers/TalonFXProControllerInterface")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::First;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDutyCycleOutController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DutyCycleOut;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProTorqueCurrentFOCController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProVoltageOutController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::VoltageOut;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProPositionDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::PositionDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProPositionVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::PositionVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProPositionTorqueCurrentFOCController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProVelocityDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProVelocityVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::VelocityVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProVelocityTorqueCurrentFOCController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicTorqueCurrentFOCController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicExpoDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicExpoDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicExpoVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicExpoTorqueCurrentFOCController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicExpoTorqueCurrentFOC;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicVelocityDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicVelocityVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProMotionMagicVelocityTorqueCurrentFOCController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDynamicMotionMagicDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDynamicMotionMagicVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDynamicMotionMagicTorqueCurrentFOCController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProFollowerController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::Follower;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProStrictFollowerController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::StrictFollower;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProNeutralOutController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::NeutralOut;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProCoastOutController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::CoastOut;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProStaticBrakeController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::StaticBrake;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialDutyCycleOutController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialDutyCycleOut;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialVoltageOutController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialVoltageOut;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialPositionDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialPositionDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialPositionVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialPositionVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialVelocityDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialVelocityDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialVelocityVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialVelocityVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialMotionMagicDutyCycleController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialMotionMagicDutyCycle;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialMotionMagicVoltageController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialMotionMagicVoltage;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialFollowerController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialFollower;
            return true;
        }
        if (type_str == "talonfxpro_controllers/TalonFXProDifferentialStrictFollowerController")
        {
            type_string_control_mode_ = hardware_interface::talonfxpro::TalonMode::DifferentialStrictFollower;
            return true;
        }
        ROS_ERROR_STREAM("Unknown control mode " << type_str << " for joint " << joint_name_);
        return false;
    }
    return true;
}

const std::string& TalonFXProCIParams::getJointName(void) const { return joint_name_;}

TalonFXProControllerInterface::TalonFXProControllerInterface() = default;

// Read params from config file and use them to
// initialize the Talon hardware
bool TalonFXProControllerInterface::initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                                                 hardware_interface::talonfxpro::TalonFXProStateInterface * /*tsi*/,
                                                 ros::NodeHandle &n)
{
    return init(tci, n, talon_, false) &&
            setInitialControlMode();
}

// Same as above, except pass in an array
// of node handles. First entry is set up as the leader
// talon and the rest are set in follower mode to follow
// the leader
bool TalonFXProControllerInterface::initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                                                 hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                                                 std::vector<ros::NodeHandle> &n)
{
    // Initialize the first talon normally
    if (!initWithNode(tci, tsi, n[0]))
    {
        return false;
    }

    const int follow_can_id = talon_.state()->getCANID();

    // If more than 1 joint ise passed in, everything
    // but the first is to be set up as a follower
    follower_talons_.resize(n.size() - 1);
    for (size_t i = 1; i < n.size(); i++)
    {
        ROS_INFO_STREAM("follower n[" << i << "] = " << n[i].getNamespace());
        if (!init(tci, n[i], follower_talons_[i-1], true))
        {
            return false;
        }
        ROS_WARN_STREAM("Setting follower control mode");
        follower_talons_[i-1]->setControlMode(hardware_interface::talonfxpro::TalonMode::Follower);
        follower_talons_[i-1]->setControlOutput(follow_can_id);
        ROS_INFO_STREAM("Set up talon FX pro " << follower_talons_[i-1].getName() << " to follow CAN ID " << follow_can_id << " (" << talon_.getName() << ")");
    }

    return true;
}

// Init with XmlRpcValue instead of NodeHandle - XmlRpcValue
// will be either a string or an array of strings of joints
// to load
bool TalonFXProControllerInterface::initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                                                 hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                                                 ros::NodeHandle &controller_nh,
                                                 XmlRpc::XmlRpcValue param)
{
    std::vector<ros::NodeHandle> joint_nodes;

    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        if (param.size() == 0)
        {
            ROS_ERROR_STREAM("Joint param is an empty list");
            return false;
        }

        for (int i = 0; i < param.size(); ++i)
        {
            if (param[i].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR_STREAM("Joint param #" << i << " isn't a string.");
                return false;
            }
        }

        for (int i = 0; i < param.size(); i++)
        {
            joint_nodes.emplace_back(controller_nh, static_cast<const std::string>(param[i]));
        }
    }
    else if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        joint_nodes.emplace_back(controller_nh, static_cast<std::string>(param));
    }
    else
    {
        ROS_ERROR_STREAM("Joint param is neither a list of strings nor a string.");
        return false;
    }

    return initWithNode(tci, tsi, joint_nodes);
}

bool TalonFXProControllerInterface::initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                                                 hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                                                 ros::NodeHandle &controller_nh,
                                                 const std::string &talonfxpro_name)
{
    return initWithNode(tci, tsi, controller_nh, XmlRpc::XmlRpcValue{talonfxpro_name});
}

bool TalonFXProControllerInterface::initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                                                 hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                                                 ros::NodeHandle &controller_nh,
                                                 const char *talonfxpro_name)
{
    return initWithNode(tci, tsi, controller_nh, XmlRpc::XmlRpcValue{talonfxpro_name});
}

bool TalonFXProControllerInterface::init(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                                         ros::NodeHandle &n,
                                         hardware_interface::talonfxpro::TalonFXProCommandHandle &talon,
                                         bool follower)
{
    ROS_WARN_STREAM(__PRETTY_FUNCTION__ << "init start");
    TalonFXProCIParams params;
    if (!readParams(n, params))
    {
        return false;
    }
    ROS_WARN_STREAM(__PRETTY_FUNCTION__ << "init past readParams for " << params.joint_name_);
    talon = tci->getHandle(params.getJointName());
    writeParamsToHW(params, talon);

    // Only allow dynamic reconfigure to be active for non-follower talons
    if (!follower)
    {
        // If this isn't a follower, meaning it is the leader, use
        // this namespace's params to initialize the TalonFXProCIParams
        // member var for the class. This hooks those config params up
        // to dynamic reconfigure and this class's set functions
        params_ = params;
        bool dynamic_reconfigure = false;
        n.param<bool>("dynamic_reconfigure", dynamic_reconfigure, dynamic_reconfigure);
        ddr_updater_ = std::make_unique<ddr_updater::DDRUpdater>(n);
        if (!dynamic_reconfigure)
        {
            ddr_updater_->shutdownDDRUpdater();
        }
        else
        {
            ddr_updater_->ddr_.registerVariable<double>("kP_0",
                                                        [this]() { return params_.kP_[0].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkP, this, _1, 0, false),
                                                        "Proportial PID constant for slot 0", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kI_0",
                                                        [this]() { return params_.kI_[0].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkI, this, _1, 0, false),
                                                        "Integral PID constant for slot 0", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kD_0",
                                                        [this]() { return params_.kD_[0].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkD, this, _1, 0, false),
                                                        "Derivative PID constant for slot 0", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kS_0",
                                                        [this]() { return params_.kS_[0].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkS, this, _1, 0, false),
                                                        "Static PID constant for slot 0", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kV_0",
                                                        [this]() { return params_.kV_[0].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkV, this, _1, 0, false),
                                                        "Velocity PID constant for slot 0", 
                                                        0, 2000);
            ddr_updater_->ddr_.registerVariable<double>("kA_0",
                                                        [this]() { return params_.kA_[0].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkA, this, _1, 0, false),
                                                        "Acceleration PID constant for slot 0", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kG_0",
                                                        [this]() { return params_.kG_[0].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkG, this, _1, 0, false),
                                                        "Gravity PID constant for slot 0", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerEnumVariable<int>("gravity_type_0",
                                                        [this]() { return static_cast<int>(params_.gravity_type_[0].load());},
                                                        [this](const int gravity_type_int) { this->setGravityType(static_cast<hardware_interface::talonfxpro::GravityType>(gravity_type_int), 0, false);},
                                                        "Gravity type for slot 0 kG term", 
                                                        std::map<std::string, int>{
                                                            {"Elevator Static", static_cast<int>(hardware_interface::talonfxpro::GravityType::Elevator_Static)},
                                                            {"Arm Cosine", static_cast<int>(hardware_interface::talonfxpro::GravityType::Arm_Cosine)}
                                                        }
                                                        );
            ddr_updater_->ddr_.registerVariable<double>("kP_1",
                                                        [this]() { return params_.kP_[1].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkP, this, _1, 1, false),
                                                        "Proportial PID constant for slot 1", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kI_1",
                                                        [this]() { return params_.kI_[1].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkI, this, _1, 1, false),
                                                        "Integral PID constant for slot 1", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kD_1",
                                                        [this]() { return params_.kD_[1].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkD, this, _1, 1, false),
                                                        "Derivative PID constant for slot 1", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kS_1",
                                                        [this]() { return params_.kS_[1].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkS, this, _1, 1, false),
                                                        "Static PID constant for slot 1", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kV_1",
                                                        [this]() { return params_.kV_[1].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkV, this, _1, 1, false),
                                                        "Velocity PID constant for slot 1", 
                                                        0, 2000);
            ddr_updater_->ddr_.registerVariable<double>("kA_1",
                                                        [this]() { return params_.kA_[1].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkA, this, _1, 1, false),
                                                        "Acceleration PID constant for slot 1", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kG_1",
                                                        [this]() { return params_.kG_[1].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkG, this, _1, 1, false),
                                                        "Gravity PID constant for slot 1", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerEnumVariable<int>("gravity_type_1",
                                                        [this]() { return static_cast<int>(params_.gravity_type_[1].load());},
                                                        [this](const int gravity_type_int) { this->setGravityType(static_cast<hardware_interface::talonfxpro::GravityType>(gravity_type_int), 1, false);},
                                                        "Gravity type for slot 1 kG term", 
                                                        std::map<std::string, int>{
                                                            {"Elevator Static", static_cast<int>(hardware_interface::talonfxpro::GravityType::Elevator_Static)},
                                                            {"Arm Cosine", static_cast<int>(hardware_interface::talonfxpro::GravityType::Arm_Cosine)}
                                                        }
                                                        );
            ddr_updater_->ddr_.registerVariable<double>("kP_2",
                                                        [this]() { return params_.kP_[2].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkP, this, _1, 2, false),
                                                        "Proportial PID constant for slot 2", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kI_2",
                                                        [this]() { return params_.kI_[2].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkI, this, _1, 2, false),
                                                        "Integral PID constant for slot 2", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kD_2",
                                                        [this]() { return params_.kD_[2].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkD, this, _1, 2, false),
                                                        "Derivative PID constant for slot 2", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kS_2",
                                                        [this]() { return params_.kS_[2].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkS, this, _1, 2, false),
                                                        "Static PID constant for slot 2", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kV_2",
                                                        [this]() { return params_.kV_[2].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkV, this, _1, 2, false),
                                                        "Velocity PID constant for slot 2", 
                                                        0, 2000);
            ddr_updater_->ddr_.registerVariable<double>("kA_2",
                                                        [this]() { return params_.kA_[2].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkA, this, _1, 2, false),
                                                        "Acceleration PID constant for slot 2", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerVariable<double>("kG_2",
                                                        [this]() { return params_.kG_[2].load();},
                                                        boost::bind(&TalonFXProControllerInterface::setkG, this, _1, 2, false),
                                                        "Gravity PID constant for slot 2", 
                                                        0, 1000);
            ddr_updater_->ddr_.registerEnumVariable<int>("gravity_type_2",
                                                        [this]() { return static_cast<int>(params_.gravity_type_[2].load());},
                                                        [this](const int gravity_type_int) { this->setGravityType(static_cast<hardware_interface::talonfxpro::GravityType>(gravity_type_int), 2, false);},
                                                        "Gravity type for slot 2 kG term", 
                                                        std::map<std::string, int>{
                                                            {"Elevator Static", static_cast<int>(hardware_interface::talonfxpro::GravityType::Elevator_Static)},
                                                            {"Arm Cosine", static_cast<int>(hardware_interface::talonfxpro::GravityType::Arm_Cosine)}
                                                        }
                                                        );
            ddr_updater_->ddr_.registerEnumVariable<int>("invert",
                                                        [this]() { return static_cast<int>(params_.invert_.load());},
                                                        [this](const int invert_int) { this->setInvert(static_cast<hardware_interface::talonfxpro::Inverted>(invert_int), false);},
                                                        "Motor invert direction",
                                                        std::map<std::string, int>{
                                                            {"CounterClockwise Positive", static_cast<int>(hardware_interface::talonfxpro::Inverted::CounterClockwise_Positive)},
                                                            {"Clockwise Positive", static_cast<int>(hardware_interface::talonfxpro::Inverted::Clockwise_Positive)}
                                                        }
                                                        );
            ddr_updater_->ddr_.registerEnumVariable<int>("neutral_mode",
                                                        [this]() { return static_cast<int>(params_.neutral_mode_.load());},
                                                        [this](const int neutral_mode_int) { this->setNeutralMode(static_cast<hardware_interface::talonfxpro::NeutralMode>(neutral_mode_int), false);},
                                                        "Motor neutral mode", 
                                                        std::map<std::string, int>{
                                                            {"Coast", static_cast<int>(hardware_interface::talonfxpro::NeutralMode::Coast)},
                                                            {"Brake", static_cast<int>(hardware_interface::talonfxpro::NeutralMode::Brake)}
                                                        }
                                                        );
            ddr_updater_->ddr_.registerVariable<double>("duty_cycle_netural_deadband",
                                                        [this]() { return params_.duty_cycle_neutral_deadband_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setDutyCycleNeutralDeadband, this, _1, false),
                                                        "Values less than this magnitue set motor output to 0",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("peak_forward_duty_cycle",
                                                        [this]() { return params_.peak_forward_duty_cycle_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakForwardDutyCycle, this, _1, false),
                                                        "Max %% output forward",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("peak_reverse_duty_cycle",
                                                        [this]() { return params_.peak_reverse_duty_cycle_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakReverseDutyCycle, this, _1, false),
                                                        "Max %% output reverse",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("stator_current_limit",
                                                        [this]() { return params_.stator_current_limit_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setStatorCurrentLimit, this, _1, false),
                                                        "Stator current limit in amps",
                                                        0, 800);
            ddr_updater_->ddr_.registerVariable<bool>  ("stator_current_limit_enable",
                                                        [this]() { return params_.stator_current_limit_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setStatorCurrentLimitEnable, this, _1, false),
                                                        "Enable stator current limit");
            ddr_updater_->ddr_.registerVariable<double>("supply_current_limit",
                                                        [this]() { return params_.supply_current_limit_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setSupplyCurrentLimit, this, _1, false),
                                                        "Supply current limit in amps",
                                                        0, 800);
            ddr_updater_->ddr_.registerVariable<bool>  ("supply_current_limit_enable",
                                                        [this]() { return params_.supply_current_limit_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setSupplyCurrentLimitEnable, this, _1, false),
                                                        "Enable supply current limit");
            ddr_updater_->ddr_.registerVariable<double>("supply_current_threshold",
                                                        [this]() { return params_.supply_current_threshold_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setSupplyCurrentThreshold, this, _1, false),
                                                        "Supply current threshold in amps",
                                                        0, 511);
            ddr_updater_->ddr_.registerVariable<double>("supply_time_threshold",
                                                        [this]() { return params_.supply_time_threshold_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setSupplyTimeThreshold, this, _1, false),
                                                        "Supply time threshold in amps",
                                                        0, 1.275);
            ddr_updater_->ddr_.registerVariable<double>("supply_voltage_time_constraint",
                                                        [this]() { return params_.supply_voltage_time_constant_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setSupplyVoltageTimeConstant, this, _1, false),
                                                        "The time constant (in seconds) of the low-pass filter for  the supply voltage",
                                                        0, 0.1);
            ddr_updater_->ddr_.registerVariable<double>("peak_forward_voltage",
                                                        [this]() { return params_.peak_forward_voltage_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakForwardVoltage, this, _1, false),
                                                        "Maximum (forward) output during voltage based control modes",
                                                        -16, 16);
            ddr_updater_->ddr_.registerVariable<double>("peak_reverse_voltage",
                                                        [this]() { return params_.peak_reverse_voltage_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakReverseVoltage, this, _1, false),
                                                        "Maximum (reverse) output during voltage based control modes",
                                                        -16, 16);
            ddr_updater_->ddr_.registerVariable<double>("peak_forward_torque_current",
                                                        [this]() { return params_.peak_forward_torque_current_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakForwardTorqueCurrent, this, _1, false),
                                                        "Maximum (forward) output during torque-current based control modes",
                                                        -800, 800);
            ddr_updater_->ddr_.registerVariable<double>("peak_reverse_torque_current",
                                                        [this]() { return params_.peak_reverse_torque_current_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakReverseTorqueCurrent, this, _1, false),
                                                        "Maximum (reverse) output during torque-current based control modes",
                                                        -800, 800);
            ddr_updater_->ddr_.registerVariable<double>("torque_neutral_deadband",
                                                        [this]() { return params_.torque_neutral_deadband_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setTorqueNeutralDeadband, this, _1, false),
                                                        "output deadband during torque-current based control modes",
                                                        0, 25);
            ddr_updater_->ddr_.registerVariable<double>("feedback_rotor_offset",
                                                        [this]() { return params_.feedback_rotor_offset_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setFeedbackRotorOffset, this, _1, false),
                                                        "offset is applied to the absolute integrated rotor sensor",
                                                        -1, 1);
            ddr_updater_->ddr_.registerVariable<double>("sensor_to_mechanism_ratio",
                                                        [this]() { return params_.sensor_to_mechanism_ratio_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setSensorToMechanismRatio, this, _1, false),
                                                        "The ratio of sensor rotations to the mechanism's output", 
                                                        -1000, 1000);
            ddr_updater_->ddr_.registerVariable<double>("rotor_to_sensor_ratio",
                                                        [this]() { return params_.rotor_to_sensor_ratio_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setRotorToSensorRatio, this, _1, false),
                                                        "the ratio between the remote sensor and the motor rotor",
                                                        -1000, 1000);

            ddr_updater_->ddr_.registerEnumVariable<int>("differential_sensor_source",
                                                        [this]() { return static_cast<int>(params_.differential_sensor_source_.load());},
                                                        [this](const int differential_sensor_source_int) { this->setDifferentialSensorSource(static_cast<hardware_interface::talonfxpro::DifferentialSensorSource>(differential_sensor_source_int), false);},
                                                        "Differential sensor source", 
                                                        std::map<std::string, int>{
                                                            {"Disabled", static_cast<int>(hardware_interface::talonfxpro::DifferentialSensorSource::Disabled)},
                                                            {"RemoteTalonFX_Diff", static_cast<int>(hardware_interface::talonfxpro::DifferentialSensorSource::RemoteTalonFX_Diff)},
                                                            {"RemotePigeon2_Yaw", static_cast<int>(hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Yaw)},
                                                            {"RemotePigeon2_Pitch", static_cast<int>(hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Pitch)},
                                                            {"RemotePigeon2_Roll", static_cast<int>(hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Roll)},
                                                            {"RemoteCANcoder", static_cast<int>(hardware_interface::talonfxpro::DifferentialSensorSource::RemoteCANcoder)}
                                                        }
                                                        );
            ddr_updater_->ddr_.registerVariable<int>   ("differential_talonfx_sensor_id_",
                                                        [this]() { return params_.differential_talonfx_sensor_id_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setDifferentialTalonFXSensorID, this, _1, false),
                                                        "differential talon fx sensor id",
                                                        0, 63);
            ddr_updater_->ddr_.registerVariable<int>   ("differential_remote_sensor_id_",
                                                        [this]() { return params_.differential_remote_sensor_id_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setDifferentialRemoteSensorID, this, _1, false),
                                                        "differential talon fx sensor id",
                                                        0, 63);
            ddr_updater_->ddr_.registerVariable<double>("peak_differential_duty_cycle",
                                                        [this]() { return params_.peak_differential_duty_cycle_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakDifferentialDutyCycle, this, _1, false),
                                                        "Maximum differential output during duty cycle based differential control modes",
                                                        0, 2);
            ddr_updater_->ddr_.registerVariable<double>("peak_differential_voltage",
                                                        [this]() { return params_.peak_differential_voltage_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakDifferentialVoltage, this, _1, false),
                                                        "Maximum differential output during voltage based differential control modes",
                                                        0, 32);
            ddr_updater_->ddr_.registerVariable<double>("peak_differential_torque_current",
                                                        [this]() { return params_.peak_differential_torque_current_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setPeakDifferentialTorqueCurrent, this, _1, false),
                                                        "Maximum differential output during torque_current based differential control modes",
                                                        0, 1600);
            ddr_updater_->ddr_.registerVariable<double>("duty_cycle_open_loop_ramp_period",
                                                        [this]() { return params_.duty_cycle_open_loop_ramp_period_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setDutyCycleOpenLoopRampPeriod, this, _1, false),
                                                        "If non-zero, this determines how much time to ramp from 0%% output to 100%% during open-loop modes",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("voltage_open_loop_ramp_period",
                                                        [this]() { return params_.voltage_open_loop_ramp_period_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setVoltageOpenLoopRampPeriod, this, _1, false),
                                                        "If non-zero, this determines how much time to ramp from 0V to 12V during open-loop modes",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("torque_open_loop_ramp_period",
                                                        [this]() { return params_.torque_open_loop_ramp_period_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setTorqueOpenLoopRampPeriod, this, _1, false),
                                                        "If non-zero, this determines how much time to ramp from 0A to 300A during open-loop modes",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("duty_cycle_closed_loop_ramp_period",
                                                        [this]() { return params_.duty_cycle_closed_loop_ramp_period_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setDutyCycleClosedLoopRampPeriod, this, _1, false),
                                                        "If non-zero, this determines how much time to ramp from 0%% output to 100%% during closed-loop modes",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("voltage_closed_loop_ramp_period",
                                                        [this]() { return params_.voltage_closed_loop_ramp_period_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setVoltageClosedLoopRampPeriod, this, _1, false),
                                                        "If non-zero, this determines how much time to ramp from 0V to 12V during closed-loop modes",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("torque_closed_loop_ramp_period",
                                                        [this]() { return params_.torque_closed_loop_ramp_period_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setTorqueClosedLoopRampPeriod, this, _1, false),
                                                        "If non-zero, this determines how much time to ramp from 0A to 300A during closed-loop modes",
                                                        0, 1);
            ddr_updater_->ddr_.registerEnumVariable<int>("forward_limit_type",
                                                        [this]() { return static_cast<int>(params_.forward_limit_type_.load());},
                                                        [this](const int forward_limit_type) { setForwardLimitType(static_cast<hardware_interface::talonfxpro::LimitType>(forward_limit_type));},
                                                        "Forward limit switch polarity",
                                                        limit_type_enum_map_);
            ddr_updater_->ddr_.registerVariable<bool>  ("forward_limit_autoset_position_enable",
                                                        [this]() { return params_.forward_limit_autoset_position_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setForwardLimitAutosetPositionEnable, this, _1, false),
                                                        "Forward Limit Autoset Position Enable");
            ddr_updater_->ddr_.registerVariable<double>("forward_limit_autoset_position_value",
                                                        [this]() { return params_.forward_limit_autoset_position_value_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setForwardLimitAutosetPositionValue, this, _1, false),
                                                        "Forward Limit Autoset Position Value",
                                                        -100, 100);
            ddr_updater_->ddr_.registerVariable<bool>  ("forward_limit_enable",
                                                        [this]() { return params_.forward_limit_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setForwardLimitEnable, this, _1, false),
                                                        "Forward Limit Enable");
            ddr_updater_->ddr_.registerEnumVariable<int>("forward_limit_source",
                                                        [this]() { return static_cast<int>(params_.forward_limit_source_.load());},
                                                        [this](const int forward_limit_source) { setForwardLimitSource(static_cast<hardware_interface::talonfxpro::LimitSource>(forward_limit_source));},
                                                        "Forward limit switch source",
                                                        limit_source_enum_map_);
            ddr_updater_->ddr_.registerVariable<int>   ("forward_limit_remote_sensor_id",
                                                        [this]() { return params_.forward_limit_remote_sensor_id_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setForwardLimitRemoteSensorID, this, _1, false),
                                                        "Forward Limit Remote Sensor ID",
                                                        0, 63);
            ddr_updater_->ddr_.registerEnumVariable<int>("reverse_limit_type",
                                                        [this]() { return static_cast<int>(params_.reverse_limit_type_.load());},
                                                        [this](const int reverse_limit_type) { setReverseLimitType(static_cast<hardware_interface::talonfxpro::LimitType>(reverse_limit_type));},
                                                        "Reverse limit switch polarity",
                                                        limit_type_enum_map_);
            ddr_updater_->ddr_.registerVariable<bool>  ("reverse_limit_autoset_position_enable",
                                                        [this]() { return params_.reverse_limit_autoset_position_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setReverseLimitAutosetPositionEnable, this, _1, false),
                                                        "Reverse Limit Autoset Position Enable");
            ddr_updater_->ddr_.registerVariable<double>("reverse_limit_autoset_position_value",
                                                        [this]() { return params_.reverse_limit_autoset_position_value_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setReverseLimitAutosetPositionValue, this, _1, false),
                                                        "Reverse Limit Autoset Position Value",
                                                        -100, 100);
            ddr_updater_->ddr_.registerVariable<bool>  ("reverse_limit_enable",
                                                        [this]() { return params_.reverse_limit_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setReverseLimitEnable, this, _1, false),
                                                        "Reverse Limit Enable");
            ddr_updater_->ddr_.registerEnumVariable<int>("reverse_limit_source",
                                                        [this]() { return static_cast<int>(params_.reverse_limit_source_.load());},
                                                        [this](const int reverse_limit_source) { setReverseLimitSource(static_cast<hardware_interface::talonfxpro::LimitSource>(reverse_limit_source));},
                                                        "Reverse limit switch source",
                                                        limit_source_enum_map_);
            ddr_updater_->ddr_.registerVariable<int>   ("reverse_limit_remote_sensor_id",
                                                        [this]() { return params_.reverse_limit_remote_sensor_id_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setReverseLimitRemoteSensorID, this, _1, false),
                                                        "Reverse Limit Remote Sensor ID",
                                                        0, 63);
            ddr_updater_->ddr_.registerVariable<bool>  ("beep_on_boot",
                                                        [this]() { return params_.beep_on_boot_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setBeepOnBoot, this, _1, false),
                                                        "Beep on Boot");
            ddr_updater_->ddr_.registerVariable<bool>  ("beep_on_config",
                                                        [this]() { return params_.beep_on_config_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setBeepOnConfig, this, _1, false),
                                                        "Beep on config");
            ddr_updater_->ddr_.registerVariable<bool>  ("allow_music_dur_disable",
                                                        [this]() { return params_.allow_music_dur_disable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setAllowMusicDurDisable, this, _1, false),
                                                        "Allow talons to play music while robot is disabled");
            ddr_updater_->ddr_.registerVariable<bool>  ("forward_softlimit_enable",
                                                        [this]() { return params_.softlimit_forward_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setForwardSoftLimitEnable, this, _1, false),
                                                        "Enable forward softlimit");
            ddr_updater_->ddr_.registerVariable<double>("forward_softlimit_threshold",
                                                        [this]() { return params_.softlimit_forward_threshold_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setForwardSoftLimitThreshold, this, _1, false),
                                                        "Forward Softlimit Threshold",
                                                        -100, 100);
            ddr_updater_->ddr_.registerVariable<bool>  ("reverse_softlimit_enable",
                                                        [this]() { return params_.softlimit_reverse_enable_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setReverseSoftLimitEnable, this, _1, false),
                                                        "Enable reverse softlimit");
            ddr_updater_->ddr_.registerVariable<double>("reverse_softlimit_threshold",
                                                        [this]() { return params_.softlimit_reverse_threshold_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setReverseSoftLimitThreshold, this, _1, false),
                                                        "Reverse Softlimit Threshold",
                                                        -100, 100);
            ddr_updater_->ddr_.registerVariable<double>("motion_magic_cruise_velocity",
                                                        [this]() { return params_.motion_magic_cruise_velocity_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setMotionMagicCruiseVelocity, this, _1, false),
                                                        "Motion Magic Cruise Velocity",
                                                        0, 9999);
            ddr_updater_->ddr_.registerVariable<double>("motion_magic_acceleration",
                                                        [this]() { return params_.motion_magic_acceleration_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setMotionMagicAcceleration, this, _1, false),
                                                        "Motion Magic Acceleration",
                                                        0, 9999);
            ddr_updater_->ddr_.registerVariable<double>("motion_magic_jerk",
                                                        [this]() { return params_.motion_magic_jerk_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setMotionMagicJerk, this, _1, false),
                                                        "Motion Magic Jerk",
                                                        0, 9999);
            ddr_updater_->ddr_.registerVariable<double>("motion_magic_expo_kV",
                                                        [this]() { return params_.motion_magic_expo_kV_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setMotionMagicExpoKV, this, _1, false),
                                                        "Motion Magic Expo kV",
                                                        0, 100);
            ddr_updater_->ddr_.registerVariable<double>("motion_magic_expo_kA",
                                                        [this]() { return params_.motion_magic_expo_kA_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setMotionMagicExpoKA, this, _1, false),
                                                        "Motion Magic Expo kA",
                                                        0, 100);
            ddr_updater_->ddr_.registerVariable<bool>  ("continuous_wrap",
                                                        [this]() { return params_.continuous_wrap_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setContinuousWrap, this, _1, false),
                                                        "Continouous Wrap");
            ddr_updater_->ddr_.registerVariable<bool>  ("control_enable_foc",
                                                        [this]() { return params_.control_enable_foc_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlEnableFOC, this, _1, false),
                                                        "Control Enable FOC");
            ddr_updater_->ddr_.registerVariable<bool>  ("control_override_brake_dur_neutral",
                                                        [this]() { return params_.control_override_brake_dur_neutral_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlOverrideBrakeDurNeutral, this, _1, false),
                                                        "Control Override Brake Dur Neutral");
            ddr_updater_->ddr_.registerVariable<double>("control_deadband",
                                                        [this]() { return params_.control_deadband_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlDeadband, this, _1, false),
                                                        "Control Deadband",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<double>("control_feedforward",
                                                        [this]() { return params_.control_feedforward_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlFeedforward, this, _1, false),
                                                        "Control Feedforward",
                                                        0, 1);
            ddr_updater_->ddr_.registerVariable<int>   ("control_slot",
                                                        [this]() { return params_.control_slot_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlSlot, this, _1, false),
                                                        "Control Slot",
                                                        0, 2);
            ddr_updater_->ddr_.registerVariable<bool>  ("control_oppose_master_direction",
                                                        [this]() { return params_.control_oppose_master_direction_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlOpposeMasterDirection, this, _1, false),
                                                        "Control oppose master direction");
            ddr_updater_->ddr_.registerVariable<bool>  ("control_limit_forward_motion",
                                                        [this]() { return params_.control_limit_forward_motion_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlLimitForwardMotion, this, _1, false),
                                                        "Control limit forward motion");
            ddr_updater_->ddr_.registerVariable<bool>  ("control_limit_reverse_motion",
                                                        [this]() { return params_.control_limit_reverse_motion_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlLimitReverseMotion, this, _1, false),
                                                        "Control limit reverse motion");
            ddr_updater_->ddr_.registerVariable<int>   ("control_differential_slot",
                                                        [this]() { return params_.control_differential_slot_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setControlDifferentialSlot, this, _1, false),
                                                        "Control Differential Slot",
                                                        0, 2);
            ddr_updater_->ddr_.registerVariable<bool>  ("enable_read_thread",
                                                        [this]() { return params_.enable_read_thread_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setEnableReadThread, this, _1, false),
                                                        "Clear to disable this motor's read thread");
            ddr_updater_->ddr_.registerVariable<double>("set_position",
                                                        [this]() { return params_.set_position_.load();},
                                                        boost::bind(&TalonFXProControllerInterface::setRotorPosition, this, _1, false),
                                                        "Immediately set motor position",
                                                        -std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            ddr_updater_->ddr_.PublishServicesTopics();
        }
    }
    ROS_WARN_STREAM(__PRETTY_FUNCTION__ << "init past ddynamic_reconfigure init for " << params.joint_name_);
    return true;
}

void TalonFXProControllerInterface::setControlOutput(const double control_output)
{
    talon_->setControlOutput(control_output);
}

void TalonFXProControllerInterface::setControlPosition(const double control_position)
{
    talon_->setControlPosition(control_position);
}

void TalonFXProControllerInterface::setControlVelocity(const double control_velocity)
{
    talon_->setControlVelocity(control_velocity);
}

void TalonFXProControllerInterface::setControlAcceleration(const double control_acceleration)
{
    talon_->setControlAcceleration(control_acceleration);
}

void TalonFXProControllerInterface::setControlJerk(const double control_jerk)
{
    talon_->setControlJerk(control_jerk);
}

void TalonFXProControllerInterface::setControlDifferentialPosition(const double control_differential_position)
{
    talon_->setControlDifferentialPosition(control_differential_position);
}

// Functions which handle dynamic reconfigurable config vars
// Template (kinda) for params which are arrays. Currently this is just
// PID config slots
#define SET_PARAM_IDX_FN(param, talon_fn)                                                                                                  \
void TalonFXProControllerInterface::talon_fn(const decltype(param)::value_type::value_type val, const size_t index, const bool update_ddr) \
{                                                                                                                                          \
    param[index] = val;                                                                                                                    \
    talon_->talon_fn(val, index);                                                                                                          \
    if (update_ddr)                                                                                                                        \
    {                                                                                                                                      \
        ddr_updater_->triggerDDRUpdate();                                                                                                  \
    }                                                                                                                                      \
}

SET_PARAM_IDX_FN(params_.kP_, setkP)
SET_PARAM_IDX_FN(params_.kI_, setkI)
SET_PARAM_IDX_FN(params_.kD_, setkD)
SET_PARAM_IDX_FN(params_.kS_, setkS)
SET_PARAM_IDX_FN(params_.kV_, setkV)
SET_PARAM_IDX_FN(params_.kA_, setkA)
SET_PARAM_IDX_FN(params_.kG_, setkG)
SET_PARAM_IDX_FN(params_.gravity_type_, setGravityType)

// Template to generate functions for normal param entries
// Takes the param name and talon function to call to set the value
#define SET_PARAM_FN(param, talon_fn)                                                                      \
void TalonFXProControllerInterface::talon_fn(const decltype(param)::value_type val, const bool update_ddr) \
{                                                                                                          \
    param = val;                                                                                           \
    talon_->talon_fn(val);                                                                                 \
    if (update_ddr)                                                                                        \
    {                                                                                                      \
        ddr_updater_->triggerDDRUpdate();                                                                  \
    }                                                                                                      \
}

SET_PARAM_FN(params_.invert_, setInvert);
SET_PARAM_FN(params_.neutral_mode_, setNeutralMode)
SET_PARAM_FN(params_.duty_cycle_neutral_deadband_, setDutyCycleNeutralDeadband)
SET_PARAM_FN(params_.peak_forward_duty_cycle_, setPeakForwardDutyCycle)
SET_PARAM_FN(params_.peak_reverse_duty_cycle_, setPeakReverseDutyCycle);
SET_PARAM_FN(params_.stator_current_limit_, setStatorCurrentLimit)
SET_PARAM_FN(params_.stator_current_limit_enable_, setStatorCurrentLimitEnable);
SET_PARAM_FN(params_.supply_current_limit_, setSupplyCurrentLimit);
SET_PARAM_FN(params_.supply_current_limit_enable_, setSupplyCurrentLimitEnable);
SET_PARAM_FN(params_.supply_current_threshold_, setSupplyCurrentThreshold);
SET_PARAM_FN(params_.supply_time_threshold_, setSupplyTimeThreshold);
SET_PARAM_FN(params_.supply_voltage_time_constant_, setSupplyVoltageTimeConstant);
SET_PARAM_FN(params_.peak_forward_voltage_, setPeakForwardVoltage);
SET_PARAM_FN(params_.peak_reverse_voltage_, setPeakReverseVoltage);
SET_PARAM_FN(params_.peak_forward_torque_current_, setPeakForwardTorqueCurrent);
SET_PARAM_FN(params_.peak_reverse_torque_current_, setPeakReverseTorqueCurrent);
SET_PARAM_FN(params_.torque_neutral_deadband_, setTorqueNeutralDeadband);
SET_PARAM_FN(params_.feedback_rotor_offset_, setFeedbackRotorOffset);
SET_PARAM_FN(params_.sensor_to_mechanism_ratio_, setSensorToMechanismRatio);
SET_PARAM_FN(params_.rotor_to_sensor_ratio_, setRotorToSensorRatio);
SET_PARAM_FN(params_.differential_sensor_source_, setDifferentialSensorSource);
SET_PARAM_FN(params_.differential_talonfx_sensor_id_, setDifferentialTalonFXSensorID);
SET_PARAM_FN(params_.differential_remote_sensor_id_, setDifferentialRemoteSensorID);
SET_PARAM_FN(params_.peak_differential_duty_cycle_, setPeakDifferentialDutyCycle);
SET_PARAM_FN(params_.peak_differential_voltage_, setPeakDifferentialVoltage);
SET_PARAM_FN(params_.peak_differential_torque_current_, setPeakDifferentialTorqueCurrent);
SET_PARAM_FN(params_.duty_cycle_open_loop_ramp_period_, setDutyCycleOpenLoopRampPeriod);
SET_PARAM_FN(params_.voltage_open_loop_ramp_period_, setVoltageOpenLoopRampPeriod);
SET_PARAM_FN(params_.torque_open_loop_ramp_period_, setTorqueOpenLoopRampPeriod);
SET_PARAM_FN(params_.duty_cycle_closed_loop_ramp_period_, setDutyCycleClosedLoopRampPeriod);
SET_PARAM_FN(params_.voltage_closed_loop_ramp_period_, setVoltageClosedLoopRampPeriod);
SET_PARAM_FN(params_.torque_closed_loop_ramp_period_, setTorqueClosedLoopRampPeriod);
SET_PARAM_FN(params_.forward_limit_type_, setForwardLimitType);
SET_PARAM_FN(params_.forward_limit_autoset_position_enable_, setForwardLimitAutosetPositionEnable);
SET_PARAM_FN(params_.forward_limit_autoset_position_value_, setForwardLimitAutosetPositionValue);
SET_PARAM_FN(params_.forward_limit_enable_, setForwardLimitEnable);
SET_PARAM_FN(params_.forward_limit_source_, setForwardLimitSource);
SET_PARAM_FN(params_.forward_limit_remote_sensor_id_, setForwardLimitRemoteSensorID);
SET_PARAM_FN(params_.reverse_limit_type_, setReverseLimitType);
SET_PARAM_FN(params_.reverse_limit_autoset_position_enable_, setReverseLimitAutosetPositionEnable);
SET_PARAM_FN(params_.reverse_limit_autoset_position_value_, setReverseLimitAutosetPositionValue);
SET_PARAM_FN(params_.reverse_limit_enable_, setReverseLimitEnable);
SET_PARAM_FN(params_.reverse_limit_source_, setReverseLimitSource);
SET_PARAM_FN(params_.reverse_limit_remote_sensor_id_, setReverseLimitRemoteSensorID);
SET_PARAM_FN(params_.beep_on_boot_, setBeepOnBoot);
SET_PARAM_FN(params_.beep_on_config_, setBeepOnConfig);
SET_PARAM_FN(params_.allow_music_dur_disable_, setAllowMusicDurDisable);
SET_PARAM_FN(params_.softlimit_forward_enable_, setForwardSoftLimitEnable);
SET_PARAM_FN(params_.softlimit_reverse_enable_, setReverseSoftLimitEnable);
SET_PARAM_FN(params_.softlimit_forward_threshold_, setForwardSoftLimitThreshold);
SET_PARAM_FN(params_.softlimit_reverse_threshold_, setReverseSoftLimitThreshold);
SET_PARAM_FN(params_.motion_magic_cruise_velocity_, setMotionMagicCruiseVelocity);
SET_PARAM_FN(params_.motion_magic_acceleration_, setMotionMagicAcceleration);
SET_PARAM_FN(params_.motion_magic_jerk_, setMotionMagicJerk);
SET_PARAM_FN(params_.motion_magic_expo_kV_, setMotionMagicExpoKV);
SET_PARAM_FN(params_.motion_magic_expo_kA_, setMotionMagicExpoKA);
SET_PARAM_FN(params_.continuous_wrap_, setContinuousWrap);
SET_PARAM_FN(params_.control_enable_foc_, setControlEnableFOC);
SET_PARAM_FN(params_.control_override_brake_dur_neutral_, setControlOverrideBrakeDurNeutral);
SET_PARAM_FN(params_.control_max_abs_duty_cycle_, setControlMaxAbsDutyCycle);
SET_PARAM_FN(params_.control_deadband_, setControlDeadband);
SET_PARAM_FN(params_.control_feedforward_, setControlFeedforward);
SET_PARAM_FN(params_.control_slot_, setControlSlot);
SET_PARAM_FN(params_.control_oppose_master_direction_, setControlOpposeMasterDirection);
SET_PARAM_FN(params_.control_limit_forward_motion_, setControlLimitForwardMotion);
SET_PARAM_FN(params_.control_limit_reverse_motion_, setControlLimitReverseMotion);
SET_PARAM_FN(params_.control_differential_slot_, setControlDifferentialSlot);
SET_PARAM_FN(params_.enable_read_thread_, setEnableReadThread);
SET_PARAM_FN(params_.set_position_, setRotorPosition);

void TalonFXProControllerInterface::setControlMode(const hardware_interface::talonfxpro::TalonMode control_mode)
{
    //ROS_WARN_STREAM(__FUNCTION__ << " " << talon_.state()->getCANID() << " control_mode = " << (int)control_mode);
    talon_->setControlMode(control_mode);
}

#ifdef TALONCI_BACKWARDS_COMPATIBILITY
    hardware_interface::TalonMode TalonFXProControllerInterface::getMode(void) const
    {
        if (!talon_.state()->getDeviceEnable())
        {
            return hardware_interface::TalonMode::TalonMode_Disabled;
        }
        switch(talon_.state()->getControlMode())
        {
        case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
        case hardware_interface::talonfxpro::TalonMode::VoltageOut:
            return hardware_interface::TalonMode::TalonMode_PercentOutput;
        case hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC:
            return hardware_interface::TalonMode::TalonMode_Current; // CloseLoop
        case hardware_interface::talonfxpro::TalonMode::PositionDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::PositionVoltage:
        case hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC:
            return hardware_interface::TalonMode::TalonMode_Position; // CloseLoop
        case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
        case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
            return hardware_interface::TalonMode::TalonMode_Velocity; // CloseLoop
        case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
            return hardware_interface::TalonMode::TalonMode_MotionMagic;
        case hardware_interface::talonfxpro::TalonMode::Follower:
        case hardware_interface::talonfxpro::TalonMode::StrictFollower:
            return hardware_interface::TalonMode::TalonMode_Follower;
        case hardware_interface::talonfxpro::TalonMode::NeutralOut:
        case hardware_interface::talonfxpro::TalonMode::CoastOut:
        case hardware_interface::talonfxpro::TalonMode::StaticBrake:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : NeutralOut / CoastOut / StaticBrake");
            return hardware_interface::TalonMode::TalonMode_First;
        case hardware_interface::talonfxpro::TalonMode::Disabled:
            return hardware_interface::TalonMode::TalonMode_Disabled;
        default:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : Unknown (" << static_cast<int>(talon_.state()->getControlMode()) << ")");
            return hardware_interface::TalonMode::TalonMode_First;
        }
    }

    void TalonFXProControllerInterface::setMode(const hardware_interface::TalonMode mode)
    {
        //ROS_INFO_STREAM_THROTTLE(.2, __FUNCTION__ << " mode = " << mode);
        switch (mode)
        {
        case hardware_interface::TalonMode::TalonMode_PercentOutput:
            setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
            break;
        case hardware_interface::TalonMode::TalonMode_Position: // Use voltage to emulate v5 voltage compensation
            setControlMode(hardware_interface::talonfxpro::TalonMode::PositionVoltage);
            break;
        case hardware_interface::TalonMode::TalonMode_Velocity: // Use voltage to emulate v5 voltage compensation
            setControlMode(hardware_interface::talonfxpro::TalonMode::VelocityVoltage);
            break;
        case hardware_interface::TalonMode::TalonMode_Current: // CloseLoop
            setControlMode(hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC);
            break;
        case hardware_interface::TalonMode::TalonMode_Follower:
            setControlMode(hardware_interface::talonfxpro::TalonMode::StrictFollower);
            break;
        case hardware_interface::TalonMode::TalonMode_MotionProfile:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : MotionProfile");
            break;
        case hardware_interface::TalonMode::TalonMode_MotionMagic: // Use voltage to emulate v5 voltage compensation
            setControlMode(hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage);
            break;
        case hardware_interface::TalonMode::TalonMode_MotionProfileArc:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : MotionProfileArc");
            break;
        case hardware_interface::TalonMode::TalonMode_Music:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : Music");
            break;
        case hardware_interface::TalonMode::TalonMode_Disabled:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : Disabled");
            break;
        default:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Unknown control mode : " << mode);
        }
    }

    void TalonFXProControllerInterface::setDemand1Type(const hardware_interface::DemandType demand_type) const
    {
        if (demand_type == hardware_interface::DemandType_AuxPID)
        {
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid demand type : AuxPID not implemented");
        }
        // Otherwise hope there's a call to zero out arb feed forward value when set to neutral_mode
        // If not, we'll need to save previously set values, set all slots to 0,
        // then restore the values when set back to arb FF
    }

    void TalonFXProControllerInterface::setDemand1Value(const double demand_value)
    {
        // TODO : should this be kS or FF?
        setControlFeedforward(demand_value);
    }

    void TalonFXProControllerInterface::setMotionCruiseVelocity(const double motion_cruise_velocity)
    {
        setMotionMagicCruiseVelocity(motion_cruise_velocity);
    }
    void TalonFXProControllerInterface::setMotionAcceleration(const double motion_acceleration)
    {
        setMotionMagicAcceleration(motion_acceleration);
    }
    void TalonFXProControllerInterface::setMotionSCurveStrength(const double motion_s_curve_strength)
    {
        setMotionMagicJerk(motion_s_curve_strength);
    }
    void TalonFXProControllerInterface::setPIDFSlot(const int slot)
    {
        setControlSlot(slot);
    }
    void TalonFXProControllerInterface::setSelectedSensorPosition(const double sensor_position)
    {
        setRotorPosition(sensor_position);
    }

    void TalonFXProControllerInterface::setCommand(const double command)
    {
        //ROS_INFO_STREAM_THROTTLE(0.2, __FUNCTION__ << " command = " << command);
        // convert to the appropriate setControl* function based on current mode
        switch(talon_->getControlMode())
        {
        case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
        case hardware_interface::talonfxpro::TalonMode::VoltageOut:
        case hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC:
            talon_->setControlOutput(command);
            talon_->setControlPosition(0);
            talon_->setControlVelocity(0);
            talon_->setControlAcceleration(0);
            break;
        case hardware_interface::talonfxpro::TalonMode::PositionDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::PositionVoltage:
        case hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC:
            talon_->setControlOutput(0);
            talon_->setControlPosition(command);
            talon_->setControlVelocity(0);
            talon_->setControlAcceleration(0);
            break;
        case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
        case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC:
            talon_->setControlOutput(0);
            talon_->setControlPosition(0);
            talon_->setControlVelocity(command);
            talon_->setControlAcceleration(0);
            break;
        case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage:
        case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoTorqueCurrentFOC:
        case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle:
        case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage:
        case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC:
            talon_->setControlOutput(0);
            talon_->setControlPosition(command);
            talon_->setControlVelocity(0);
            talon_->setControlAcceleration(0);
            break;
        case hardware_interface::talonfxpro::TalonMode::Follower:
        case hardware_interface::talonfxpro::TalonMode::StrictFollower:
            break;
        case hardware_interface::talonfxpro::TalonMode::NeutralOut:
        case hardware_interface::talonfxpro::TalonMode::CoastOut:
        case hardware_interface::talonfxpro::TalonMode::StaticBrake:
            talon_->setControlOutput(0);
            talon_->setControlPosition(0);
            talon_->setControlVelocity(0);
            talon_->setControlAcceleration(0);
            //ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : NeutralOut / CoastOut / StaticBrake");
            break;
        case hardware_interface::talonfxpro::TalonMode::Disabled:
            return;
        default:
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : Invalid control mode : Unknown (" << static_cast<int>(talon_->getControlMode()) << ")");
            return;
        }
    }

    void TalonFXProControllerInterface::setNeutralMode(const hardware_interface::NeutralMode neutral_mode)
    {
        if (neutral_mode == hardware_interface::NeutralMode_Coast)
        {
            setNeutralMode(hardware_interface::talonfxpro::NeutralMode::Coast);
        }
        else if (neutral_mode == hardware_interface::NeutralMode_Brake)
        {
            setNeutralMode(hardware_interface::talonfxpro::NeutralMode::Brake);
        }
        else
        {
            ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Unknown neutral mode " << neutral_mode);
        }
    }
#endif

bool TalonFXProControllerInterface::readParams(const ros::NodeHandle &n, TalonFXProCIParams &params) const
{
    // Call each params read function
    return params.readJointName(n) &&
           params.readCloseLoopParams(n) &&
           params.readInvert(n) &&
           params.readNeutralMode(n) &&
           params.readDutyCycleOutputShaping(n) &&
           params.readStatorCurrentLimits(n) &&
           params.readSupplyCurrentLimits(n) &&
           params.readVoltageLimits(n) &&
           params.readTorqueLimits(n) &&
           params.readFeedback(n) &&
           params.readDifferentialConfig(n) &&
           params.readDifferentialConstants(n) &&
           params.readOpenLoopRamps(n) &&
           params.readClosedLoopRamps(n) &&
           params.readLimitSwitches(n) &&
           params.readAudio(n) &&
           params.readSoftLimits(n) &&
           params.readMotionMagic(n) &&
           params.readContinuousWrap(n) &&
           params.readControl(n) &&
           params.readEnableReadThread(n) &&
           params.readTypeStringControlMode(n);
}

void TalonFXProControllerInterface::writeParamsToHW(TalonFXProCIParams &params,
                                                    hardware_interface::talonfxpro::TalonFXProCommandHandle &talon) const
{
    // For each entry in params, write the value to the cooresponding
    // talon set() config call
    for (size_t i = 0; i < hardware_interface::talonfxpro::TALON_PIDF_SLOTS; i++)
    {
        talon->setkP(params.kP_[i], i);
        talon->setkI(params.kI_[i], i);
        talon->setkD(params.kD_[i], i);
        talon->setkS(params.kS_[i], i);
        talon->setkV(params.kV_[i], i);
        talon->setkA(params.kA_[i], i);
        talon->setkG(params.kG_[i], i);
        talon->setGravityType(params.gravity_type_[i], i);
    }
    talon->setInvert(params.invert_);
    talon->setNeutralMode(params.neutral_mode_);
    talon->setDutyCycleNeutralDeadband(params.duty_cycle_neutral_deadband_);
    talon->setPeakForwardDutyCycle(params.peak_forward_duty_cycle_);
    talon->setPeakReverseDutyCycle(params.peak_reverse_duty_cycle_);
    talon->setStatorCurrentLimit(params.stator_current_limit_);
    talon->setStatorCurrentLimitEnable(params.stator_current_limit_enable_);
    talon->setSupplyCurrentLimit(params.supply_current_limit_);
    talon->setSupplyCurrentLimitEnable(params.supply_current_limit_enable_);
    talon->setSupplyCurrentThreshold(params.supply_current_threshold_);
    talon->setSupplyTimeThreshold(params.supply_time_threshold_);
    talon->setSupplyVoltageTimeConstant(params.supply_voltage_time_constant_);
    talon->setPeakForwardVoltage(params.peak_forward_voltage_);
    talon->setPeakReverseVoltage(params.peak_reverse_voltage_);
    talon->setPeakForwardTorqueCurrent(params.peak_forward_torque_current_);
    talon->setPeakReverseTorqueCurrent(params.peak_reverse_torque_current_);
    talon->setTorqueNeutralDeadband(params.torque_neutral_deadband_);
    talon->setFeedbackRotorOffset(params.feedback_rotor_offset_);
    talon->setSensorToMechanismRatio(params.sensor_to_mechanism_ratio_);
    talon->setRotorToSensorRatio(params.rotor_to_sensor_ratio_);
    talon->setFeedbackSensorSource(params.feedback_sensor_source_);
    talon->setFeedbackRemoteSensorID(params.feedback_remote_sensor_id_);
    talon->setDifferentialSensorSource(params.differential_sensor_source_);
    talon->setDifferentialTalonFXSensorID(params.differential_talonfx_sensor_id_);
    talon->setDifferentialRemoteSensorID(params.differential_remote_sensor_id_);
    talon->setPeakDifferentialDutyCycle(params.peak_differential_duty_cycle_);
    talon->setPeakDifferentialVoltage(params.peak_differential_voltage_);
    talon->setPeakDifferentialTorqueCurrent(params.peak_differential_torque_current_);
    talon->setDutyCycleOpenLoopRampPeriod(params.duty_cycle_open_loop_ramp_period_);
    talon->setVoltageOpenLoopRampPeriod(params.voltage_open_loop_ramp_period_);
    talon->setTorqueOpenLoopRampPeriod(params.torque_open_loop_ramp_period_);
    talon->setDutyCycleClosedLoopRampPeriod(params.duty_cycle_closed_loop_ramp_period_);
    talon->setVoltageClosedLoopRampPeriod(params.voltage_closed_loop_ramp_period_);
    talon->setTorqueClosedLoopRampPeriod(params.torque_closed_loop_ramp_period_);
    talon->setForwardLimitType(params.forward_limit_type_);
    talon->setForwardLimitAutosetPositionEnable(params.forward_limit_autoset_position_enable_);
    talon->setForwardLimitAutosetPositionValue(params.forward_limit_autoset_position_value_);
    talon->setForwardLimitEnable(params.forward_limit_enable_);
    talon->setForwardLimitSource(params.forward_limit_source_);
    talon->setForwardLimitRemoteSensorID(params.forward_limit_remote_sensor_id_);
    talon->setReverseLimitType(params.reverse_limit_type_);
    talon->setReverseLimitAutosetPositionEnable(params.reverse_limit_autoset_position_enable_);
    talon->setReverseLimitAutosetPositionValue(params.reverse_limit_autoset_position_value_);
    talon->setReverseLimitEnable(params.reverse_limit_enable_);
    talon->setReverseLimitSource(params.reverse_limit_source_);
    talon->setReverseLimitRemoteSensorID(params.reverse_limit_remote_sensor_id_);
    talon->setBeepOnBoot(params.beep_on_boot_);
    talon->setBeepOnConfig(params.beep_on_config_);
    talon->setAllowMusicDurDisable(params.allow_music_dur_disable_);
    talon->setForwardSoftLimitEnable(params.softlimit_forward_enable_);
    talon->setReverseSoftLimitEnable(params.softlimit_reverse_enable_);
    talon->setForwardSoftLimitThreshold(params.softlimit_forward_threshold_);
    talon->setReverseSoftLimitThreshold(params.softlimit_reverse_threshold_);
    talon->setMotionMagicCruiseVelocity(params.motion_magic_cruise_velocity_);
    talon->setMotionMagicAcceleration(params.motion_magic_acceleration_);
    talon->setMotionMagicJerk(params.motion_magic_jerk_);
    talon->setMotionMagicExpoKV(params.motion_magic_expo_kV_);
    talon->setMotionMagicExpoKA(params.motion_magic_expo_kA_);
    talon->setContinuousWrap(params.continuous_wrap_);
    talon->setControlEnableFOC(params.control_enable_foc_);
    talon->setControlOverrideBrakeDurNeutral(params.control_override_brake_dur_neutral_);
    talon->setControlMaxAbsDutyCycle(params.control_max_abs_duty_cycle_);
    talon->setControlDeadband(params.control_deadband_);
    talon->setControlFeedforward(params.control_feedforward_);
    talon->setControlSlot(params.control_slot_);
    talon->setControlOpposeMasterDirection(params.control_oppose_master_direction_);
    talon->setControlLimitForwardMotion(params.control_limit_forward_motion_);
    talon->setControlLimitReverseMotion(params.control_limit_reverse_motion_);
    talon->setEnableReadThread(params.enable_read_thread_);
    talon->setControlDifferentialSlot(params.control_differential_slot_);
    // Don't call setSetRotorPosition at startup since this isn't a value read from config
}

bool TalonFXProControllerInterface::setInitialControlMode(void)
{
    ROS_INFO_STREAM("TalonFXPro " << talon_.getName() << " base class setInitialControlMode");
    return true;
}

void TalonFXProControllerInterface::setControlModeFromTypeString(void)
{
    if (params_.type_string_control_mode_ == hardware_interface::talonfxpro::TalonMode::First)
    {
        ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " : Can't set mode because type: was set to base class");
    }
    else
    {
        talon_->setControlMode(params_.type_string_control_mode_);
    }
}

//
// TalonFXProFixedModeControllerInterface member function specializations
//
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlMode(const hardware_interface::talonfxpro::TalonMode /*mode*/)
{
    ROS_WARN("Can't reset mode using this TalonFXProControllerInterface");
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
bool TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>::setInitialControlMode(void)
{
    // Set the mode at init time - since this
    // class is derived from the FixedMode class
    // it can't be reset
    talon_->setControlMode(TALON_MODE);
    return true;
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlJerk(const double control_jerk)
{
    ROS_WARN_THROTTLE(1, "Control jerk is not used for this controller type");
    TalonFXProControllerInterface::setControlJerk(control_jerk);
}

//
// TalonFXProPositionControllerInterface member function specializations
//
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProPositionControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlMode(const hardware_interface::talonfxpro::TalonMode /*mode*/)
{
    ROS_WARN("Can't reset mode using this TalonFXProControllerInterface");
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProPositionControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlOutput(const double control_output)
{
    ROS_WARN_THROTTLE(1, "Control output is not used for position controllers, use setControlPosition() for setpoint");
    TalonFXProControllerInterface::setControlOutput(control_output);
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProPositionControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlAcceleration(const double control_acceleration)
{
    ROS_WARN_THROTTLE(1, "Control acceleration is not used for position controllers, use setControlPosition() for setpoint");
    TalonFXProControllerInterface::setControlAcceleration(control_acceleration);
}

//
// TalonFXProVelocityControllerInterface member function specializations
//
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProVelocityControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlOutput(const double control_output)
{
    ROS_WARN_THROTTLE(1, "Control output is not used for velocity controllers, use setControlVelocity() for setpoint");
    TalonFXProControllerInterface::setControlOutput(control_output);
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProVelocityControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlPosition(const double control_position)
{
    ROS_WARN_THROTTLE(1, "Control position is not used for velocity controllers, use setControlVelocity() for setpoint");
    TalonFXProControllerInterface::setControlPosition(control_position);
}

//
// TalonFXProMotionMagicControllerInterface member function specializations
//
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProMotionMagicControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlOutput(const double control_output)
{
    ROS_WARN_THROTTLE(1, "Control output is not used for motion magic controllers, use setControlPosition() for setpoint");
    TalonFXProControllerInterface::setControlOutput(control_output);
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProMotionMagicControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlVelocity(const double control_velocity)
{
    ROS_WARN_THROTTLE(1, "Control velocity is not used for motion magic controllers, use setControlPosition() for setpoint");
    TalonFXProControllerInterface::setControlVelocity(control_velocity);
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProMotionMagicControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlAcceleration(const double control_acceleration)
{
    ROS_WARN_THROTTLE(1, "Control acceleration is not used for motion magic controllers, use setControlPosition() for setpoint");
    TalonFXProControllerInterface::setControlAcceleration(control_acceleration);
}

//
// TalonFXProMotionMagicVelocityControllerInterface member function specializations
//
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProMotionMagicVelocityControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlOutput(const double control_output)
{
    ROS_WARN_THROTTLE(1, "Control output is not used for velocity motion magic controllers, use setControlVelocity() for setpoint");
    TalonFXProControllerInterface::setControlOutput(control_output);
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProMotionMagicVelocityControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlPosition(const double control_position)
{
    ROS_WARN_THROTTLE(1, "Control position is not used for velocity motion magic controllers, use setControlVelocity() for setpoint");
    TalonFXProControllerInterface::setControlPosition(control_position);
}
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProMotionMagicVelocityControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlAcceleration(const double control_acceleration)
{
    ROS_WARN_THROTTLE(1, "Control acceleration is not used for velocity motion magic controllers, use setControlVelocity() for setpoint");
    TalonFXProControllerInterface::setControlAcceleration(control_acceleration);
}

//
// TalonFXProDynamicMotionMagicControllerInterface member function specializations
//
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProDynamicMotionMagicControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlOutput(const double control_output)
{
    ROS_WARN_THROTTLE(1, "Control output is not used for dynamic motion magic controllers, use setControlPosition() for setpoint");
    TalonFXProControllerInterface::setControlOutput(control_output);
}
// This is the only set of controllers which cares about the control jerk setting
// so override the function in the base class which warns if it is set
template<typename hardware_interface::talonfxpro::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFXProDynamicMotionMagicControllerInterface<TALON_MODE, TALON_MODE_NAME>::setControlJerk(const double control_jerk)
{
    TalonFXProControllerInterface::setControlJerk(control_jerk);
}

//
// TalonFXFollowerControllerInterface member function specializations
//
template <bool STRICT>
bool TalonFXProFollowerControllerInterfaceBase<STRICT>::initWithNode(hardware_interface::talonfxpro::TalonFXProCommandInterface *tci,
                                                                     hardware_interface::talonfxpro::TalonFXProStateInterface *tsi,
                                                                     ros::NodeHandle &n)
{
    if (!tsi)
    {
        ROS_ERROR("NULL TalonFXProStateInterface in TalonFXProFollowerBaseCommandInterface");
        return false;
    }

    // Call base-class init to load config params
    if (!TalonFXProControllerInterface::initWithNode(tci, tsi, n))
    {
        ROS_ERROR("TalonFXProFollowerController base initWithNode failed");
        return false;
    }

    std::string follow_joint_name;
    if (!n.getParam("follow_joint", follow_joint_name))
    {
        ROS_ERROR("No follow joint specified for TalonFollowerControllerInterface");
        return false;
    }

    hardware_interface::talonfxpro::TalonFXProStateHandle follow_handle = tsi->getHandle(follow_joint_name);
    const int follow_can_id = follow_handle->getCANID();

    // Set the mode and CAN ID of talon to follow at init time -
    // since this class is derived from the FixedMode class
    // these can't be reset. Hopefully we never have a case
    // where a follower mode Talon changes which other
    // Talon it is following during a match?
    talon_->setControlMode(STRICT ? hardware_interface::talonfxpro::TalonMode::StrictFollower : hardware_interface::talonfxpro::TalonMode::Follower);
    talon_->setControlOutput(follow_can_id);

    ROS_INFO_STREAM("Launching TalonFXPro follower " << params_.joint_name_ << " to " << (STRICT ? " strictly" : "") << " follow CAN ID " << follow_can_id << " (" << follow_handle.getName() << ")");
    return true;
}

template <bool STRICT>
void TalonFXProFollowerControllerInterfaceBase<STRICT>::setControlMode(const hardware_interface::talonfxpro::TalonMode /*mode*/)
{
    ROS_WARN("Can't reset mode using this TalonFXProControllerInterface");
}

template <bool STRICT>
void TalonFXProFollowerControllerInterfaceBase<STRICT>::setControlOutput(const double /*command*/)
{
    ROS_WARN("Can't set a command in follower mode!");
}
template <bool STRICT>
void TalonFXProFollowerControllerInterfaceBase<STRICT>::setControlPosition(const double /*command*/)
{
    ROS_WARN("Can't set a position in follower mode!");
}
template <bool STRICT>
void TalonFXProFollowerControllerInterfaceBase<STRICT>::setControlVelocity(const double /*command*/)
{
    ROS_WARN("Can't set a velocity in follower mode!");
}
template <bool STRICT>
void TalonFXProFollowerControllerInterfaceBase<STRICT>::setControlAcceleration(const double /*command*/)
{
    ROS_WARN("Can't set an acceleration in follower mode!");
}

// Explicit instantiation of the controller zoo
template class TalonFXProFixedModeControllerInterface<hardware_interface::talonfxpro::TalonMode::DutyCycleOut, DUTY_CYCLE_NAME>;
template class TalonFXProFixedModeControllerInterface<hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC, TORQUE_CURRENT_FOC_NAME>;
template class TalonFXProFixedModeControllerInterface<hardware_interface::talonfxpro::TalonMode::VoltageOut, VOLTAGE_NAME>;
template class TalonFXProPositionControllerInterface<hardware_interface::talonfxpro::TalonMode::PositionDutyCycle, POSITION_DUTY_CYCLE_NAME>;
template class TalonFXProPositionControllerInterface<hardware_interface::talonfxpro::TalonMode::PositionVoltage, POSITION_VOLTAGE_NAME>;
template class TalonFXProPositionControllerInterface<hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC, POSITION_TORQUE_CURRENT_FOC_NAME>;
template class TalonFXProVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle, VELOCITY_DUTY_CYCLE_NAME>;
template class TalonFXProVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::VelocityVoltage, VELOCITY_VOLTAGE_NAME>;
template class TalonFXProVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC, VELOCITY_TORQUE_CURRENT_FOC_NAME>;
template class TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage, MOTION_MAGIC_VOLTAGE_NAME>;
template class TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle, MOTION_MAGIC_DUTY_CYCLE_NAME>;
template class TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC, MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME>;
template class TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage, MOTION_MAGIC_EXPO_VOLTAGE_NAME>;
template class TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicExpoDutyCycle, MOTION_MAGIC_EXPO_DUTY_CYCLE_NAME>;
template class TalonFXProMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicExpoTorqueCurrentFOC, MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC_NAME>;
template class TalonFXProMotionMagicVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle, MOTION_MAGIC_VELOCITY_DUTY_CYCLE_NAME>;
template class TalonFXProMotionMagicVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage, MOTION_MAGIC_VELOCITY_VOLTAGE_NAME>;
template class TalonFXProMotionMagicVelocityControllerInterface<hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC, MOTION_MAGIC_VELOCITY_TORQUE_CURRENT_FOC_NAME>;
template class TalonFXProDynamicMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage, DYNAMIC_MOTION_MAGIC_VOLTAGE_NAME>;
template class TalonFXProDynamicMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle, DYNAMIC_MOTION_MAGIC_DUTY_CYCLE_NAME>;
template class TalonFXProDynamicMotionMagicControllerInterface<hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC, DYNAMIC_MOTION_MAGIC_TORQUE_CURRENT_FOC_NAME>;

template class TalonFXProFollowerControllerInterfaceBase<false>;
template class TalonFXProFollowerControllerInterfaceBase<true>;
} // namespace talonfxpro_controllers
