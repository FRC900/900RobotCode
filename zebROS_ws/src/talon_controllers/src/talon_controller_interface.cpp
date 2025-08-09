#ifdef __linux__
#include <sched.h>
#endif

#include "talon_controllers/TalonConfigConfig.h"
#include "talon_controllers/talon_controller_interface.h"
#include <XmlRpcValue.h>

namespace talon_controllers
{
const char PERCENT_OUTPUT_NAME[] = "Percent Output";
const char CLOSE_LOOP_POSITION_NAME[] = "Close Loop Position";
const char CLOSE_LOOP_VELOCITY_NAME[] = "Close Loop Velocity";
const char CLOSE_LOOP_CURRENT_NAME[] = "Close Loop Current";
const char MOTION_PROFILE_NAME[] = "Motion Profile";
const char MOTION_MAGIC_NAME[] = "Motion Magic";

TalonCIParams::TalonCIParams(void) :
    p_{0, 0, 0, 0},
    i_{0, 0, 0, 0},
    d_{0, 0, 0, 0},
    f_{0, 0, 0, 0},
    izone_{0, 0, 0, 0},
    allowable_closed_loop_error_{0, 0, 0, 0}, // need better defaults
    max_integral_accumulator_{0, 0, 0, 0},
    closed_loop_peak_output_{1, 1, 1, 1},
    closed_loop_period_{1, 1, 1, 1},
    pidf_slot_(0),
    aux_pid_polarity_(false),
    demand1_type_(hardware_interface::DemandType_Neutral),
    demand1_value_(0.),
    invert_output_(false),
    sensor_phase_(false),
    neutral_mode_(hardware_interface::NeutralMode_Coast),
    feedback_type_(hardware_interface::FeedbackDevice_Uninitialized),
    feedback_coefficient_(1.0),
    remote_feedback_type_(hardware_interface::RemoteFeedbackDevice_None),
    ticks_per_rotation_(4096),
    remote_feedback_device_ids_{0, 0},
    remote_feedback_filters_{hardware_interface::RemoteSensorSource_Off, hardware_interface::RemoteSensorSource_Off},
    sensor_terms_{hardware_interface::FeedbackDevice_QuadEncoder,hardware_interface::FeedbackDevice_QuadEncoder,hardware_interface::FeedbackDevice_QuadEncoder,hardware_interface::FeedbackDevice_QuadEncoder},
    closed_loop_ramp_(0.),
    open_loop_ramp_(0.),
    peak_output_forward_(1.),
    peak_output_reverse_(-1.),
    nominal_output_forward_(0.),
    nominal_output_reverse_(0.),
    neutral_deadband_(41.0 / 1023.0),
    voltage_compensation_saturation_(12),
    voltage_measurement_filter_(32),
    voltage_compensation_enable_(true),
    velocity_measurement_period_(hardware_interface::Period_100Ms),
    velocity_measurement_window_(64),
    limit_switch_local_forward_source_(hardware_interface::LimitSwitchSource_FeedbackConnector),
    limit_switch_local_forward_normal_(hardware_interface::LimitSwitchNormal_Disabled),
    limit_switch_local_reverse_source_(hardware_interface::LimitSwitchSource_FeedbackConnector),
    limit_switch_local_reverse_normal_(hardware_interface::LimitSwitchNormal_Disabled),
    limit_switch_remote_forward_source_(hardware_interface::RemoteLimitSwitchSource_Deactivated),
    limit_switch_remote_forward_normal_(hardware_interface::LimitSwitchNormal_Disabled),
    limit_switch_remote_forward_id_(0),
    limit_switch_remote_reverse_source_(hardware_interface::RemoteLimitSwitchSource_Deactivated),
    limit_switch_remote_reverse_normal_(hardware_interface::LimitSwitchNormal_Disabled),
    limit_switch_remote_reverse_id_(0),
    softlimit_forward_threshold_(0.0),
    softlimit_forward_enable_(false),
    softlimit_reverse_threshold_(0.0),
    softlimit_reverse_enable_(false),
    override_limit_switches_enable_(true),
    current_limit_peak_amps_(0),
    current_limit_peak_msec_(10), // to avoid errata - see https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/README.md#motor-output-direction-is-incorrect-or-accelerates-when-current-limit-is-enabled and https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B/Current%20Limit/src/Robot.cpp#L37
    current_limit_continuous_amps_(0),
    current_limit_enable_(false),
    // current limiting - TalonFX / Falcon500
    supply_current_limit_(0),
    supply_current_trigger_threshold_current_(0),
    supply_current_trigger_threshold_time_(0),
    supply_current_limit_enable_(false),

    motion_cruise_velocity_(0),
    motion_acceleration_(0),
    motion_s_curve_strength_(0),
    motion_profile_trajectory_period_(0),

    conversion_factor_(1.0),

    enable_read_thread_(true)
{
    status_frame_periods_[hardware_interface::Status_1_General] = hardware_interface::status_1_general_default;
    status_frame_periods_[hardware_interface::Status_2_Feedback0] = hardware_interface::status_2_feedback0_default;
    status_frame_periods_[hardware_interface::Status_3_Quadrature] = hardware_interface::status_3_quadrature_default;
    status_frame_periods_[hardware_interface::Status_4_AinTempVbat] = hardware_interface::status_4_aintempvbat_default;
    status_frame_periods_[hardware_interface::Status_6_Misc] = hardware_interface::status_6_misc_default;
    status_frame_periods_[hardware_interface::Status_7_CommStatus] = hardware_interface::status_7_commstatus_default;
    status_frame_periods_[hardware_interface::Status_8_PulseWidth] = hardware_interface::status_8_pulsewidth_default;
    status_frame_periods_[hardware_interface::Status_9_MotProfBuffer] = hardware_interface::status_9_motprofbuffer_default;
    status_frame_periods_[hardware_interface::Status_10_MotionMagic] = hardware_interface::status_10_motionmagic_default;
    status_frame_periods_[hardware_interface::Status_11_UartGadgeteer] = hardware_interface::status_11_uartgadgeteer_default;
    status_frame_periods_[hardware_interface::Status_12_Feedback1] = hardware_interface::status_12_feedback1_default;
    status_frame_periods_[hardware_interface::Status_13_Base_PIDF0] = hardware_interface::status_13_base_pidf0_default;
    status_frame_periods_[hardware_interface::Status_14_Turn_PIDF1] = hardware_interface::status_14_turn_pidf1_default;
    status_frame_periods_[hardware_interface::Status_15_FirmwareApiStatus] = hardware_interface::status_15_firmwareapistatus_default;
    status_frame_periods_[hardware_interface::Status_17_Targets1] = hardware_interface::status_17_targets1_default;
    status_frame_periods_[hardware_interface::Status_Brushless_Current] = hardware_interface::status_brushless_current_default;

    control_frame_periods_[hardware_interface::Control_3_General] = hardware_interface::control_3_general_default;
    control_frame_periods_[hardware_interface::Control_4_Advanced] = hardware_interface::control_4_advanced_default;
    control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride] = hardware_interface::control_5_feedbackoutputoverride_default;
    control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint] = hardware_interface::control_6_motprofaddtrajpoint_default;
}

// Update params set by a dynamic reconfig config
// Also pass in current params for ones which aren't
// dynamically reconfigurable - pass them through
// to the new one
TalonCIParams::TalonCIParams(const TalonCIParams &params, const TalonConfigConfig &config)
{
    // Default to passed in params, overwrite dynamic reconfig values
    // on top of them. This will preserve settings of things which aren't
    // dynamic reconfig (joint name, enable read thread, etc) and then update
    // the ones which are with updated dynamic reconfig values
    *this = params;
    p_[0] = config.p0;
    p_[1] = config.p1;
    p_[2] = config.p2;
    p_[3] = config.p3;
    i_[0] = config.i0;
    i_[1] = config.i1;
    i_[2] = config.i2;
    i_[3] = config.i3;
    d_[0] = config.d0;
    d_[1] = config.d1;
    d_[2] = config.d2;
    d_[3] = config.d3;
    f_[0] = config.f0;
    f_[1] = config.f1;
    f_[2] = config.f2;
    f_[3] = config.f3;
    izone_[0] = config.izone0;
    izone_[1] = config.izone1;
    izone_[2] = config.izone2;
    izone_[3] = config.izone3;
    allowable_closed_loop_error_[0] = config.allowable_closed_loop_error0;
    allowable_closed_loop_error_[1] = config.allowable_closed_loop_error1;
    allowable_closed_loop_error_[2] = config.allowable_closed_loop_error2;
    allowable_closed_loop_error_[3] = config.allowable_closed_loop_error3;
    max_integral_accumulator_[0] = config.max_integral_accumulator0;
    max_integral_accumulator_[1] = config.max_integral_accumulator1;
    max_integral_accumulator_[2] = config.max_integral_accumulator2;
    max_integral_accumulator_[3] = config.max_integral_accumulator3;
    closed_loop_peak_output_[0] = config.closed_loop_peak_output0;
    closed_loop_peak_output_[1] = config.closed_loop_peak_output1;
    closed_loop_peak_output_[2] = config.closed_loop_peak_output2;
    closed_loop_peak_output_[3] = config.closed_loop_peak_output3;
    closed_loop_period_[0] = config.closed_loop_period0;
    closed_loop_period_[1] = config.closed_loop_period1;
    closed_loop_period_[2] = config.closed_loop_period2;
    closed_loop_period_[3] = config.closed_loop_period3;
    pidf_slot_ = config.pid_config;
    aux_pid_polarity_ = config.aux_pid_polarity;
    demand1_type_ = static_cast<hardware_interface::DemandType>(config.demand1_type);
    demand1_value_ = config.demand1_value;
    invert_output_ = config.invert_output;

    sensor_phase_ = config.sensor_phase;
    feedback_type_ = static_cast<hardware_interface::FeedbackDevice>(config.feedback_type);
    feedback_coefficient_ = config.feedback_coefficient;
    remote_feedback_type_ = static_cast<hardware_interface::RemoteFeedbackDevice>(config.remote_feedback_type);
    remote_feedback_device_ids_[0] = config.remote_feedback_device_id0;
    remote_feedback_filters_[0] = static_cast<hardware_interface::RemoteSensorSource>(config.remote_feedback_filter0);
    remote_feedback_device_ids_[1] = config.remote_feedback_device_id1;
    remote_feedback_filters_[1] = static_cast<hardware_interface::RemoteSensorSource>(config.remote_feedback_filter1);
    sensor_terms_[hardware_interface::SensorTerm_Sum0] = static_cast<hardware_interface::FeedbackDevice>(config.sensor_term_sum0);
    sensor_terms_[hardware_interface::SensorTerm_Sum1] = static_cast<hardware_interface::FeedbackDevice>(config.sensor_term_sum1);
    sensor_terms_[hardware_interface::SensorTerm_Diff0] = static_cast<hardware_interface::FeedbackDevice>(config.sensor_term_diff0);
    sensor_terms_[hardware_interface::SensorTerm_Diff1] = static_cast<hardware_interface::FeedbackDevice>(config.sensor_term_diff1);
    ticks_per_rotation_ = config.encoder_ticks_per_rotation;
    neutral_mode_ = static_cast<hardware_interface::NeutralMode>(config.neutral_mode);
    closed_loop_ramp_ = config.closed_loop_ramp;
    open_loop_ramp_ = config.open_loop_ramp;
    peak_output_forward_ = config.peak_output_forward;
    peak_output_reverse_ = config.peak_output_reverse;
    nominal_output_forward_ = config.nominal_output_forward;
    nominal_output_reverse_ = config.nominal_output_reverse;
    neutral_deadband_ = config.neutral_deadband;
    voltage_compensation_saturation_ = config.voltage_compensation_saturation;
    voltage_measurement_filter_ = config.voltage_measurement_filter;
    voltage_compensation_enable_ = config.voltage_compensation_enable;
    velocity_measurement_period_ = static_cast<hardware_interface::VelocityMeasurementPeriod>(config.velocity_measurement_period);
    velocity_measurement_window_ = config.velocity_measurement_window;
    limit_switch_local_forward_source_ = static_cast<hardware_interface::LimitSwitchSource>(config.limit_switch_local_forward_source);
    limit_switch_local_forward_normal_ = static_cast<hardware_interface::LimitSwitchNormal>(config.limit_switch_local_forward_normal);
    limit_switch_local_reverse_source_ = static_cast<hardware_interface::LimitSwitchSource>(config.limit_switch_local_reverse_source);
    limit_switch_local_reverse_normal_ = static_cast<hardware_interface::LimitSwitchNormal>(config.limit_switch_local_reverse_normal);
    limit_switch_remote_forward_source_ = static_cast<hardware_interface::RemoteLimitSwitchSource>(config.limit_switch_remote_forward_source);
    limit_switch_remote_forward_normal_ = static_cast<hardware_interface::LimitSwitchNormal>(config.limit_switch_remote_forward_normal);
    limit_switch_remote_forward_id_     = config.limit_switch_remote_forward_id;
    limit_switch_remote_reverse_source_ = static_cast<hardware_interface::RemoteLimitSwitchSource>(config.limit_switch_remote_reverse_source);
    limit_switch_remote_reverse_normal_ = static_cast<hardware_interface::LimitSwitchNormal>(config.limit_switch_remote_reverse_normal);
    limit_switch_remote_reverse_id_     = config.limit_switch_remote_reverse_id;

    clear_position_on_limit_f_          = config.clear_position_on_limit_f;
    clear_position_on_limit_r_          = config.clear_position_on_limit_r;

    softlimit_forward_threshold_ = config.softlimit_forward_threshold;
    softlimit_forward_enable_ = config.softlimit_forward_enable;
    softlimit_reverse_threshold_ = config.softlimit_reverse_threshold;
    softlimit_reverse_enable_ = config.softlimit_reverse_enable;
    override_limit_switches_enable_ = config.softlimits_override_enable;

    current_limit_peak_amps_ = config.current_limit_peak_amps;
    current_limit_peak_msec_ = config.current_limit_peak_msec;
    current_limit_continuous_amps_ = config.current_limit_continuous_amps;
    current_limit_enable_ = config.current_limit_enable;

    supply_current_limit_ = config.supply_current_limit;
    supply_current_trigger_threshold_current_ = config.supply_current_trigger_threshold_current;
    supply_current_trigger_threshold_time_ = config.supply_current_trigger_threshold_time;
    supply_current_limit_enable_ = config.supply_current_limit_enable;

    motion_cruise_velocity_ = config.motion_cruise_velocity;
    motion_acceleration_ = config.motion_acceleration;
    motion_s_curve_strength_ = config.motion_s_curve_strength;
    motion_profile_trajectory_period_ = config.motion_profile_trajectory_period;

    status_frame_periods_[hardware_interface::Status_1_General] = config.status_1_general_period;
    status_frame_periods_[hardware_interface::Status_2_Feedback0] = config.status_2_feedback0_period;
    status_frame_periods_[hardware_interface::Status_3_Quadrature] = config.status_3_quadrature_period;
    status_frame_periods_[hardware_interface::Status_4_AinTempVbat] = config.status_4_aintempvbat_period;
    status_frame_periods_[hardware_interface::Status_6_Misc] = config.status_6_misc_period;
    status_frame_periods_[hardware_interface::Status_7_CommStatus] = config.status_7_commstatus_period;
    status_frame_periods_[hardware_interface::Status_8_PulseWidth] = config.status_8_pulsewidth_period;
    status_frame_periods_[hardware_interface::Status_9_MotProfBuffer] = config.status_9_motprofbuffer_period;
    status_frame_periods_[hardware_interface::Status_10_MotionMagic] = config.status_10_motionmagic_period;
    status_frame_periods_[hardware_interface::Status_11_UartGadgeteer] = config.status_11_uartgadgeteer_period;
    status_frame_periods_[hardware_interface::Status_12_Feedback1] = config.status_12_feedback1_period;
    status_frame_periods_[hardware_interface::Status_13_Base_PIDF0] = config.status_13_base_pidf0_period;
    status_frame_periods_[hardware_interface::Status_14_Turn_PIDF1] = config.status_14_turn_pidf1_period;
    status_frame_periods_[hardware_interface::Status_15_FirmwareApiStatus] = config.status_15_firmwareapistatus_period;
    status_frame_periods_[hardware_interface::Status_17_Targets1] = config.status_17_targets1_period;
    status_frame_periods_[hardware_interface::Status_Brushless_Current] = config.status_brushless_current_period;

    control_frame_periods_[hardware_interface::Control_3_General] = config.control_3_general_period;
    control_frame_periods_[hardware_interface::Control_4_Advanced] = config.control_4_advanced_period;
    control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride] = config.control_5_feedbackoutputoverride_period;
    control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint] = config.control_6_motprofaddtrajpoint_period;
    conversion_factor_ = config.conversion_factor;

}

// Copy from internal state to TalonConfigConfig state
TalonConfigConfig TalonCIParams::toConfig(void) const
{
    TalonConfigConfig config;
    config.p0            = p_[0];
    config.p1            = p_[1];
    config.p2            = p_[2];
    config.p3            = p_[3];
    config.i0            = i_[0];
    config.i1            = i_[1];
    config.i2            = i_[2];
    config.i3            = i_[3];
    config.d0            = d_[0];
    config.d1            = d_[1];
    config.d2            = d_[2];
    config.d3            = d_[3];
    config.f0            = f_[0];
    config.f1            = f_[1];
    config.f2            = f_[2];
    config.f3            = f_[3];
    config.izone0        = izone_[0];
    config.izone1        = izone_[1];
    config.izone2        = izone_[2];
    config.izone3        = izone_[3];
    config.allowable_closed_loop_error0 = allowable_closed_loop_error_[0];
    config.allowable_closed_loop_error1 = allowable_closed_loop_error_[1];
    config.allowable_closed_loop_error2 = allowable_closed_loop_error_[2];
    config.allowable_closed_loop_error3 = allowable_closed_loop_error_[3];
    config.max_integral_accumulator0 = max_integral_accumulator_[0];
    config.max_integral_accumulator1 = max_integral_accumulator_[1];
    config.max_integral_accumulator2 = max_integral_accumulator_[2];
    config.max_integral_accumulator3 = max_integral_accumulator_[3];
    config.closed_loop_peak_output0 = closed_loop_peak_output_[0];
    config.closed_loop_peak_output1 = closed_loop_peak_output_[1];
    config.closed_loop_peak_output2 = closed_loop_peak_output_[2];
    config.closed_loop_peak_output3 = closed_loop_peak_output_[3];
    config.closed_loop_period0 = closed_loop_period_[0];
    config.closed_loop_period1 = closed_loop_period_[1];
    config.closed_loop_period2 = closed_loop_period_[2];
    config.closed_loop_period3 = closed_loop_period_[3];
    config.pid_config    = pidf_slot_;
    config.aux_pid_polarity = aux_pid_polarity_;
    config.demand1_type = demand1_type_;
    config.demand1_value = demand1_value_;
    config.invert_output = invert_output_;
    config.sensor_phase  = sensor_phase_;
    config.feedback_type = feedback_type_;
    config.feedback_coefficient = feedback_coefficient_;
    config.remote_feedback_type = remote_feedback_type_;
    config.encoder_ticks_per_rotation = ticks_per_rotation_;
    config.remote_feedback_device_id0 = remote_feedback_device_ids_[0];
    config.remote_feedback_filter0 = remote_feedback_filters_[0];
    config.remote_feedback_device_id1 = remote_feedback_device_ids_[1];
    config.remote_feedback_filter1 = remote_feedback_filters_[1];
    config.sensor_term_sum0 = sensor_terms_[hardware_interface::SensorTerm_Sum0];
    config.sensor_term_sum1 = sensor_terms_[hardware_interface::SensorTerm_Sum1];
    config.sensor_term_diff0 = sensor_terms_[hardware_interface::SensorTerm_Diff0];
    config.sensor_term_diff1 = sensor_terms_[hardware_interface::SensorTerm_Diff1];
    config.neutral_mode  = neutral_mode_;
    config.closed_loop_ramp = closed_loop_ramp_;
    config.open_loop_ramp = open_loop_ramp_;
    config.peak_output_forward = peak_output_forward_;
    config.peak_output_reverse = peak_output_reverse_;
    config.nominal_output_forward = nominal_output_forward_;
    config.nominal_output_reverse = nominal_output_reverse_;
    config.neutral_deadband = neutral_deadband_;
    config.voltage_compensation_saturation = voltage_compensation_saturation_;
    config.voltage_measurement_filter = voltage_measurement_filter_;
    config.voltage_compensation_enable = voltage_compensation_enable_;
    config.velocity_measurement_period = velocity_measurement_period_;
    config.velocity_measurement_window = velocity_measurement_window_;
    config.limit_switch_local_forward_source = limit_switch_local_forward_source_;
    config.limit_switch_local_forward_normal = limit_switch_local_forward_normal_;
    config.limit_switch_local_reverse_source = limit_switch_local_reverse_source_;
    config.limit_switch_local_reverse_normal = limit_switch_local_reverse_normal_;
    config.limit_switch_remote_forward_source = limit_switch_remote_forward_source_;
    config.limit_switch_remote_forward_normal = limit_switch_remote_forward_normal_;
    config.limit_switch_remote_forward_id     = limit_switch_remote_forward_id_;
    config.limit_switch_remote_reverse_source = limit_switch_remote_reverse_source_;
    config.limit_switch_remote_reverse_normal = limit_switch_remote_reverse_normal_;
    config.limit_switch_remote_reverse_id     = limit_switch_remote_reverse_id_;
    config.clear_position_on_limit_f          = clear_position_on_limit_f_;
    config.clear_position_on_limit_r          = clear_position_on_limit_r_;
    config.softlimit_forward_threshold = softlimit_forward_threshold_;
    config.softlimit_forward_enable = softlimit_forward_enable_;
    config.softlimit_reverse_threshold = softlimit_reverse_threshold_;
    config.softlimit_reverse_enable = softlimit_reverse_enable_;
    config.softlimits_override_enable = override_limit_switches_enable_;
    config.current_limit_peak_amps = current_limit_peak_amps_;
    config.current_limit_peak_msec = current_limit_peak_msec_;
    config.current_limit_continuous_amps = current_limit_continuous_amps_;
    config.current_limit_enable = current_limit_enable_;
    config.supply_current_limit = supply_current_limit_;
    config.supply_current_trigger_threshold_current = supply_current_trigger_threshold_current_;
    config.supply_current_trigger_threshold_time = supply_current_trigger_threshold_time_;
    config.supply_current_limit_enable = supply_current_limit_enable_;
    config.motion_cruise_velocity = motion_cruise_velocity_;
    config.motion_acceleration = motion_acceleration_;
    config.motion_s_curve_strength = motion_s_curve_strength_;
    config.motion_profile_trajectory_period = motion_profile_trajectory_period_;

    config.status_1_general_period = status_frame_periods_[hardware_interface::Status_1_General];
    config.status_2_feedback0_period = status_frame_periods_[hardware_interface::Status_2_Feedback0];
    config.status_3_quadrature_period = status_frame_periods_[hardware_interface::Status_3_Quadrature];
    config.status_4_aintempvbat_period = status_frame_periods_[hardware_interface::Status_4_AinTempVbat];
    config.status_6_misc_period = status_frame_periods_[hardware_interface::Status_6_Misc];
    config.status_7_commstatus_period = status_frame_periods_[hardware_interface::Status_7_CommStatus];
    config.status_8_pulsewidth_period = status_frame_periods_[hardware_interface::Status_8_PulseWidth];
    config.status_9_motprofbuffer_period = status_frame_periods_[hardware_interface::Status_9_MotProfBuffer];
    config.status_10_motionmagic_period = status_frame_periods_[hardware_interface::Status_10_MotionMagic];
    config.status_11_uartgadgeteer_period = status_frame_periods_[hardware_interface::Status_11_UartGadgeteer];
    config.status_12_feedback1_period = status_frame_periods_[hardware_interface::Status_12_Feedback1];
    config.status_13_base_pidf0_period = status_frame_periods_[hardware_interface::Status_13_Base_PIDF0];
    config.status_14_turn_pidf1_period = status_frame_periods_[hardware_interface::Status_14_Turn_PIDF1];
    config.status_15_firmwareapistatus_period = status_frame_periods_[hardware_interface::Status_15_FirmwareApiStatus];
    config.status_17_targets1_period = status_frame_periods_[hardware_interface::Status_17_Targets1];
    config.status_brushless_current_period = status_frame_periods_[hardware_interface::Status_Brushless_Current];

    config.control_3_general_period = control_frame_periods_[hardware_interface::Control_3_General];
    config.control_4_advanced_period = control_frame_periods_[hardware_interface::Control_4_Advanced];
    config.control_5_feedbackoutputoverride_period = control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride];
    config.control_6_motprofaddtrajpoint_period = control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint];

    config.conversion_factor = conversion_factor_;
    return config;
}

// Read a joint name from the given nodehandle's params
bool TalonCIParams::readJointName(const ros::NodeHandle &n)
{
    if (!n.getParam("joint", joint_name_))
    {
        ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
        return false;
    }
    return true;
}

bool TalonCIParams::readConversion(const ros::NodeHandle &n)
{
    n.getParam("conversion_factor", conversion_factor_);
    return true;
}

// Read a joint name from the given nodehandle's params
bool TalonCIParams::readNeutralMode(const ros::NodeHandle &n)
{
    if (std::string mode_string; n.getParam("neutral_mode", mode_string))
    {
        if (mode_string == "EEPROM")
            neutral_mode_ = hardware_interface::NeutralMode_EEPROM_Setting;
        else if (mode_string == "Coast")
            neutral_mode_ = hardware_interface::NeutralMode_Coast;
        else if (mode_string == "Brake")
            neutral_mode_ = hardware_interface::NeutralMode_Brake;
        else
        {
            ROS_ERROR("Invalid neutral mode name (namespace: %s, %s)",
                      n.getNamespace().c_str(), mode_string.c_str());
            return false;
        }
    }
    return true;
}
// TODO: create a method that reads the feedback settings enum
bool TalonCIParams::readFeedbackType(const ros::NodeHandle &n)
{
    std::string str;
    if (n.getParam("feedback_type", str) && !stringToFeedbackDevice(str, feedback_type_))
    {
        ROS_ERROR_STREAM("Invalid feedback device name given : " << str);
        return false;
    }

    n.getParam("ticks_per_rotation", ticks_per_rotation_);
    n.getParam("feedback_coefficient", feedback_coefficient_);
    if (n.getParam("remote_feedback_type", str))
    {
        if (str == "SensorSum")
            remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_SensorSum;
        else if (str == "SensorDifference")
            remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_SensorDifference;
        else if (str == "RemoteSensor0")
            remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_RemoteSensor0;
        else if (str == "RemoteSensor1")
            remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_RemoteSensor1;
        else if (str == "None")
            remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_None;
        else if (str == "SoftwareEmulatedSensor")
            remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_SoftwareEmulatedSensor;
        else
        {
            ROS_ERROR_STREAM("Invalid remote feedback device name given : " << str);
            return false;
        }
    }
    n.getParam("remote_feedback_device_id0", remote_feedback_device_ids_[0]);
    if (remote_feedback_device_ids_[0] > 15)
    {
        ROS_ERROR_STREAM("remote_feedback_device_id0 must be less than 15");
        return false;
    }
    if (n.getParam("remote_feedback_filter0", str) &&
        !stringToRemoteSensorSource(str, remote_feedback_filters_[0]))
    {
        ROS_ERROR_STREAM("Invalid remote_feedback_filter0 device name given : " << str);
        return false;
    }
    n.getParam("remote_feedback_device_id1", remote_feedback_device_ids_[1]);
    if (remote_feedback_device_ids_[1] > 15)
    {
        ROS_ERROR_STREAM("remote_feedback_device_id1 must be less than 15");
        return false;
    }
    if (n.getParam("remote_feedback_filter1", str) &&
        stringToRemoteSensorSource(str, remote_feedback_filters_[1]))
    {
        ROS_ERROR_STREAM("Invalid remote_feedback_filter1 device name given : " << str);
        return false;
    }

    if (n.getParam("sensor_term_sum0", str) &&
        !stringToFeedbackDevice(str, sensor_terms_[hardware_interface::SensorTerm_Sum0]))
    {
        ROS_ERROR_STREAM("Invalid sensor_term_sum0 device name given : " << str);
        return false;
    }
    if (n.getParam("sensor_term_sum1", str) &&
        !stringToFeedbackDevice(str, sensor_terms_[hardware_interface::SensorTerm_Sum1]))
    {
        ROS_ERROR_STREAM("Invalid sensor_term_sum1 device name given : " << str);
        return false;
    }
    if (n.getParam("sensor_term_diff0", str) &&
        !stringToFeedbackDevice(str, sensor_terms_[hardware_interface::SensorTerm_Diff0]))
    {
        ROS_ERROR_STREAM("Invalid sensor_term_diff0 device name given : " << str);
        return false;
    }
    if (n.getParam("sensor_term_diff1", str) &&
        !stringToFeedbackDevice(str, sensor_terms_[hardware_interface::SensorTerm_Diff1]))
    {
        ROS_ERROR_STREAM("Invalid sensor_term_diff1 device name given : " << str);
        return false;
    }

    return true;
}

bool TalonCIParams::readInverts(const ros::NodeHandle &n)
{
    n.getParam("invert_output", invert_output_);
    n.getParam("sensor_phase", sensor_phase_);
    return true;
}

bool TalonCIParams::readCloseLoopParams(const ros::NodeHandle &n)
{
    XmlRpc::XmlRpcValue pid_param_list;

    n.getParam("aux_pid_polarity", aux_pid_polarity_);
    std::string demand1_mode_string;
    if (n.getParam("demand1_mode", demand1_mode_string))
    {
        if (demand1_mode_string == "Neutral")
            demand1_type_ = hardware_interface::DemandType_Neutral;
        else if (demand1_mode_string == "AuxPID")
            demand1_type_ = hardware_interface::DemandType_AuxPID;
        else if (demand1_mode_string == "ArbitraryFeedForward")
            demand1_type_ = hardware_interface::DemandType_ArbitraryFeedForward;
        else
        {
            ROS_ERROR("Invalid demand1 mode type (namespace: %s) %s",
                      n.getNamespace().c_str(), demand1_mode_string.c_str());
            return false;
        }
    }
    n.getParam("demand1_value", demand1_value_);
    if (!n.getParam("close_loop_values", pid_param_list))
        return true;
    if (pid_param_list.size() <= static_cast<int>(hardware_interface::TALON_PIDF_SLOTS))
    {
        for (int i = 0; i < pid_param_list.size(); i++)
        {
            XmlRpc::XmlRpcValue &pidparams = pid_param_list[i];

            findFloatParam("p", pidparams, p_[i]);
            findFloatParam("i", pidparams, i_[i]);
            findFloatParam("d", pidparams, d_[i]);
            findFloatParam("f", pidparams, f_[i]);
            findFloatParam("i_zone", pidparams, izone_[i]);
            findFloatParam("allowable_closed_loop_error", pidparams, allowable_closed_loop_error_[i]);
            findFloatParam("max_integral_accumulator", pidparams, max_integral_accumulator_[i]);
            findFloatParam("closed_loop_peak_output", pidparams, closed_loop_peak_output_[i]);
            findIntParam("closed_loop_period", pidparams, closed_loop_period_[i]);
        }
        return true;
    }
    else
    {
        throw std::runtime_error("Too many pid_param values");
    }
    return false;
}

bool TalonCIParams::readOutputShaping(const ros::NodeHandle &n)
{
    n.getParam("closed_loop_ramp", closed_loop_ramp_);
    n.getParam("open_loop_ramp", open_loop_ramp_);
    n.getParam("peak_output_forward", peak_output_forward_);
    n.getParam("peak_output_reverse", peak_output_reverse_);
    n.getParam("nominal_output_forward", nominal_output_forward_);
    n.getParam("nominal_output_reverse", nominal_output_reverse_);
    n.getParam("neutral_deadband", neutral_deadband_);
    return true;
}
bool TalonCIParams::readVoltageCompensation(const ros::NodeHandle &n)
{
    bool saturation_read = false;
    if (n.getParam("voltage_compensation_saturation", voltage_compensation_saturation_))
    {
        saturation_read = true;
    }
    if (n.getParam("voltage_measurement_filter", voltage_measurement_filter_) &&
        n.getParam("voltage_compensation_enable", voltage_compensation_enable_) &&
        voltage_compensation_enable_ && !saturation_read)
    {
        ROS_WARN_STREAM("Using default voltage_compensation_saturation for " << joint_name_ << " since value not set in config");
    }
    return true;
}

bool TalonCIParams::readVelocitySignalConditioning(const ros::NodeHandle &n)
{
    std::string str_val;
    n.getParam("velocity_measurement_period", str_val);
    if (str_val != "")
    {
        if (str_val == "Period_1Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_1Ms;
        else if (str_val == "Period_2Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_2Ms;
        else if (str_val == "Period_5Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_5Ms;
        else if (str_val == "Period_10Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_10Ms;
        else if (str_val == "Period_20Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_20Ms;
        else if (str_val == "Period_25Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_25Ms;
        else if (str_val == "Period_50Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_50Ms;
        else if (str_val == "Period_100Ms")
            velocity_measurement_period_ = hardware_interface::VelocityMeasurementPeriod::Period_100Ms;
        else
        {
            ROS_ERROR_STREAM("Invalid velocity_measurement_period (" << str_val << ")");
            return false;
        }
    }

    n.getParam("velocity_measurement_window", velocity_measurement_window_);
    return true;
};

bool TalonCIParams::readLimitSwitches(const ros::NodeHandle &n)
{
    std::string str_val;
    hardware_interface::LimitSwitchSource limit_switch_source;
    hardware_interface::LimitSwitchNormal limit_switch_normal;
    if (n.getParam("limit_switch_local_forward_source", str_val))
    {
        if (!stringToLimitSwitchSource(str_val, limit_switch_source))
            return false;
        limit_switch_local_forward_source_ = limit_switch_source;
    }
    if (n.getParam("limit_switch_local_forward_normal", str_val))
    {
        if (!stringToLimitSwitchNormal(str_val, limit_switch_normal))
            return false;
        limit_switch_local_forward_normal_ = limit_switch_normal;
    }
    if (n.getParam("limit_switch_local_reverse_source", str_val))
    {
        if (!stringToLimitSwitchSource(str_val, limit_switch_source))
            return false;
        limit_switch_local_reverse_source_ = limit_switch_source;
    }
    if (n.getParam("limit_switch_local_reverse_normal", str_val))
    {
        if (!stringToLimitSwitchNormal(str_val, limit_switch_normal))
            return false;
        limit_switch_local_reverse_normal_ = limit_switch_normal;
    }
    hardware_interface::RemoteLimitSwitchSource remote_limit_switch_source;
    if (n.getParam("limit_switch_remote_forward_source", str_val))
    {
        if (!stringToRemoteLimitSwitchSource(str_val, remote_limit_switch_source))
            return false;
        limit_switch_remote_forward_source_ = remote_limit_switch_source;
        int id;
        if (n.getParam("limit_switch_remote_forward_id", id))
        {
            limit_switch_remote_forward_id_ = static_cast<unsigned int>(id);
        }
        else if (limit_switch_remote_forward_source_ != hardware_interface::RemoteLimitSwitchSource_Deactivated)
        {
            ROS_ERROR("Remote limt switch forward id must be set if source is not \"Disabled\"");
            return false;
        }
    }
    if (n.getParam("limit_switch_remote_forward_normal", str_val))
    {
        if (!stringToLimitSwitchNormal(str_val, limit_switch_normal))
            return false;
        limit_switch_remote_forward_normal_ = limit_switch_normal;
    }
    if (n.getParam("limit_switch_remote_reverse_source", str_val))
    {
        if (!stringToRemoteLimitSwitchSource(str_val, remote_limit_switch_source))
            return false;
        limit_switch_remote_reverse_source_ = remote_limit_switch_source;
        int id;
        if (n.getParam("limit_switch_remote_reverse_id", id))
        {
            limit_switch_remote_reverse_id_ = static_cast<unsigned int>(id);
        }
        else if (limit_switch_remote_reverse_source_ != hardware_interface::RemoteLimitSwitchSource_Deactivated)
        {
            ROS_ERROR("Remote limt switch reverse id must be set if source is not \"Disabled\"");
            return false;
        }
    }
    if (n.getParam("limit_switch_remote_reverse_normal", str_val))
    {
        if (!stringToLimitSwitchNormal(str_val, limit_switch_normal))
            return false;
        limit_switch_remote_reverse_normal_ = limit_switch_normal;
    }
    n.getParam("clear_position_on_limit_f", clear_position_on_limit_f_);
    n.getParam("clear_position_on_limit_r", clear_position_on_limit_r_);
    return true;
}

bool TalonCIParams::readSoftLimits(const ros::NodeHandle &n)
{
    int param_count = 0;
    if (n.getParam("softlimit_forward_threshold", softlimit_forward_threshold_))
        param_count = 1;
    if (n.getParam("softlimit_forward_enable", softlimit_forward_enable_) &&
        softlimit_forward_enable_ && (param_count == 0))
        ROS_WARN_STREAM("Enabling forward softlimits for " << joint_name_ << " without setting threshold");
    param_count = 0;
    if (n.getParam("softlimit_reverse_threshold", softlimit_reverse_threshold_))
        param_count = 1;
    if (n.getParam("softlimit_reverse_enable", softlimit_reverse_enable_) &&
        softlimit_reverse_enable_ && (param_count == 0))
        ROS_WARN_STREAM("Enabling forward softlimits for " << joint_name_ << " without setting threshold");
    n.getParam("override_limit_switches_enable", override_limit_switches_enable_);
    return true;
}

bool TalonCIParams::readCurrentLimits(const ros::NodeHandle &n)
{
    size_t params_read = 0;
    if (n.getParam("current_limit_peak_amps", current_limit_peak_amps_))
        params_read += 1;
    if (n.getParam("current_limit_peak_msec", current_limit_peak_msec_))
        params_read += 1;
    if (n.getParam("current_limit_continuous_amps", current_limit_continuous_amps_))
        params_read += 1;
    if (n.getParam("current_limit_enable", current_limit_enable_) &&
        current_limit_enable_ && (params_read < 3))
        ROS_WARN_STREAM("Not all current limits set for " << joint_name_ << " before enabling - using defaults might not work as expected");
    return true;
}

bool TalonCIParams::readSupplyCurrentLimits(const ros::NodeHandle &n)
{
    int params_read = 0;
    if (n.getParam("supply_current_limit", supply_current_limit_))
        params_read += 1;
    if (n.getParam("supply_current_trigger_threshold_current", supply_current_trigger_threshold_current_))
        params_read += 1;
    if (n.getParam("supply_current_trigger_threshold_time", supply_current_trigger_threshold_time_))
        params_read += 1;
    if (n.getParam("supply_current_limit_enable", supply_current_limit_enable_) &&
        supply_current_limit_enable_ && (params_read < 3))
        ROS_WARN_STREAM("Not all supply current limits set for " << joint_name_
                                                                 << " before enabling - using defaults might not work as expected");
    return true;
}

bool TalonCIParams::readMotionControl(const ros::NodeHandle &n)
{
    n.getParam("motion_cruise_velocity", motion_cruise_velocity_);
    n.getParam("motion_acceleration", motion_acceleration_);
    n.getParam("motion_s_curve_strength", motion_s_curve_strength_);
    n.getParam("motion_profile_trajectory_period", motion_profile_trajectory_period_);
    return true;
}

bool TalonCIParams::readStatusFramePeriods(const ros::NodeHandle &n)
{
    n.getParam("status_1_general_period", status_frame_periods_[hardware_interface::Status_1_General]);
    n.getParam("status_2_feedback0_period", status_frame_periods_[hardware_interface::Status_2_Feedback0]);
    n.getParam("status_3_quadrature_period", status_frame_periods_[hardware_interface::Status_3_Quadrature]);
    n.getParam("status_4_aintempvbat_period", status_frame_periods_[hardware_interface::Status_4_AinTempVbat]);
    n.getParam("status_6_misc_period", status_frame_periods_[hardware_interface::Status_6_Misc]);
    n.getParam("status_7_commstatus_period", status_frame_periods_[hardware_interface::Status_7_CommStatus]);
    n.getParam("status_8_pulsewidth_period", status_frame_periods_[hardware_interface::Status_8_PulseWidth]);
    n.getParam("status_9_motprofbuffer_period", status_frame_periods_[hardware_interface::Status_9_MotProfBuffer]);
    n.getParam("status_10_motionmagic_period", status_frame_periods_[hardware_interface::Status_10_MotionMagic]);
    n.getParam("status_11_uartgadgeteer_period", status_frame_periods_[hardware_interface::Status_11_UartGadgeteer]);
    n.getParam("status_12_feedback1_period", status_frame_periods_[hardware_interface::Status_12_Feedback1]);
    n.getParam("status_13_base_pidf0_period", status_frame_periods_[hardware_interface::Status_13_Base_PIDF0]);
    n.getParam("status_14_turn_pidf1_period", status_frame_periods_[hardware_interface::Status_14_Turn_PIDF1]);
    n.getParam("status_15_firmwareapistatus_period", status_frame_periods_[hardware_interface::Status_15_FirmwareApiStatus]);
    n.getParam("status_17_targets1_period", status_frame_periods_[hardware_interface::Status_17_Targets1]);
    n.getParam("status_17_brushless_current_period", status_frame_periods_[hardware_interface::Status_Brushless_Current]);
    return true;
}

bool TalonCIParams::readControlFramePeriods(const ros::NodeHandle &n)
{
    n.getParam("control_3_general_period", control_frame_periods_[hardware_interface::Control_3_General]);
    n.getParam("control_4_advanced_period", control_frame_periods_[hardware_interface::Control_4_Advanced]);
    n.getParam("control_5_feedbackoutputoverride_period", control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride]);
    n.getParam("control_6_motprofaddtrajpoint_period", control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint]);

    return true;
}

bool TalonCIParams::readTalonThread(const ros::NodeHandle &n)
{
    n.getParam("enable_read_thread", enable_read_thread_);
    return true;
}

// Read a double named <param_type> from the array/map
// in params
bool TalonCIParams::findFloatParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, double &val) const
{
    if (!params.hasMember(param_name))
        return false;
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
        throw std::runtime_error(param_name + " was not a valid double type");
    if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        val = (double)param;
        return true;
    }
    else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        val = (int)param;
        return true;
    }
    else
        throw std::runtime_error("A non-double value was passed for " + param_name);

    return false;
}

// Read an integer named <param_name> from the array/map
// in params
bool TalonCIParams::findIntParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, int &val) const
{
    if (!params.hasMember(param_name))
        return false;
    XmlRpc::XmlRpcValue &param = params[param_name];
    if (!param.valid())
        throw std::runtime_error(param_name + " was not a valid int type");
    if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
        val = (int)param;
    else
        throw std::runtime_error("A non-int value was passed for " + param_name);
    return false;
}

bool TalonCIParams::stringToLimitSwitchSource(const std::string &str,
                                              hardware_interface::LimitSwitchSource &limit_switch_source) const
{
    if (str == "FeedbackConnector")
        limit_switch_source = hardware_interface::LimitSwitchSource_FeedbackConnector;
    else if (str == "RemoteTalonSRX")
        limit_switch_source = hardware_interface::LimitSwitchSource_RemoteTalonSRX;
    else if (str == "RemoteCANifier")
        limit_switch_source = hardware_interface::LimitSwitchSource_RemoteCANifier;
    else if (str == "Deactivated")
        limit_switch_source = hardware_interface::LimitSwitchSource_Deactivated;
    else
    {
        ROS_ERROR_STREAM("Invalid limit switch source : " << str);
        return false;
    }
    return true;
}

bool TalonCIParams::stringToRemoteLimitSwitchSource(const std::string &str,
                                                    hardware_interface::RemoteLimitSwitchSource &limit_switch_source) const
{
    if (str == "RemoteTalonSRX")
        limit_switch_source = hardware_interface::RemoteLimitSwitchSource_RemoteTalonSRX;
    else if (str == "RemoteCANifier")
        limit_switch_source = hardware_interface::RemoteLimitSwitchSource_RemoteCANifier;
    else if (str == "Deactivated")
        limit_switch_source = hardware_interface::RemoteLimitSwitchSource_Deactivated;
    else
    {
        ROS_ERROR_STREAM("Invalid remote limit switch source : " << str);
        return false;
    }
    return true;
}

bool TalonCIParams::stringToLimitSwitchNormal(const std::string &str,
                                              hardware_interface::LimitSwitchNormal &limit_switch_normal) const
{
    if (str == "NormallyOpen")
        limit_switch_normal = hardware_interface::LimitSwitchNormal_NormallyOpen;
    else if (str == "NormallyClosed")
        limit_switch_normal = hardware_interface::LimitSwitchNormal_NormallyClosed;
    else if (str == "Disabled")
        limit_switch_normal = hardware_interface::LimitSwitchNormal_Disabled;
    else
    {
        ROS_ERROR_STREAM("Invalid limit switch normal : " << str);
        return false;
    }
    return true;
}

bool TalonCIParams::stringToFeedbackDevice(const std::string &str,
                                           hardware_interface::FeedbackDevice &feedback_device) const
{
    if (str == "QuadEncoder")
        feedback_device = hardware_interface::FeedbackDevice_QuadEncoder;
    else if (str == "Analog")
        feedback_device = hardware_interface::FeedbackDevice_Analog;
    else if (str == "Tachometer")
        feedback_device = hardware_interface::FeedbackDevice_Tachometer;
    else if (str == "PulseWidthEncodedPosition")
        feedback_device = hardware_interface::FeedbackDevice_PulseWidthEncodedPosition;
    else if (str == "SensorSum")
        feedback_device = hardware_interface::FeedbackDevice_SensorSum;
    else if (str == "SensorDifference")
        feedback_device = hardware_interface::FeedbackDevice_SensorDifference;
    else if (str == "RemoteSensor0")
        feedback_device = hardware_interface::FeedbackDevice_RemoteSensor0;
    else if (str == "RemoteSensor1")
        feedback_device = hardware_interface::FeedbackDevice_RemoteSensor1;
    else if (str == "None")
        feedback_device = hardware_interface::FeedbackDevice_None;
    else if (str == "SoftwareEmulatedSensor")
        feedback_device = hardware_interface::FeedbackDevice_SoftwareEmulatedSensor;
    else if (str == "CTRE_MagEncoder_Absolute")
        feedback_device = hardware_interface::FeedbackDevice_CTRE_MagEncoder_Absolute;
    else if (str == "CTRE_MagEncoder_Relative")
        feedback_device = hardware_interface::FeedbackDevice_CTRE_MagEncoder_Relative;
    else
    {
        ROS_ERROR_STREAM("Invalid feedback device name given : " << str);
        return false;
    }
    return true;
}

bool TalonCIParams::stringToRemoteSensorSource(const std::string &str,
                                               hardware_interface::RemoteSensorSource &source) const
{
    if (str == "Off")
        source = hardware_interface::RemoteSensorSource_Off;
    else if (str == "TalonSRX_SelectedSensor")
        source = hardware_interface::RemoteSensorSource_TalonSRX_SelectedSensor;
    else if (str == "Pigeon_Yaw")
        source = hardware_interface::RemoteSensorSource_Pigeon_Yaw;
    else if (str == "Pigeon_Pitch")
        source = hardware_interface::RemoteSensorSource_Pigeon_Pitch;
    else if (str == "Pigeon_Roll")
        source = hardware_interface::RemoteSensorSource_Pigeon_Roll;
    else if (str == "CANifier_Quadrature")
        source = hardware_interface::RemoteSensorSource_CANifier_Quadrature;
    else if (str == "CANifier_PWMInput0")
        source = hardware_interface::RemoteSensorSource_CANifier_PWMInput0;
    else if (str == "CANifier_PWMInput1")
        source = hardware_interface::RemoteSensorSource_CANifier_PWMInput1;
    else if (str == "CANifier_PWMInput2")
        source = hardware_interface::RemoteSensorSource_CANifier_PWMInput2;
    else if (str == "CANifier_PWMInput3")
        source = hardware_interface::RemoteSensorSource_CANifier_PWMInput3;
    else if (str == "GadgeteerPigeon_Yaw")
        source = hardware_interface::RemoteSensorSource_GadgeteerPigeon_Yaw;
    else if (str == "GadgeteerPigeon_Pitch")
        source = hardware_interface::RemoteSensorSource_GadgeteerPigeon_Pitch;
    else if (str == "GadgeteerPigeon_Roll")
        source = hardware_interface::RemoteSensorSource_GadgeteerPigeon_Roll;
    else if (str == "CANCoder")
        source = hardware_interface::RemoteSensorSource_CANCoder;
    else
    {
        ROS_ERROR_STREAM("Invalid remote sensor source filter type : " << str);
        return false;
    }
    return true;
}


TalonControllerInterface::TalonControllerInterface(void)
    : srv_mutex_{std::make_shared<boost::recursive_mutex>()}
    , srv_update_thread_flag_{}
    , srv_update_thread_active_{false}
    , srv_update_thread_{}
{
    srv_update_thread_flag_.test_and_set();
}

TalonControllerInterface::~TalonControllerInterface()
{
    srv_update_thread_active_ = false;
}

// Standardize format for reading params for
// motor controller
bool TalonControllerInterface::readParams(ros::NodeHandle &n, TalonCIParams &params)
{
    return params.readJointName(n) &&
            params.readConversion(n) &&
            params.readCloseLoopParams(n) &&
            params.readNeutralMode(n) &&
            params.readInverts(n) &&
            params.readFeedbackType(n) &&
            params.readOutputShaping(n) &&
            params.readVoltageCompensation(n) &&
            params.readVelocitySignalConditioning(n) &&
            params.readLimitSwitches(n) &&
            params.readSoftLimits(n) &&
            params.readCurrentLimits(n) &&
            params.readSupplyCurrentLimits(n) &&
            params.readMotionControl(n) &&
            params.readStatusFramePeriods(n) &&
            params.readControlFramePeriods(n) &&
            params.readTalonThread(n);
}

// Read params from config file and use them to
// initialize the Talon hardware
bool TalonControllerInterface::initWithNode(hardware_interface::TalonCommandInterface *tci,
                                            hardware_interface::TalonStateInterface * /*tsi*/,
                                            ros::NodeHandle &n)
{
    return init(tci, n, talon_, srv_mutex_, false) &&
            setInitialMode();
}

// Same as above, except pass in an array
// of node handles. First entry is set up as the leader
// talon and the rest are set in follower mode to follow
// the leader
bool TalonControllerInterface::initWithNode(hardware_interface::TalonCommandInterface *tci,
                                            hardware_interface::TalonStateInterface *tsi,
                                            std::vector<ros::NodeHandle> &n)
{
    if (!initWithNode(tci, tsi, n[0]))
        return false;

    const int follow_can_id = talon_.state()->getCANID();

    // If more than 1 joints are passed in, everything
    // but the first is to be set up as a follower
    follower_talons_.resize(n.size() - 1);
    for (size_t i = 1; i < n.size(); i++)
    {
        if (!init(tci, n[i], follower_talons_[i-1], nullptr, true))
            return false;
        follower_talons_[i-1]->setMode(hardware_interface::TalonMode_Follower);
        follower_talons_[i-1]->set(follow_can_id);
        ROS_INFO_STREAM("Set up talon " << follower_talons_[i-1].getName() << " to follow CAN ID " << follow_can_id << " (" << talon_.getName() << ")");
    }

    return true;
}

// Init with XmlRpcValue instead of NodeHandle - XmlRpcValue
// will be either a string or an array of strings of joints
// to load
bool TalonControllerInterface::initWithNode(hardware_interface::TalonCommandInterface *tci,
                                            hardware_interface::TalonStateInterface *tsi,
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

        for (int i = 0; i < param.size(); ++i)
        {
            joint_nodes.emplace_back(controller_nh, static_cast<std::string>(param[i]));
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

bool TalonControllerInterface::initWithNode(hardware_interface::TalonCommandInterface *tci,
                                            hardware_interface::TalonStateInterface *tsi,
                                            ros::NodeHandle &controller_nh,
                                            const std::string &talon_name)
                                            
{
    return initWithNode(tci, tsi, controller_nh, XmlRpc::XmlRpcValue{talon_name});
}

bool TalonControllerInterface::initWithNode(hardware_interface::TalonCommandInterface *tci,
                                            hardware_interface::TalonStateInterface *tsi,
                                            ros::NodeHandle &controller_nh,
                                            const char *talon_name)
                                            
{
    return initWithNode(tci, tsi, controller_nh, XmlRpc::XmlRpcValue{talon_name});
}

#if 0
// Allow users of the ControllerInterface to get
// a copy of the parameters currently set for the
// Talon.  They can then modify them at will and
// call initWithParams to reprogram the Talon.
// Hopefully this won't be needed all that often ...
// the goal should be to provide a simpler interface
// for commonly used operations
TalonCIParams TalonControllerInterface::getParams(void) const
{
    return params_;
}

// Initialize Talon hardware with the settings in params
bool TalonControllerInterface::initWithParams(hardware_interface::TalonCommandInterface *tci,
                                              const TalonCIParams &params)
{
    talon_ = tci->getHandle(params.joint_name_);
    return writeParamsToHW(params);
}
#endif

void TalonControllerInterface::callback(talon_controllers::TalonConfigConfig &config, uint32_t /*level*/)
{
    // TODO : this list is rapidly getting out of date.
    // Update it or remove the printout?
    ROS_INFO("Reconfigure request : %s %f %f %f %f %f %f %f %f %f %f %d %d",
                talon_.getName().c_str(),
                config.p0,
                config.p1,
                config.i0,
                config.i1,
                config.d0,
                config.d1,
                config.f0,
                config.f1,
                config.izone0,
                config.izone1,
                config.invert_output,
                config.sensor_phase);

    writeParamsToHW(TalonCIParams(params_, config), talon_);
}

// Set the setpoint for the motor controller
void TalonControllerInterface::setCommand(const double command)
{
    talon_->set(command);
}

// Set the mode of the motor controller
void TalonControllerInterface::setMode(hardware_interface::TalonMode mode)
{
    talon_->setMode(mode);
}

// Set the mode of the motor controller
hardware_interface::TalonMode TalonControllerInterface::getMode(void) const
{
    return TalonControllerInterface::talon_.state()->getTalonMode();
}


// Set the mode of the motor controller
void TalonControllerInterface::setNeutralMode(hardware_interface::NeutralMode neutral_mode)
{
    if (neutral_mode == params_.neutral_mode_)
        return;
    params_.neutral_mode_ = neutral_mode;
    syncDynamicReconfigure();
    talon_->setNeutralMode(neutral_mode);
}

// Pick the config slot (0 or 1) for PIDF/IZone values
bool TalonControllerInterface::setPIDFSlot(int slot)
{
    if ((slot < 0) || (slot > 3))
    {
        //ROS_WARN_STREAM("controller set of PID slot:  (false): " << slot);
        return false;
    }
    if (slot == params_.pidf_slot_)
    {
        //ROS_WARN_STREAM("controller set of PID slot:  (true): " << slot);

        talon_->setPidfSlot(slot);
        return true;
    }
    params_.pidf_slot_ = slot;

    // If dynamic reconfigure is running update
    // the reported config there with the new internal
    // state
    syncDynamicReconfigure();

    talon_->setPidfSlot(params_.pidf_slot_);
    return true;
}

bool TalonControllerInterface::setP(double newP, int slot)
{
    if ((slot < 0) || (slot > 3))
    {
        //ROS_WARN_STREAM("controller set of PID slot:  (false): " << slot);
        return false;
    }
    syncDynamicReconfigure();

    talon_->setP(newP, slot);
    return true;
}

void TalonControllerInterface::setIntegralAccumulator(double iaccum)
{
    talon_->setIntegralAccumulator(iaccum);
}

void TalonControllerInterface::setOverrideSoftLimitsEnable(bool enable)
{
    if (enable == params_.override_limit_switches_enable_)
        return;
    params_.override_limit_switches_enable_ = enable;
    syncDynamicReconfigure();
    talon_->setOverrideSoftLimitsEnable(enable);
}

void TalonControllerInterface::setPeakCurrentLimit(int amps)
{
    if (amps == params_.current_limit_peak_amps_)
        return;
    params_.current_limit_peak_amps_ = amps;

    syncDynamicReconfigure();

    talon_->setPeakCurrentLimit(params_.current_limit_peak_amps_);
}

void TalonControllerInterface::setPeakCurrentDuration(int msec)
{
    if (msec == params_.current_limit_peak_msec_)
        return;
    params_.current_limit_peak_msec_ = msec;

    syncDynamicReconfigure();

    talon_->setPeakCurrentDuration(params_.current_limit_peak_msec_);
}

void TalonControllerInterface::setContinuousCurrentLimit(int amps)
{
    if (amps == params_.current_limit_continuous_amps_)
        return;
    params_.current_limit_continuous_amps_ = amps;

    syncDynamicReconfigure();

    talon_->setContinuousCurrentLimit(params_.current_limit_continuous_amps_);
}

void TalonControllerInterface::setCurrentLimitEnable(bool enable)
{
    if (enable == params_.current_limit_enable_)
        return;
    params_.current_limit_enable_ = enable;

    syncDynamicReconfigure();

    talon_->setCurrentLimitEnable(params_.current_limit_enable_);
}

void TalonControllerInterface::setForwardSoftLimitThreshold(double threshold)
{
    if (threshold == params_.softlimit_forward_threshold_)
        return;
    params_.softlimit_forward_threshold_= threshold;

    syncDynamicReconfigure();
    talon_->setForwardSoftLimitThreshold(threshold);
}

void TalonControllerInterface::setForwardSoftLimitEnable(bool enable)
{
    if (enable == params_.softlimit_forward_enable_)
        return;
    params_.softlimit_forward_enable_= enable;

    syncDynamicReconfigure();
    talon_->setForwardSoftLimitEnable(enable);
}

void TalonControllerInterface::setReverseSoftLimitThreshold(double threshold)
{
    if (threshold == params_.softlimit_reverse_threshold_)
        return;
    params_.softlimit_reverse_threshold_= threshold;

    syncDynamicReconfigure();
    talon_->setReverseSoftLimitThreshold(threshold);
}

void TalonControllerInterface::setReverseSoftLimitEnable(bool enable)
{
    if (enable == params_.softlimit_reverse_enable_)
        return;
    params_.softlimit_reverse_enable_= enable;

    syncDynamicReconfigure();
    talon_->setReverseSoftLimitEnable(enable);
}

void TalonControllerInterface::setSelectedSensorPosition(double position)
{
    talon_->setSelectedSensorPosition(position);
}

void TalonControllerInterface::clearStickyFaults(void)
{
    talon_->setClearStickyFaults();
}

void TalonControllerInterface::setMotionCruiseVelocity(double velocity)
{
    if (velocity == params_.motion_cruise_velocity_)
        return;
    params_.motion_cruise_velocity_ = velocity;

    syncDynamicReconfigure();

    talon_->setMotionCruiseVelocity(params_.motion_cruise_velocity_);
}

void TalonControllerInterface::setMotionAcceleration(double acceleration)
{
    if (acceleration == params_.motion_acceleration_)
        return;
    params_.motion_acceleration_ = acceleration;

    syncDynamicReconfigure();

    talon_->setMotionAcceleration(params_.motion_acceleration_);
}

void TalonControllerInterface::setMotionSCurveStrength(unsigned int s_curve_strength)
{
    if (s_curve_strength == params_.motion_s_curve_strength_)
        return;
    params_.motion_s_curve_strength_ = s_curve_strength;

    syncDynamicReconfigure();

    talon_->setMotionSCurveStrength(params_.motion_s_curve_strength_);
}

double TalonControllerInterface::getMotionCruiseVelocity(void)
{
    return params_.motion_cruise_velocity_;
}

double TalonControllerInterface::getMotionAcceleration(void)
{
    return params_.motion_acceleration_;
}

double TalonControllerInterface::getMotionSCurveStrength(void)
{
    return params_.motion_s_curve_strength_;

}
void TalonControllerInterface::setStatusFramePeriod(hardware_interface::StatusFrame status_frame, uint8_t period)
{
    if ((status_frame < hardware_interface::Status_1_General) ||
        (status_frame >= hardware_interface::Status_Last))
    {
        ROS_ERROR("Invalid status_frame value in TalonController::setStatusFramePeriod()");
        return;
    }
    if (period == params_.status_frame_periods_[status_frame])
        return;
    params_.status_frame_periods_[status_frame] = period;

    syncDynamicReconfigure();

    talon_->setMotionProfileTrajectoryPeriod(params_.status_frame_periods_[status_frame]);
}

void TalonControllerInterface::setControlFramePeriod(hardware_interface::ControlFrame control_frame, uint8_t period)
{
    if ((control_frame < hardware_interface::Control_3_General) ||
        (control_frame >= hardware_interface::Control_Last))
    {
        ROS_ERROR("Invalid control_frame value in TalonController::setControlFramePeriod()");
        return;
    }
    if (period == params_.control_frame_periods_[control_frame])
        return;
    params_.control_frame_periods_[control_frame] = period;

    syncDynamicReconfigure();

    talon_->setMotionProfileTrajectoryPeriod(params_.control_frame_periods_[control_frame]);
}

void TalonControllerInterface::setMotionProfileTrajectoryPeriod(int msec)
{
    if (msec == params_.motion_profile_trajectory_period_)
        return;
    params_.motion_profile_trajectory_period_ = msec;

    syncDynamicReconfigure();

    talon_->setMotionProfileTrajectoryPeriod(params_.motion_profile_trajectory_period_);
}

void TalonControllerInterface::clearMotionProfileTrajectories(void)
{
    talon_->setClearMotionProfileTrajectories();
}

void TalonControllerInterface::clearMotionProfileHasUnderrun(void)
{
    talon_->setClearMotionProfileHasUnderrun();
}

void TalonControllerInterface::pushMotionProfileTrajectory(const hardware_interface::TrajectoryPoint &traj_pt)
{
    talon_->PushMotionProfileTrajectory(traj_pt);
}

double TalonControllerInterface::getPosition(void) const
{
    return talon_.state()->getPosition();
}

double TalonControllerInterface::getSpeed(void) const
{
    return talon_.state()->getSpeed();
}

double TalonControllerInterface::getOutputCurrent(void) const
{
    return talon_.state()->getOutputCurrent();
}

double TalonControllerInterface::getStatorCurrent(void) const
{
    return talon_.state()->getStatorCurrent();
}

double TalonControllerInterface::getSupplyCurrent(void) const
{
    return talon_.state()->getSupplyCurrent();
}

bool TalonControllerInterface::getForwardLimitSwitch(void) const
{
    return talon_.state()->getForwardLimitSwitch();
}
bool TalonControllerInterface::getReverseLimitSwitch(void) const
{
    return talon_.state()->getReverseLimitSwitch();
}

void TalonControllerInterface::setPeakOutputForward(double peak)
{
    if (fabs(peak - params_.peak_output_forward_) < double_value_epsilon)
        return;
    params_.peak_output_forward_ = peak;

    syncDynamicReconfigure();
    talon_->setPeakOutputForward(peak);
}

void TalonControllerInterface::setPeakOutputReverse(double peak)
{
    if (fabs(peak - params_.peak_output_reverse_) < double_value_epsilon)
        return;
    params_.peak_output_reverse_ = peak;
    syncDynamicReconfigure();
    talon_->setPeakOutputReverse(peak);
}

void TalonControllerInterface::setClosedloopRamp(double closed_loop_ramp)
{
    if (fabs(closed_loop_ramp - params_.closed_loop_ramp_) < double_value_epsilon)
        return;
    params_.closed_loop_ramp_ = closed_loop_ramp;
    syncDynamicReconfigure();
    talon_->setClosedloopRamp(params_.closed_loop_ramp_);
}

void TalonControllerInterface::setOpenloopRamp(double open_loop_ramp)
{
    if (fabs(open_loop_ramp - params_.open_loop_ramp_) < double_value_epsilon)
        return;
    params_.open_loop_ramp_ = open_loop_ramp;
    syncDynamicReconfigure();
    talon_->setOpenloopRamp(params_.open_loop_ramp_);
}

void TalonControllerInterface::setDemand1Type(hardware_interface::DemandType demand1_type)
{
    if (demand1_type == params_.demand1_type_)
        return;
    if (demand1_type == hardware_interface::DemandType_AuxPID)
    {
        ROS_ERROR_STREAM("Demand Type is DemandType_AuxPID! Not supported!");
        return;
    }
    params_.demand1_type_ = demand1_type;
    syncDynamicReconfigure();
    talon_->setDemand1Type(demand1_type);
}

void TalonControllerInterface::setDemand1Value(double demand1_value)
{
    if (fabs(demand1_value - params_.demand1_value_) < double_value_epsilon)
        return;
    params_.demand1_value_ = demand1_value;
    syncDynamicReconfigure();
    talon_->setDemand1Value(demand1_value);
}

// Used to set initial (and only) talon
// mode for FixedMode derived classes
bool TalonControllerInterface::setInitialMode(void)
{
    ROS_INFO_STREAM("Talon " << talon_.getName() << " Base class setInitialMode");
    return true;
}

bool TalonControllerInterface::init(hardware_interface::TalonCommandInterface *tci,
                                    ros::NodeHandle &n,
                                    hardware_interface::TalonCommandHandle &talon,
                                    std::shared_ptr<boost::recursive_mutex> srv_mutex,
                                    bool follower)
{
    ROS_WARN("init start");

    // Read params from startup and intialize Talon using them
    TalonCIParams params;
    if (!readParams(n, params))
    {
        return false;
    }
    ROS_WARN("init past readParams");

    talon = tci->getHandle(params.joint_name_);
    writeParamsToHW(params, talon, !follower);
    ROS_WARN("init past writeParamsToHW");

    bool dynamic_reconfigure = false;
    // Only allow dynamic reconfigure to be active for non-follower talons
    if (!follower)
    {
        n.param<bool>("dynamic_reconfigure", dynamic_reconfigure, dynamic_reconfigure);
    }
    if (dynamic_reconfigure)
    {
        // Create dynamic_reconfigure Server. Pass in n
        // so that all the vars for the class are grouped
        // under the node's name.  Doing so allows multiple
        // copies of the class to be started, each getting
        // their own namespace.
        auto srv = std::make_shared<dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig>>(*srv_mutex, n);

        ROS_WARN("init updateConfig");
        // Without this, the first call to callback()
        // will overwrite anything passed in from the
        // launch file
        srv->updateConfig(params_.toConfig());

        ROS_WARN("init setCallback");
        // Register a callback function which is run each
        // time parameters are changed using
        // rqt_reconfigure or the like
        srv->setCallback(boost::bind(&TalonControllerInterface::callback, this, _1, _2));

        // Create a thread to update the server with new values written by users of this interface
        srv_update_thread_ = std::jthread(std::bind(&TalonControllerInterface::srvUpdateThread, this, srv));
    }
    ROS_WARN("init returning");

    return true;
}

// If dynamic reconfigure is running then update
// the reported config there with the new internal state
// Trigger a write of the current CIParams to the DDR server. This needs
// to happen if the values have been updated via the interface code.
// The meaning of the flag - set == no new updates, cleared == data has
// been updated via code and the reconfigure gui needs that new data sent
// to it to stay in sync
// This call is on the control loop update path and needs to be quick.
// atomic_flags are lock free, so clearing it should be a fast operation
// which never blocks.  Thus, with any luck it won't slow down the
// control loop by any significant amount.
void TalonControllerInterface::syncDynamicReconfigure()
{
    srv_update_thread_flag_.clear();
}

// Loop forever, waiting for requests from the main thread
// to update values from this class to the DDR server.
void TalonControllerInterface::srvUpdateThread(std::shared_ptr<dynamic_reconfigure::Server<TalonConfigConfig>> srv)
{
    ROS_INFO_STREAM("srvUpdateThread started for joint " << params_.joint_name_);
    // Early out if ddr isn't used for this controller
    if (!srv)
    {
        return;
    }
#ifdef __linux__
    struct sched_param sp;
    sp.sched_priority = 0;
    sched_setscheduler(0, SCHED_IDLE, &sp); // GUI update is low priority compared to driving the robot
    pthread_setname_np(pthread_self(), "tci_ddr_upd");
    ROS_INFO_STREAM("srvUpdateThread priority set for joint " << params_.joint_name_);
#endif
    srv_update_thread_active_ = true;
    ros::Rate r(10);
    while (srv_update_thread_active_)
    {
        // Loop forever, periodically checking for the flag to be cleared
        // Test and set returns the previous value of the variable and sets
        // it, all in one atomic operation.  The set will reset the flag
        // so after running updateConfig() the code will loop back and wait
        // here for the next time the flag is cleared by syncDynamicReconfigure.
        if (!srv_update_thread_flag_.test_and_set())
        {
            TalonConfigConfig config(params_.toConfig());
            srv->updateConfig(config);
        }

        r.sleep();
    }
}

// Use data in params to actually set up Talon
// hardware. Make this a separate method outside of
// init() so that dynamic reconfigure callback can write
// values using this method at any time
// Technically, this writes to the talon command buffers
// (via the talon_ handle), which doesn't actually write
// to hardware. But writing to those buffers queues up an
// actual write to the hardware in write() in the hardware
// interface
void TalonControllerInterface::writeParamsToHW(const TalonCIParams &params,
                                hardware_interface::TalonCommandHandle &talon,
                                bool update_params)
{
    std::lock_guard l{*talon};
    // perform additional hardware init here
    // but don't set mode - either force the caller to
    // set it or use one of the derived, fixed-mode
    // classes instead
    for (size_t i = 0; i < hardware_interface::TALON_PIDF_SLOTS; i++)
    {
        talon->setP(params.p_[i], i);
        talon->setI(params.i_[i], i);
        talon->setD(params.d_[i], i);
        talon->setF(params.f_[i], i);
        talon->setIZ(params.izone_[i], i);
        // TODO : I'm worried about these. We need
        // better default values than 0.0
        talon->setAllowableClosedloopError(params.allowable_closed_loop_error_[i], i);
        talon->setMaxIntegralAccumulator(params.max_integral_accumulator_[i], i);
        talon->setClosedLoopPeakOutput(params.closed_loop_peak_output_[i], i);
        talon->setClosedLoopPeriod(params.closed_loop_period_[i], i);
    }
    talon->setPidfSlot(params.pidf_slot_);
    talon->setAuxPidPolarity(params.aux_pid_polarity_);
    talon->setDemand1Type(params.demand1_type_);
    talon->setDemand1Value(params.demand1_value_);
    talon->setNeutralMode(params.neutral_mode_);

    talon->setEncoderFeedback(params.feedback_type_);
    talon->setFeedbackCoefficient(params.feedback_coefficient_);
    talon->setRemoteEncoderFeedback(params.remote_feedback_type_);
    talon->setEncoderTicksPerRotation(params.ticks_per_rotation_);
    for (size_t i = 0; i < params.remote_feedback_filters_.size(); ++i)
    {
        talon->setRemoteFeedbackDeviceId(params.remote_feedback_device_ids_[i], i);
        talon->setRemoteFeedbackFilter(params.remote_feedback_filters_[i], i);
    }
    for (hardware_interface::SensorTerm i = hardware_interface::SensorTerm_Sum0; i < hardware_interface::SensorTerm_Last; i = static_cast<hardware_interface::SensorTerm>(i + 1))
        talon->setSensorTerm(params.sensor_terms_[i], i);

    talon->setInvert(params.invert_output_);
    talon->setSensorPhase(params.sensor_phase_);

    talon->setClosedloopRamp(params.closed_loop_ramp_);
    talon->setOpenloopRamp(params.open_loop_ramp_);
    talon->setPeakOutputForward(params.peak_output_forward_);
    talon->setPeakOutputReverse(params.peak_output_reverse_);
    talon->setNominalOutputForward(params.nominal_output_forward_);
    talon->setNominalOutputReverse(params.nominal_output_reverse_);
    talon->setNeutralDeadband(params.neutral_deadband_);

    talon->setVoltageCompensationSaturation(params.voltage_compensation_saturation_);
    talon->setVoltageMeasurementFilter(params.voltage_measurement_filter_);
    talon->setVoltageCompensationEnable(params.voltage_compensation_enable_);

    talon->setVelocityMeasurementPeriod(params.velocity_measurement_period_);
    talon->setVelocityMeasurementWindow(params.velocity_measurement_window_);

    talon->setForwardLimitSwitchSource(params.limit_switch_local_forward_source_, params.limit_switch_local_forward_normal_);
    talon->setReverseLimitSwitchSource(params.limit_switch_local_reverse_source_, params.limit_switch_local_reverse_normal_);

    talon->setRemoteForwardLimitSwitchSource(params.limit_switch_remote_forward_source_, params.limit_switch_remote_forward_normal_, params.limit_switch_remote_forward_id_);
    talon->setRemoteReverseLimitSwitchSource(params.limit_switch_remote_reverse_source_, params.limit_switch_remote_reverse_normal_, params.limit_switch_remote_reverse_id_);
    talon->setOverrideSoftLimitsEnable(params.override_limit_switches_enable_);
    talon->setForwardSoftLimitThreshold(params.softlimit_forward_threshold_);
    talon->setForwardSoftLimitEnable(params.softlimit_forward_enable_);
    talon->setReverseSoftLimitThreshold(params.softlimit_reverse_threshold_);
    talon->setReverseSoftLimitEnable(params.softlimit_reverse_enable_);
    talon->setClearPositionOnLimitF(params.clear_position_on_limit_f_);
    talon->setClearPositionOnLimitR(params.clear_position_on_limit_r_);

    talon->setPeakCurrentLimit(params.current_limit_peak_amps_);
    talon->setPeakCurrentDuration(params.current_limit_peak_msec_);
    talon->setContinuousCurrentLimit(params.current_limit_continuous_amps_);
    talon->setCurrentLimitEnable(params.current_limit_enable_);

    talon->setSupplyCurrentLimit(params.supply_current_limit_);
    talon->setSupplyCurrentTriggerThresholdCurrent(params.supply_current_trigger_threshold_current_);
    talon->setSupplyCurrentTriggerThresholdTime(params.supply_current_trigger_threshold_time_);
    talon->setSupplyCurrentLimitEnable(params.supply_current_limit_enable_);

    talon->setMotionCruiseVelocity(params.motion_cruise_velocity_);
    talon->setMotionAcceleration(params.motion_acceleration_);
    talon->setMotionProfileTrajectoryPeriod(params.motion_profile_trajectory_period_);
    for (int i = hardware_interface::Status_1_General; i < hardware_interface::Status_Last; i++)
        talon->setStatusFramePeriod(static_cast<hardware_interface::StatusFrame>(i), params.status_frame_periods_[i]);
    for (int i = hardware_interface::Control_3_General; i < hardware_interface::Control_Last; i++)
        talon->setControlFramePeriod(static_cast<hardware_interface::ControlFrame>(i), params.control_frame_periods_[i]);

    talon->setConversionFactor(params.conversion_factor_);

    talon->setEnableReadThread(params.enable_read_thread_);

    // Save copy of params written to HW
    // so they can be queried later?
    if (update_params)
        params_ = params;
}

template<typename hardware_interface::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
TalonFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>::~TalonFixedModeControllerInterface() = default;

// Disable changing mode for controllers derived from this class
template<typename hardware_interface::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
void TalonFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>::setMode(hardware_interface::TalonMode /*mode*/)
{
    ROS_WARN("Can't reset mode using this TalonControllerInterface");
}

template<typename hardware_interface::TalonMode TALON_MODE, const char *TALON_MODE_NAME>
bool TalonFixedModeControllerInterface<TALON_MODE, TALON_MODE_NAME>::setInitialMode(void)
{
    // Set the mode at init time for fixed mode controllers since
    // they can't be changed after that
    talon_->setMode(TALON_MODE);
    ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in " << TALON_MODE_NAME << " mode");
    return true;
}
TalonFollowerControllerInterface::~TalonFollowerControllerInterface() = default;

bool TalonFollowerControllerInterface::initWithNode(hardware_interface::TalonCommandInterface *tci,
                                                    hardware_interface::TalonStateInterface *tsi,
                                                    ros::NodeHandle &n)
{
    if (!tsi)
    {
        ROS_ERROR("NULL TalonStateInterface in TalonFollowerCommandInterface");
        return false;
    }

    // Call base-class init to load config params
    if (!TalonControllerInterface::initWithNode(tci, tsi, n))
    {
        ROS_ERROR("TalonFollowerController base initWithNode failed");
        return false;
    }

    std::string follow_joint_name;
    if (!n.getParam("follow_joint", follow_joint_name))
    {
        ROS_ERROR("No follow joint specified for TalonFollowerControllerInterface");
        return false;
    }

    hardware_interface::TalonStateHandle follow_handle = tsi->getHandle(follow_joint_name);
    const int follow_can_id = follow_handle->getCANID();

    // Set the mode and CAN ID of talon to follow at init time -
    // since this class is derived from the FixedMode class
    // these can't be reset. Hopefully we never have a case
    // where a follower mode Talon changes which other
    // Talon it is following during a match?
    talon_->setMode(hardware_interface::TalonMode_Follower);
    talon_->set(follow_can_id);

    ROS_INFO_STREAM("Launching follower " << params_.joint_name_ << " to follow CAN ID " << follow_can_id << " (" << follow_handle.getName() << ")");
    return true;
}

// Maybe disable the setPIDFSlot call since that makes
// no sense for a non-PID controller mode?
void TalonFollowerControllerInterface::setMode(hardware_interface::TalonMode /*mode*/)
{
    ROS_WARN("Can't set a command in follower mode!");
}
void TalonFollowerControllerInterface::setCommand(const double /*command*/)
{
    ROS_WARN("Can't set a command in follower mode!");
}

template class TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_PercentOutput, PERCENT_OUTPUT_NAME>;
template class TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_Position, CLOSE_LOOP_POSITION_NAME>;
template class TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_Velocity, CLOSE_LOOP_VELOCITY_NAME>;
template class TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_Current, CLOSE_LOOP_CURRENT_NAME>;
template class TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_MotionProfile, MOTION_PROFILE_NAME>;
template class TalonFixedModeControllerInterface<hardware_interface::TalonMode::TalonMode_MotionMagic, MOTION_MAGIC_NAME>;

} // namespace talon_controllers