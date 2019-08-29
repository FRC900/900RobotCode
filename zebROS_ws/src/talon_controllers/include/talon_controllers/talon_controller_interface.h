#pragma once
#include <dynamic_reconfigure/server.h>
#include <talon_interface/talon_command_interface.h>
#include <talon_controllers/TalonConfigConfig.h>
#include <XmlRpcValue.h>

namespace talon_controllers
{
// Create a wrapper class for each Talon mode.  For the basic controllers
// this isn't really helpful since the code could just be included in the
// controller itself.  But consider a more complex controller, for example
// swerve. The swerve controller runs a number of wheels, and each wheel
// has both position and velocity.  The wheel class might create a
// TalonPosisionPIDControllerInterface member var for the position motor and
// also a TalonVelocityPIDControllerInterface member var for the velocity.
// And since it will be creating one per wheel, it makes sense to wrap the common
// init code into a class rather than duplicate it for each wheel. Another
// controller - say a shooter wheel - could also use this same code to
// create a talon handle to access that motor
//

// Class which provides a common set of code for reading
// parameters for motor controllers from yaml / command line
// ROS params.  Not all of these values will be needed for all
// modes - specific controller interfaces will use what
// they do need and ignore the rest.
// The idea here is that code using a particular CI would
// call readParams(), modify any parameters which are specific
// to the controller, and then call init using the specificed
// parameters. This will handle the common case where most
// code using the CI will want to use the default names of settings
// but also allow customization
class TalonCIParams
{
	public:
		// Initialize with relatively sane defaults
		// for all parameters
		TalonCIParams(void) :
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
			neutral_mode_(hardware_interface::NeutralMode_Uninitialized),
			feedback_type_(hardware_interface::FeedbackDevice_Uninitialized),
			feedback_coefficient_(1.0),
			remote_feedback_type_(hardware_interface::RemoteFeedbackDevice_FactoryDefaultOff),
			ticks_per_rotation_(4096),
			remote_feedback_device_ids_{0, 0},
			remote_feedback_filters_{hardware_interface::RemoteSensorSource_Off, hardware_interface::RemoteSensorSource_Off},
			sensor_terms_{hardware_interface::FeedbackDevice_Uninitialized,hardware_interface::FeedbackDevice_Uninitialized,hardware_interface::FeedbackDevice_Uninitialized,hardware_interface::FeedbackDevice_Uninitialized},
			closed_loop_ramp_(0.),
			open_loop_ramp_(0.),
			peak_output_forward_(1.),
			peak_output_reverse_(-1.),
			nominal_output_forward_(0.),
			nominal_output_reverse_(0.),
			neutral_deadband_(0.04),
			voltage_compensation_saturation_(12.5),
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
			motion_cruise_velocity_(0),
			motion_acceleration_(0),
			motion_s_curve_strength_(0),
			motion_profile_trajectory_period_(0),

			conversion_factor_(1.0),

			custom_profile_hz_(20.0),

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

			control_frame_periods_[hardware_interface::Control_3_General] = hardware_interface::control_3_general_default;
			control_frame_periods_[hardware_interface::Control_4_Advanced] = hardware_interface::control_4_advanced_default;
			control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride] = hardware_interface::control_5_feedbackoutputoverride_default;
			control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint] = hardware_interface::control_6_motprofaddtrajpoint_default;
		}

		// Update params set by a dynamic reconfig config
		// Also pass in current params for ones which aren't
		// dynamically reconfigurable - pass them through
		// to the new one
		TalonCIParams(const TalonConfigConfig &config)
		{
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

			softlimit_forward_threshold_ = config.softlimit_forward_threshold;
			softlimit_forward_enable_ = config.softlimit_forward_enable;
			softlimit_reverse_threshold_ = config.softlimit_reverse_threshold;
			softlimit_reverse_enable_ = config.softlimit_reverse_enable;
			override_limit_switches_enable_ = config.softlimits_override_enable;

			current_limit_peak_amps_ = config.current_limit_peak_amps;
			current_limit_peak_msec_ = config.current_limit_peak_msec;
			current_limit_continuous_amps_ = config.current_limit_continuous_amps;
			current_limit_enable_ = config.current_limit_enable;
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

			control_frame_periods_[hardware_interface::Control_3_General] = config.control_3_general_period;
			control_frame_periods_[hardware_interface::Control_4_Advanced] = config.control_4_advanced_period;
			control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride] = config.control_5_feedbackoutputoverride_period;
			control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint] = config.control_6_motprofaddtrajpoint_period;
			conversion_factor_ = config.conversion_factor;

			custom_profile_hz_ = config.custom_profile_hz;

			enable_read_thread_ = true;
		}

		// Copy from internal state to TalonConfigConfig state
		TalonConfigConfig toConfig(void) const
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
			config.softlimit_forward_threshold = softlimit_forward_threshold_;
			config.softlimit_forward_enable = softlimit_forward_enable_;
			config.softlimit_reverse_threshold = softlimit_reverse_threshold_;
			config.softlimit_reverse_enable = softlimit_reverse_enable_;
			config.softlimits_override_enable = override_limit_switches_enable_;
			config.current_limit_peak_amps = current_limit_peak_amps_;
			config.current_limit_peak_msec = current_limit_peak_msec_;
			config.current_limit_continuous_amps = current_limit_continuous_amps_;
			config.current_limit_enable = current_limit_enable_;
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

			config.control_3_general_period = control_frame_periods_[hardware_interface::Control_3_General];
			config.control_4_advanced_period = control_frame_periods_[hardware_interface::Control_4_Advanced];
			config.control_5_feedbackoutputoverride_period = control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride];
			config.control_6_motprofaddtrajpoint_period = control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint];

			config.conversion_factor = conversion_factor_;
			config.custom_profile_hz = custom_profile_hz_;
			return config;
		}

		// Read a joint name from the given nodehandle's params
		bool readJointName(ros::NodeHandle &n)
		{
			if (!n.getParam("joint", joint_name_))
			{
				ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
				return false;
			}
			return true;
		}

		bool readConversion(ros::NodeHandle &n)
		{
			n.getParam("conversion_factor", conversion_factor_);
			return true;
		}

		// Read a joint name from the given nodehandle's params
		bool readNeutralMode(ros::NodeHandle &n)
		{
			std::string mode_string;
			if (n.getParam("neutral_mode", mode_string))
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
		//TODOa: create a method that reads the feedback settings enum
		bool readFeedbackType(ros::NodeHandle &n)
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
				if (str == "FactoryDefaultOff")
					remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_FactoryDefaultOff;
				else if (str == "SensorSum")
					remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_SensorSum;
				else if (str == "SensorDifference")
					remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_SensorDifference;
				else if (str == "RemoteSensor0")
					remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_RemoteSensor0;
				else if (str == "RemoteSensor1")
					remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_RemoteSensor1;
				else if (str == "SoftwareEmulatedSensor")
					remote_feedback_type_ = hardware_interface::RemoteFeedbackDevice_SoftwareEmulatedSensor;
				else
				{
					ROS_ERROR_STREAM("Invalid remote feedback device name given : " << str);
					return false;
				}
			}
			n.getParam("remote_feedback_device_id0", remote_feedback_device_ids_[0]);
			if (n.getParam("remote_feedback_filter0", str) &&
				!stringToRemoteSensorSource(str, remote_feedback_filters_[0]))
			{
				ROS_ERROR_STREAM("Invalid remote_feedback_filter0 device name given : " << str);
				return false;
			}
			n.getParam("remote_feedback_device_id1", remote_feedback_device_ids_[1]);
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

		bool readInverts(ros::NodeHandle &n)
		{
			n.getParam("invert_output", invert_output_);
			n.getParam("sensor_phase", sensor_phase_);
			return true;
		}

		bool readCloseLoopParams(ros::NodeHandle &n)
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
					findIntParam("i_zone", pidparams, izone_[i]);
					findIntParam("allowable_closed_loop_error", pidparams, allowable_closed_loop_error_[i]);
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

		bool readOutputShaping(ros::NodeHandle &n)
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
		bool readVoltageCompensation(ros::NodeHandle &n)
		{
			int params_read = 0;
			if (n.getParam("voltage_compensation_saturation", voltage_compensation_saturation_))
				params_read += 1;
			if (n.getParam("voltage_measurement_filter", voltage_measurement_filter_))
				params_read += 1;
			if (n.getParam("voltage_compensation_enable", voltage_compensation_enable_) &&
				voltage_compensation_enable_ && (params_read < 2))
				ROS_WARN("Not all voltage compensation params set before enabling - using defaults might not work as expected");
			return true;
		}

		bool readVelocitySignalConditioning(ros::NodeHandle &n)
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

		bool readLimitSwitches(ros::NodeHandle &n)
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
			return true;
		}

		bool readSoftLimits(ros::NodeHandle &n)
		{
			int param_count = 0;
			if (n.getParam("softlimit_forward_threshold", softlimit_forward_threshold_))
				param_count = 1;
			if (n.getParam("softlimit_forward_enable", softlimit_forward_enable_) &&
					softlimit_forward_enable_ && (param_count == 0))
				ROS_WARN("Enabling forward softlimits without setting threshold");
			param_count = 0;
			if (n.getParam("softlimit_reverse_threshold", softlimit_reverse_threshold_))
				param_count = 1;
			if (n.getParam("softlimit_reverse_enable", softlimit_reverse_enable_) &&
				softlimit_reverse_enable_ && (param_count == 0))
					ROS_WARN("Enabling forward softlimits without setting threshold");
			n.getParam("override_limit_switches_enable", override_limit_switches_enable_);
			return true;
		}

		bool readCurrentLimits(ros::NodeHandle &n)
		{
			int params_read = 0;
			if (n.getParam("current_limit_peak_amps", current_limit_peak_amps_))
				params_read += 1;
			if (n.getParam("current_limit_peak_msec", current_limit_peak_msec_))
				params_read += 1;
			if (n.getParam("current_limit_continuous_amps", current_limit_continuous_amps_))
				params_read += 1;
			if (n.getParam("current_limit_enable", current_limit_enable_) &&
				current_limit_enable_ && (params_read < 3))
				ROS_WARN("Not all current limits set before enabling - using defaults might not work as expected");
			return true;
		}

		bool readMotionControl(ros::NodeHandle &n)
		{
			n.getParam("motion_cruise_velocity", motion_cruise_velocity_);
			n.getParam("motion_acceleration", motion_acceleration_);
			n.getParam("motion_s_curve_strength", motion_s_curve_strength_);
			n.getParam("motion_profile_trajectory_period", motion_profile_trajectory_period_);
			return true;
		}

		bool readStatusFramePeriods(ros::NodeHandle &n)
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
			return true;
		}

		bool readControlFramePeriods(ros::NodeHandle &n)
		{
			n.getParam("control_3_general_period", control_frame_periods_[hardware_interface::Control_3_General]);
			n.getParam("control_4_advanced_period", control_frame_periods_[hardware_interface::Control_4_Advanced]);
			n.getParam("control_5_feedbackoutputoverride_period", control_frame_periods_[hardware_interface::Control_5_FeedbackOutputOverride]);
			n.getParam("control_6_motprofaddtrajpoint_period", control_frame_periods_[hardware_interface::Control_6_MotProfAddTrajPoint]);

			return true;
		}

		bool readCustomProfile(ros::NodeHandle &n)
		{
			n.getParam("custom_profile_hz", custom_profile_hz_);
			return true;
		}

		bool readTalonThread(ros::NodeHandle &n)
		{
			n.getParam("enable_read_thread", enable_read_thread_);
			return true;
		}

		std::string joint_name_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> p_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> i_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> d_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> f_;
		std::array<int, hardware_interface::TALON_PIDF_SLOTS>    izone_;
		std::array<int, hardware_interface::TALON_PIDF_SLOTS>    allowable_closed_loop_error_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> max_integral_accumulator_;
		std::array<double, hardware_interface::TALON_PIDF_SLOTS> closed_loop_peak_output_;
		std::array<int, hardware_interface::TALON_PIDF_SLOTS>    closed_loop_period_;
		int    pidf_slot_;
		bool   aux_pid_polarity_;
		hardware_interface::DemandType demand1_type_;
		double demand1_value_;
		bool   invert_output_;
		bool   sensor_phase_;
		hardware_interface::NeutralMode neutral_mode_;
		hardware_interface::FeedbackDevice feedback_type_;
		double feedback_coefficient_;
		hardware_interface::RemoteFeedbackDevice remote_feedback_type_;
		int    ticks_per_rotation_;
		std::array<int, 2>                                    remote_feedback_device_ids_;
		std::array<hardware_interface::RemoteSensorSource, 2> remote_feedback_filters_;
		std::array<hardware_interface::FeedbackDevice, hardware_interface::SensorTerm_Last> sensor_terms_;
		double closed_loop_ramp_;
		double open_loop_ramp_;
		double peak_output_forward_;
		double peak_output_reverse_;
		double nominal_output_forward_;
		double nominal_output_reverse_;
		double neutral_deadband_;
		double voltage_compensation_saturation_;
		int    voltage_measurement_filter_;
		bool   voltage_compensation_enable_;
		hardware_interface::VelocityMeasurementPeriod velocity_measurement_period_;
		int    velocity_measurement_window_;

		hardware_interface::LimitSwitchSource limit_switch_local_forward_source_;
		hardware_interface::LimitSwitchNormal limit_switch_local_forward_normal_;
		hardware_interface::LimitSwitchSource limit_switch_local_reverse_source_;
		hardware_interface::LimitSwitchNormal limit_switch_local_reverse_normal_;
		hardware_interface::RemoteLimitSwitchSource limit_switch_remote_forward_source_;
		hardware_interface::LimitSwitchNormal limit_switch_remote_forward_normal_;
		unsigned int                          limit_switch_remote_forward_id_;
		hardware_interface::RemoteLimitSwitchSource limit_switch_remote_reverse_source_;
		hardware_interface::LimitSwitchNormal limit_switch_remote_reverse_normal_;
		unsigned int                          limit_switch_remote_reverse_id_;

		double softlimit_forward_threshold_;
		bool   softlimit_forward_enable_;
		double softlimit_reverse_threshold_;
		bool   softlimit_reverse_enable_;
		bool   override_limit_switches_enable_;
		int    current_limit_peak_amps_;
		int    current_limit_peak_msec_;
		int    current_limit_continuous_amps_;
		bool   current_limit_enable_;
		double motion_cruise_velocity_;
		double motion_acceleration_;
		double motion_s_curve_strength_;
		int    motion_profile_trajectory_period_;
		std::array<int, hardware_interface::Status_Last> status_frame_periods_;
		std::array<int, hardware_interface::Control_Last> control_frame_periods_;

		double conversion_factor_;

		double custom_profile_hz_;

		bool enable_read_thread_;

	private:
		// Read a double named <param_type> from the array/map
		// in params
		bool findFloatParam(std::string param_type, XmlRpc::XmlRpcValue &params, double &val) const
		{
			if (!params.hasMember(param_type))
				return false;
			XmlRpc::XmlRpcValue &param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a double valid type");
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
				throw std::runtime_error("A non-double value was passed for" + param_type);

			return false;
		}

		// Read an integer named <param_type> from the array/map
		// in params
		bool findIntParam(std::string param_type, XmlRpc::XmlRpcValue &params, int &val) const
		{
			if (!params.hasMember(param_type))
				return false;
			XmlRpc::XmlRpcValue &param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a valid int type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
				val = (int)param;
			else
				throw std::runtime_error("A non-int value was passed for" + param_type);
			return false;
		}

		bool stringToLimitSwitchSource(const std::string &str,
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
		bool stringToRemoteLimitSwitchSource(const std::string &str,
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
		bool stringToLimitSwitchNormal(const std::string &str,
									   hardware_interface::LimitSwitchNormal &limit_switch_source) const
		{
			if (str == "NormallyOpen")
				limit_switch_source = hardware_interface::LimitSwitchNormal_NormallyOpen;
			else if (str == "NormallyClosed")
				limit_switch_source = hardware_interface::LimitSwitchNormal_NormallyClosed;
			else if (str == "Disabled")
				limit_switch_source = hardware_interface::LimitSwitchNormal_Disabled;
			else
			{
				ROS_ERROR_STREAM("Invalid limit switch normal : " << str);
				return false;
			}
			return true;
		}

		bool stringToFeedbackDevice(const std::string &str,
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

		bool stringToRemoteSensorSource(const std::string &str,
				hardware_interface::RemoteSensorSource &source)
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
			else
			{
				ROS_ERROR_STREAM("Invalid remote sensor source filter type : " << str);
				return false;
			}
			return true;
		}
};

// Base class for controller interface types. This class
// will be the least restrictive - allow users to swtich modes,
// reprogram any config values, and so on.
// Derived classes will be more specialized - they'll only allow
// a specific Talon mode and disable code which doesn't apply
// to that mode
class TalonControllerInterface
{
	public:
		TalonControllerInterface(void) :
			srv_(nullptr),
			srv_mutex_(nullptr)
		{
		}

		// Standardize format for reading params for
		// motor controller
		virtual bool readParams(ros::NodeHandle &n, TalonCIParams &params)
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
				   params.readMotionControl(n) &&
				   params.readStatusFramePeriods(n) &&
				   params.readControlFramePeriods(n) &&
				   params.readCustomProfile(n) &&
				   params.readTalonThread(n);
		}

		// Read params from config file and use them to
		// initialize the Talon hardware
		// Useful for the hopefully common case where there's
		// no need to modify the parameters after reading
		// them
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
								  hardware_interface::TalonStateInterface * /*tsi*/,
								  ros::NodeHandle &n)
		{
			return init(tci, n, talon_, srv_mutex_, srv_, true) &&
				   setInitialMode();
		}

		// Same as above, except pass in an array
		// of node handles. First entry is set up as the master
		// talon and the rest are set in follower mode to follow
		// the leader
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
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
				follower_srv_mutexes_.push_back(nullptr);
				follower_srvs_.push_back(nullptr);
				if (!init(tci, n[i], follower_talons_[i-1], follower_srv_mutexes_[i-1], follower_srvs_[i-1], false))
					return false;
				follower_talons_[i-1]->setMode(hardware_interface::TalonMode_Follower);
				follower_talons_[i-1]->set(follow_can_id);
				follower_talons_[i-1]->setCustomProfileDisable(true);
				ROS_INFO_STREAM("Set up talon " << follower_talons_[i-1].getName() << " to follow CAN ID " << follow_can_id << " (" << talon_.getName() << ")");
			}

			return true;
		}

		// Init with XmlRpcValue instead of NodeHandle - XmlRpcValue
		// will be either a string or an array of strings of joints
		// to load
		virtual bool initWithNode(hardware_interface::TalonCommandInterface *tci,
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
					joint_nodes.push_back(ros::NodeHandle(controller_nh,
								static_cast<std::string>(param[i])));
			}
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
			{
				joint_nodes.push_back(ros::NodeHandle(controller_nh,
							static_cast<std::string>(param)));
			}
			else
			{
				ROS_ERROR_STREAM("Joint param is neither a list of strings nor a string.");
				return false;
			}

			return initWithNode(tci, tsi, joint_nodes);
		}

#if 0
		// Allow users of the ControllerInterface to get
		// a copy of the parameters currently set for the
		// Talon.  They can then modify them at will and
		// call initWithParams to reprogram the Talon.
		// Hopefully this won't be needed all that often ...
		// the goal should be to provide a simpler interface
		// for commonly used operations
		virtual TalonCIParams getParams(void) const
		{
			return params_;
		}

		// Initialize Talon hardware with the settings in params
		virtual bool initWithParams(hardware_interface::TalonCommandInterface *tci,
									const TalonCIParams &params)
		{
			talon_ = tci->getHandle(params.joint_name_);
			return writeParamsToHW(params);
		}
#endif

		void callback(talon_controllers::TalonConfigConfig &config, uint32_t /*level*/)
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

			writeParamsToHW(TalonCIParams(config), talon_);
		}

		// Set the setpoint for the motor controller
		virtual void setCommand(const double command)
		{
			talon_->set(command);
		}

		// Set the mode of the motor controller
		virtual void setMode(hardware_interface::TalonMode mode)
		{
			talon_->setMode(mode);
		}

		// Set the mode of the motor controller
		hardware_interface::TalonMode getMode(void) const
		{
			return talon_.state()->getTalonMode();
		}

		// Pick the config slot (0 or 1) for PIDF/IZone values
		virtual bool setPIDFSlot(int slot)
		{
			if ((slot != 0) && (slot != 1))
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

		virtual bool setP(double newP, int slot)
		{
			if ((slot != 0) && (slot != 1))
			{
				//ROS_WARN_STREAM("controller set of PID slot:  (false): " << slot);
				return false;
			}
			syncDynamicReconfigure();

			talon_->setP(newP, slot);
			return true;
		}

		virtual void setNeutralOutput(void)
		{
			talon_->setNeutralOutput();
		}

		virtual void setIntegralAccumulator(double iaccum)
		{
			talon_->setIntegralAccumulator(iaccum);
		}

		virtual void setOverrideSoftLimitsEnable(bool enable)
		{
			if (enable == params_.override_limit_switches_enable_)
				return;
			params_.override_limit_switches_enable_ = enable;
			syncDynamicReconfigure();
			talon_->setOverrideSoftLimitsEnable(enable);
		}

		virtual void setPeakCurrentLimit(int amps)
		{
			if (amps == params_.current_limit_peak_amps_)
				return;
			params_.current_limit_peak_amps_ = amps;

			syncDynamicReconfigure();

			talon_->setPeakCurrentLimit(params_.current_limit_peak_amps_);
		}

		virtual void setPeakCurrentDuration(int msec)
		{
			if (msec == params_.current_limit_peak_msec_)
				return;
			params_.current_limit_peak_msec_ = msec;

			syncDynamicReconfigure();

			talon_->setPeakCurrentDuration(params_.current_limit_peak_msec_);
		}

		virtual void setContinuousCurrentLimit(int amps)
		{
			if (amps == params_.current_limit_continuous_amps_)
				return;
			params_.current_limit_continuous_amps_ = amps;

			syncDynamicReconfigure();

			talon_->setContinuousCurrentLimit(params_.current_limit_continuous_amps_);
		}

		virtual void setCurrentLimitEnable(bool enable)
		{
			if (enable == params_.current_limit_enable_)
				return;
			params_.current_limit_enable_ = enable;

			syncDynamicReconfigure();

			talon_->setCurrentLimitEnable(params_.current_limit_enable_);
		}

		virtual void setForwardSoftLimitThreshold(double threshold)
		{
			if (threshold == params_.softlimit_forward_threshold_)
				return;
			params_.softlimit_forward_threshold_= threshold;

			syncDynamicReconfigure();
			talon_->setForwardSoftLimitThreshold(threshold);
		}

		virtual void setForwardSoftLimitEnable(bool enable)
		{
			if (enable == params_.softlimit_forward_enable_)
				return;
			params_.softlimit_forward_enable_= enable;

			syncDynamicReconfigure();
			talon_->setForwardSoftLimitEnable(enable);
		}

		virtual void setReverseSoftLimitThreshold(double threshold)
		{
			if (threshold == params_.softlimit_reverse_threshold_)
				return;
			params_.softlimit_reverse_threshold_= threshold;

			syncDynamicReconfigure();
			talon_->setReverseSoftLimitThreshold(threshold);
		}

		virtual void setReverseSoftLimitEnable(bool enable)
		{
			if (enable == params_.softlimit_reverse_enable_)
				return;
			params_.softlimit_reverse_enable_= enable;

			syncDynamicReconfigure();
			talon_->setReverseSoftLimitEnable(enable);
		}

		virtual void setSelectedSensorPosition(double position)
		{
			talon_->setSelectedSensorPosition(position);
		}

		virtual void clearStickyFaults(void)
		{
			talon_->setClearStickyFaults();
		}

		virtual void setMotionCruiseVelocity(double velocity)
		{
			if (velocity == params_.motion_cruise_velocity_)
				return;
			params_.motion_cruise_velocity_ = velocity;

			syncDynamicReconfigure();

			talon_->setMotionCruiseVelocity(params_.motion_cruise_velocity_);
		}

		virtual void setMotionAcceleration(double acceleration)
		{
			if (acceleration == params_.motion_acceleration_)
				return;
			params_.motion_acceleration_ = acceleration;

			syncDynamicReconfigure();

			talon_->setMotionAcceleration(params_.motion_acceleration_);
		}

		virtual void setMotionSCurveStrength(unsigned int s_curve_strength)
		{
			if (s_curve_strength == params_.motion_s_curve_strength_)
				return;
			params_.motion_s_curve_strength_ = s_curve_strength;

			syncDynamicReconfigure();

			talon_->setMotionSCurveStrength(params_.motion_s_curve_strength_);
		}

		virtual double getMotionCruiseVelocity(void)
		{
			return params_.motion_cruise_velocity_;
		}

		virtual double getMotionAcceleration(void)
		{
			return params_.motion_acceleration_;
		}

		virtual void setStatusFramePeriod(hardware_interface::StatusFrame status_frame, uint8_t period)
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

		virtual void setControlFramePeriod(hardware_interface::ControlFrame control_frame, uint8_t period)
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

		virtual void setMotionProfileTrajectoryPeriod(int msec)
		{
			if (msec == params_.motion_profile_trajectory_period_)
				return;
			params_.motion_profile_trajectory_period_ = msec;

			syncDynamicReconfigure();

			talon_->setMotionProfileTrajectoryPeriod(params_.motion_profile_trajectory_period_);
		}

		virtual void clearMotionProfileTrajectories(void)
		{
			talon_->setClearMotionProfileTrajectories();
		}

		virtual void clearMotionProfileHasUnderrun(void)
		{
			talon_->setClearMotionProfileHasUnderrun();
		}

		virtual void pushMotionProfileTrajectory(const hardware_interface::TrajectoryPoint &traj_pt)
		{
			talon_->PushMotionProfileTrajectory(traj_pt);
		}

		double getPosition(void) const
		{
			return talon_.state()->getPosition();
		}

		double getSpeed(void) const
		{
			return talon_.state()->getSpeed();
		}

		bool getForwardLimitSwitch(void) const
		{
			return talon_.state()->getForwardLimitSwitch();
		}
		bool getReverseLimitSwitch(void) const
		{
			return talon_.state()->getReverseLimitSwitch();
		}

		void setPeakOutputForward(double peak)
		{
			if (peak == params_.peak_output_forward_)
				return;
			params_.peak_output_forward_ = peak;

			syncDynamicReconfigure();
			talon_->setPeakOutputForward(peak);
		}

		void setPeakOutputReverse(double peak)
		{
			if (peak == params_.peak_output_reverse_)
				return;
			params_.peak_output_reverse_ = peak;

			syncDynamicReconfigure();
			talon_->setPeakOutputReverse(peak);
		}

		void setClosedloopRamp(double closed_loop_ramp)
		{
			if (closed_loop_ramp == params_.closed_loop_ramp_)
				return;
			params_.closed_loop_ramp_ = closed_loop_ramp;

			syncDynamicReconfigure();

			talon_->setClosedloopRamp(params_.closed_loop_ramp_);
		}

		void setOpenloopRamp(double open_loop_ramp)
		{
			if (open_loop_ramp == params_.open_loop_ramp_)
				return;
			params_.open_loop_ramp_ = open_loop_ramp;

			syncDynamicReconfigure();

			talon_->setOpenloopRamp(params_.open_loop_ramp_);
		}

		void setDemand1Type(hardware_interface::DemandType demand1_type)
		{
			if (demand1_type == params_.demand1_type_)
				return;
			params_.demand1_type_ = demand1_type;
			syncDynamicReconfigure();
			talon_->setDemand1Type(demand1_type);
		}

		void setDemand1Value(double demand1_value)
		{
			if (demand1_value == params_.demand1_value_)
				return;
			params_.demand1_value_ = demand1_value;
			syncDynamicReconfigure();
			talon_->setDemand1Value(demand1_value);
		}

		virtual void setCustomProfileHz(const double &hz)
		{
			if (hz == params_.custom_profile_hz_)
                return;
            params_.custom_profile_hz_ = hz;

            syncDynamicReconfigure();
			talon_->setCustomProfileHz(params_.custom_profile_hz_);
		}

		double getCustomProfileHz(void) const
		{
			return params_.custom_profile_hz_;
		}

		virtual void setCustomProfileRun(const bool &run)
        {
			talon_->setCustomProfileRun(run);
        }

        bool getCustomProfileRun(void)
        {
			return talon_->getCustomProfileRun();
        }

        virtual void setCustomProfileNextSlot(const std::vector<int> &next_slot)
        {
            talon_->setCustomProfileNextSlot(next_slot);
        }

        std::vector<int> getCustomProfileNextSlot(void)
        {
			return talon_->getCustomProfileNextSlot();
        }

        virtual void setCustomProfileSlot(const int &slot)
        {
            talon_->setCustomProfileSlot(slot);
        }

        int getCustomProfileSlot(void)
        {
			return talon_->getCustomProfileSlot();
        }

        void pushCustomProfilePoint(const hardware_interface::CustomProfilePoint &point, int slot)
        {
            talon_->pushCustomProfilePoint(point, slot);
        }

        void pushCustomProfilePoints(const std::vector<hardware_interface::CustomProfilePoint> &points, int slot)
        {
            talon_->pushCustomProfilePoints(points, slot);
        }

        void overwriteCustomProfilePoints(const std::vector<hardware_interface::CustomProfilePoint> &points, int slot)
        {
            talon_->overwriteCustomProfilePoints(points, slot);
        }

		hardware_interface::CustomProfileStatus getCustomProfileStatus(void)
		{
			return talon_.state()->getCustomProfileStatus();
		}

		//Does the below function need to be accessable?
		//#if 0
        std::vector<hardware_interface::CustomProfilePoint> getCustomProfilePoints(int slot) /*const*/ //TODO, can be const?
        {
            return talon_->getCustomProfilePoints(slot);
        }
		//#endif

	protected:
		hardware_interface::TalonCommandHandle                          talon_;
		TalonCIParams                                                   params_;
		std::shared_ptr<dynamic_reconfigure::Server<TalonConfigConfig>> srv_;
		std::shared_ptr<boost::recursive_mutex>                         srv_mutex_;

		// List of follower talons associated with the master
		// listed above
		std::vector<hardware_interface::TalonCommandHandle>                          follower_talons_;
		std::vector<std::shared_ptr<dynamic_reconfigure::Server<TalonConfigConfig>>> follower_srvs_;
		std::vector<std::shared_ptr<boost::recursive_mutex>>                         follower_srv_mutexes_;

		// Used to set initial (and only) talon
		// mode for FixedMode derived classes
		virtual bool setInitialMode(void)
		{
			ROS_INFO_STREAM("Talon " << talon_.getName() << " Base class setInitialMode");
			return true;
		}

	private :
		virtual bool init(hardware_interface::TalonCommandInterface *tci,
							ros::NodeHandle &n,
							hardware_interface::TalonCommandHandle &talon,
							std::shared_ptr<boost::recursive_mutex> &srv_mutex,
							std::shared_ptr<dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig>> &srv,
							bool update_params)
		{
			ROS_WARN("init start");
			// Read params from startup and intialize Talon using them
			TalonCIParams params;
			if (!readParams(n, params))
			   return false;
			bool dynamic_reconfigure;
			n.param<bool>("dynamic_reconfigure", dynamic_reconfigure, false);
			ROS_WARN("init past readParams");

			talon = tci->getHandle(params.joint_name_);
			writeParamsToHW(params, talon, update_params);

			ROS_WARN("init past writeParamsToHW");
			if (dynamic_reconfigure)
			{
				// Create dynamic_reconfigure Server. Pass in n
				// so that all the vars for the class are grouped
				// under the node's name.  Doing so allows multiple
				// copies of the class to be started, each getting
				// their own namespace.
				srv_mutex = std::make_shared<boost::recursive_mutex>();
				srv = std::make_shared<dynamic_reconfigure::Server<talon_controllers::TalonConfigConfig>>(*srv_mutex_, n);

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
			}
			ROS_WARN("init returning");

			return true;
		}

		// If dynamic reconfigure is running then update
		// the reported config there with the new internal
		// state
		void syncDynamicReconfigure(void)
		{
			if (srv_)
			{
				TalonConfigConfig config(params_.toConfig());
				// first call in updateConfig is another lock, this is probably
				// redundant
				// boost::recursive_mutex::scoped_lock lock(*srv_mutex_);
				srv_->updateConfig(config);
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
		virtual void writeParamsToHW(const TalonCIParams &params,
				hardware_interface::TalonCommandHandle talon,
				bool update_params = true)
		{
			talon->lock();
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

			talon->setPeakCurrentLimit(params.current_limit_peak_amps_);
			talon->setPeakCurrentDuration(params.current_limit_peak_msec_);
			talon->setContinuousCurrentLimit(params.current_limit_continuous_amps_);
			talon->setCurrentLimitEnable(params.current_limit_enable_);

			talon->setMotionCruiseVelocity(params.motion_cruise_velocity_);
			talon->setMotionAcceleration(params.motion_acceleration_);
			talon->setMotionProfileTrajectoryPeriod(params.motion_profile_trajectory_period_);
			for (int i = hardware_interface::Status_1_General; i < hardware_interface::Status_Last; i++)
				talon->setStatusFramePeriod(static_cast<hardware_interface::StatusFrame>(i), params.status_frame_periods_[i]);
			for (int i = hardware_interface::Control_3_General; i < hardware_interface::Control_Last; i++)
				talon->setControlFramePeriod(static_cast<hardware_interface::ControlFrame>(i), params.control_frame_periods_[i]);

			talon->setConversionFactor(params.conversion_factor_);

			talon->setCustomProfileHz(params.custom_profile_hz_);

			talon->setEnableReadThread(params.enable_read_thread_);

			// Save copy of params written to HW
			// so they can be queried later?
			if (update_params)
				params_ = params;
			talon->unlock();
		}
};

// A derived class which disables mode switching. Any
// single-mode CI class should derive from this class
class TalonFixedModeControllerInterface : public TalonControllerInterface
{
	protected:
		// Disable changing mode for controllers derived from this class
		void setMode(hardware_interface::TalonMode /*mode*/) override
		{
			ROS_WARN("Can't reset mode using this TalonControllerInterface");
		}
};

class TalonPercentOutputControllerInterface : public TalonFixedModeControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_PercentOutput);
			talon_->setCustomProfileDisable(true);
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Percent Output mode");
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
};

class TalonFollowerControllerInterface : public TalonFixedModeControllerInterface
{
	public:
		bool initWithNode(hardware_interface::TalonCommandInterface *tci,
						  hardware_interface::TalonStateInterface   *tsi,
						  ros::NodeHandle &n) override
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
			talon_->setCustomProfileDisable(true);

			ROS_INFO_STREAM("Launching follower " << params_.joint_name_ << " to follow CAN ID " << follow_can_id << " (" << follow_handle.getName() << ")");
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
		void setCommand(const double /*command*/) override
		{
			ROS_WARN("Can't set a command in follower mode!");
		}
};

// Use this to create any methods common to all
// Close Loop modes, if any
class TalonCloseLoopControllerInterface : public TalonFixedModeControllerInterface
{
};

class TalonPositionCloseLoopControllerInterface : public TalonCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to position close loop mode
			talon_->setMode(hardware_interface::TalonMode_Position);

			//RG: We should consider setting the PID config for position to default to 1
			//Sorting PID values based on type (position vs velocity) seems like a good idea and
			//having a position and a velocity mode is relatively common for drive trains
			//In other cases it will make it clearer how the PID vals are used
			//KCJ - works for now, at least until we get virtual PID config slots going...
			//      at that point my hope is to have named PID configs rather than numbers
			//      and then add initial PID mode as one of the yaml params
			setPIDFSlot(1); // pick a default?
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Close Loop Position mode");

			return true;
		}
};

class TalonVelocityCloseLoopControllerInterface : public TalonCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			talon_->setMode(hardware_interface::TalonMode_Velocity);
			setPIDFSlot(0); // pick a default?
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Close Loop Velocity mode");

			return true;
		}
};

class TalonCurrentControllerCloseLoopInterface : public TalonCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to current close loop mode
			talon_->setMode(hardware_interface::TalonMode_Current);
			setPIDFSlot(0); // pick a default?
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in Close Loop Current mode");
			return true;
		}
};

class TalonMotionProfileControllerInterface : public TalonCloseLoopControllerInterface // double check that this works
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_MotionProfile);
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in MotionProfile mode");
			talon_->setCustomProfileDisable(true);
			return true;
		}
};

//RG: I can think of few to no situations were we would have a talon in motion magic mode for an entire match
//Honestly I wouldn't ever use motion magic mode, I would use the MotionProfile mode (above)
// KCJ -- in general the code we actually use will get a lot more attention. Not sure if that
// means we should pull out less-tested stuff like this or leave it in and fix it if
// we need it at some point?
class TalonMotionMagicCloseLoopControllerInterface : public TalonCloseLoopControllerInterface // double check that this works
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			talon_->setMode(hardware_interface::TalonMode_MotionMagic);
			ROS_INFO_STREAM("Set up talon " << talon_.getName() << " in MotionMagic mode");
			talon_->setCustomProfileDisable(true);
			return true;
		}
};

} // namespace
