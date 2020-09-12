/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Original Author: Dave Coleman
Desc:   Example ros_control hardware interface blank template for the FRCRobot
For a more detailed simulation example, see sim_hw_interface.cpp
*/
#include <ros/ros.h>

#include "frc/DriverStation.h"
#include "mockdata/DIOData.h"
#include "mockdata/DriverStationData.h"

#include <ros_control_boilerplate/frcrobot_sim_interface.h>
#include <ros_control_boilerplate/nextVelocity.h>
#include <ros_control_boilerplate/set_limit_switch.h>


namespace frcrobot_control
{

FRCRobotSimInterface::FRCRobotSimInterface(ros::NodeHandle &nh,
		urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
}
FRCRobotSimInterface::~FRCRobotSimInterface()
{
}

void FRCRobotSimInterface::match_data_callback(const frc_msgs::MatchSpecificData &match_data) {
	std::lock_guard<std::mutex> l(match_data_mutex_);
	HALSIM_SetDriverStationMatchTime(match_data.matchTimeRemaining);
	HAL_AllianceStationID alliance_station_id = HAL_AllianceStationID_kRed1;
	if (match_data.allianceColor == frc::DriverStation::kRed)
	{
		if (match_data.driverStationLocation == 1)
		{
			alliance_station_id = HAL_AllianceStationID_kRed1;
		}
		else if (match_data.driverStationLocation == 2)
		{
			alliance_station_id = HAL_AllianceStationID_kRed2;
		}
		else if (match_data.driverStationLocation == 3)
		{
			alliance_station_id = HAL_AllianceStationID_kRed3;
		}
	}
	else if (match_data.allianceColor == frc::DriverStation::kBlue)
	{
		if (match_data.driverStationLocation == 1)
		{
			alliance_station_id = HAL_AllianceStationID_kBlue1;
		}
		else if (match_data.driverStationLocation == 2)
		{
			alliance_station_id = HAL_AllianceStationID_kBlue2;
		}
		else if (match_data.driverStationLocation == 3)
		{
			alliance_station_id = HAL_AllianceStationID_kBlue3;
		}
	}
	HALSIM_SetDriverStationAllianceStationId(alliance_station_id);
	HALSIM_SetDriverStationEnabled(match_data.Enabled);
	HALSIM_SetDriverStationAutonomous(match_data.Autonomous);
	HALSIM_SetDriverStationDsAttached(match_data.DSAttached);
	HALSIM_SetDriverStationFmsAttached(match_data.FMSAttached);
	HALSIM_SetDriverStationTest(match_data.Test);
	HALSIM_SetDriverStationEStop(match_data.EStopped);
	// TODO - HALSIM_SetDriverStationBatteryVoltage(match_data.BatteryVoltage);

	HAL_MatchInfo hal_match_info;
	strncpy(hal_match_info.eventName, match_data.eventName.c_str(), sizeof(hal_match_info.eventName));
	hal_match_info.matchType = static_cast<HAL_MatchType>(match_data.matchType);
	hal_match_info.matchNumber = match_data.matchNumber;
	hal_match_info.replayNumber = match_data.replayNumber;
	for (size_t i = 0; i < std::min(match_data.gameSpecificData.size(), sizeof(hal_match_info.gameSpecificMessage)); i++)
	{
		hal_match_info.gameSpecificMessage[i] = match_data.gameSpecificData[i];
	}

	hal_match_info.gameSpecificMessageSize = match_data.gameSpecificData.size();
	HALSIM_SetMatchInfo(&hal_match_info);
}

void FRCRobotSimInterface::joystickCallback(const sensor_msgs::JoyConstPtr &msg, int32_t joystick_num) {
	std::lock_guard<std::mutex> l(joystick_mutex_);

	HAL_JoystickAxes hal_axes;
	hal_axes.count = std::min(msg->axes.size(), 6UL); // the last two entries (6,7) are for POV
	std::memset(hal_axes.axes, 0, sizeof(hal_axes.axes));
	for(int i = 0; i < hal_axes.count; i++)
	{
		hal_axes.axes[i] = msg->axes[i];
	}

	HAL_JoystickButtons hal_buttons;
	hal_buttons.count = std::min(msg->buttons.size(), 32UL); // DriverStationGui.cpp used 32, comment below says 16?
	hal_buttons.buttons = 0;
	for(size_t i = 0; i < msg->buttons.size(); i++)
	{
		// TODO This is probably so wrong
		// GenericHID.h : The buttons are returned in a single 16 bit
		// value with one bit representing the state of each button
		hal_buttons.buttons = ((msg->buttons[i] ? 1 : 0) << i) | hal_buttons.buttons;
	}

	if (msg->axes.size() >= 8)
	{
		HAL_JoystickPOVs hal_povs;
		hal_povs.count = 1;
		std::memset(hal_povs.povs, -1, sizeof(hal_povs.povs));
		//TODO Do we have a standard epsilon somewhere in here?
		//TODO - also check see if it needs to be < -1e-5
		const bool direction_left = msg->axes[6] > 1e-5;
		const bool direction_right = msg->axes[6] < -1e-5;
		const bool direction_up = msg->axes[7] > 1e-5;
		const bool direction_down = msg->axes[7] < -1e-5;

		if(direction_up && !direction_left && !direction_right)
		{
			hal_povs.povs[0] = 0;
		}
		else if(direction_up && direction_right)
		{
			hal_povs.povs[0] = 45;
		}
		else if(!direction_up && !direction_down && direction_right)
		{
			hal_povs.povs[0] = 90;
		}
		else if(direction_down && direction_right)
		{
			hal_povs.povs[0] = 135;
		}
		else if(direction_down && !direction_left && !direction_right)
		{
			hal_povs.povs[0] = 180;
		}
		else if(direction_down && direction_left)
		{
			hal_povs.povs[0] = 225;
		}
		else if(!direction_up && !direction_down && direction_left)
		{
			hal_povs.povs[0] = 270;
		}
		else if(direction_up && direction_left)
		{
			hal_povs.povs[0] = 315;
		}
		HALSIM_SetJoystickPOVs(joystick_num, &hal_povs);
	}
	//TODO check default pov?
	//TODO do you need to set JoystickDescriptor?

	HALSIM_SetJoystickAxes(joystick_num, &hal_axes);
	HALSIM_SetJoystickButtons(joystick_num,
			&hal_buttons);
}

bool FRCRobotSimInterface::setlimit(ros_control_boilerplate::set_limit_switch::Request &req,ros_control_boilerplate::set_limit_switch::Response &/*res*/)
{
	for (std::size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		if (!can_ctre_mc_local_hardwares_[joint_id])
			continue;
        auto &ts = talon_state_[joint_id];
        if((!req.target_joint_name.length() && (ts.getCANID() == req.target_joint_id)) ||
		   (req.target_joint_name == can_ctre_mc_names_[joint_id]))
		{
            ts.setForwardLimitSwitch(req.forward);
			ts.setReverseLimitSwitch(req.reverse);
        }
    }
	return true;
}

bool FRCRobotSimInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	ROS_WARN("Passes");
	if (!FRCRobotInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " failed");
		return false;
	}
	ROS_WARN("Passes");

	ROS_WARN("fails here?1");
	// Loop through the list of joint names
	// specified as params for the hardware_interface.
	// For each of them, create a Talon object. This
	// object is used to send and recieve commands
	// and status to/from the physical Talon motor
	// controller on the robot.  Use this pointer
	// to initialize each Talon with various params
	// set for that motor controller in config files.
	// TODO : assert can_ctre_mc_names_.size() == can_ctre_mc_can_ids_.size()

	limit_switch_srv_ = root_nh.advertiseService("set_limit_switch",&FRCRobotSimInterface::setlimit,this);
    match_data_sub_ = root_nh.subscribe("/frcrobot_rio/match_data_in", 1, &FRCRobotSimInterface::match_data_callback, this);
    //TODO fix joystick topic
	for (size_t i = 0; i < HAL_kMaxJoysticks; i++)
	{
		std::stringstream s;
		s << "js" << i << "_in";
		joystick_subs_.push_back(root_nh.subscribe<sensor_msgs::Joy>(s.str(), 1, boost::bind(&FRCRobotSimInterface::joystickCallback, this, _1, i)));
	}

	linebreak_sensor_srv_ = root_nh.advertiseService("linebreak_service_set",&FRCRobotSimInterface::evaluateDigitalInput, this);
	ROS_INFO_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready.");
	for (size_t i = 0; i < can_ctre_mc_names_.size(); i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << can_ctre_mc_names_[i] <<
							  (can_ctre_mc_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (can_ctre_mc_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as " << (can_ctre_mc_is_talon_fx_[i] ? "TalonFX" : (can_ctre_mc_is_talon_srx_[i] ? "TalonSRX" : "VictorSPX"))
							  << " CAN id " << can_ctre_mc_can_ids_[i]);

		ROS_WARN_STREAM("fails here? 56789: " << i);
		// Loop through the list of joint names

	}
	for (size_t i = 0; i < num_canifiers_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << canifier_names_[i] <<
							  (canifier_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (canifier_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << canifier_can_ids_[i]);
	}
	for (size_t i = 0; i < num_cancoders_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << cancoder_names_[i] <<
							  (cancoder_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (cancoder_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << cancoder_can_ids_[i]);
	}

	// TODO - merge me into frc robot interface, add a sim setting, etc.
	for (size_t i = 0; i < num_as726xs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading as726x joint " << i << "=" << as726x_names_[i] <<
							  (as726x_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (as726x_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as as726x with port=" << as726x_ports_[i] <<
							  " address=" << as726x_addresses_[i]);
		if (as726x_local_hardwares_[i])
		{
			if ((as726x_ports_[i] != "onboard") && (as726x_ports_[i] != "mxp"))
			{
				ROS_ERROR_STREAM("Invalid port specified for as726x - " <<
						as726x_ports_[i] << "valid options are onboard and mxp");
				return false;
			}
		}
	}
	for (size_t i = 0; i < num_talon_orchestras_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << talon_orchestra_names_[i]);
	}

	ROS_INFO_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready.");
	return true;
}

void FRCRobotSimInterface::read(const ros::Time& time, const ros::Duration& period)
{
	FRCRobotInterface::read(time, period);
	for (size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		// Do nothing
    }
}

bool FRCRobotSimInterface::evaluateDigitalInput(ros_control_boilerplate::LineBreakSensors::Request &req,
							ros_control_boilerplate::LineBreakSensors::Response &/*res*/)
{
	if (req.name.length())
	{
		for (size_t i = 0; i < digital_input_names_.size(); i++)
		{
			if (digital_input_names_[i] == req.name)
			{
				HALSIM_SetDIOValue(digital_input_dio_channels_[i], req.value ^ digital_input_inverts_[i]);
				ROS_INFO_STREAM(digital_input_names_[i] << " set to " << (int)req.value);
				break;
			}
		}
	}
	else if (req.j < digital_input_names_.size())
	{
		HALSIM_SetDIOValue(digital_input_dio_channels_[req.j], req.value ^ digital_input_inverts_[req.j]);
		ROS_INFO_STREAM(digital_input_names_[req.j] << " set to " << (int)req.value);
	}
	else
	{
		ROS_INFO_STREAM(digital_input_names_[req.j] << " not set to " << (int)req.value << " index out of bounds");
	}
	return true;
}

void FRCRobotSimInterface::write(const ros::Time& time, const ros::Duration& period)
{
	FRCRobotInterface::write(time, period);
	// Was the robot enabled last time write was run?
	static bool last_robot_enabled = false;

	// Is match data reporting the robot enabled now?
	bool robot_enabled = false;
	{
		std::unique_lock<std::mutex> l(match_data_mutex_, std::try_to_lock);
		if (l.owns_lock())
			robot_enabled = match_data_.isEnabled();
		else
			robot_enabled = last_robot_enabled;
	}

	for (size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		if (!can_ctre_mc_local_hardwares_[joint_id])
			continue;

		if (!talon_command_[joint_id].try_lock())
			continue;

		custom_profile_write(joint_id);

		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];

		// If commanded mode changes, copy it over
		// to current state
		hardware_interface::TalonMode new_mode = tc.getMode();

		// Only set mode to requested one when robot is enabled
		if (robot_enabled)
		{
			if (tc.modeChanged(new_mode))
			{
				ts.setTalonMode(new_mode);
				ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" mode " << (int)new_mode);
			}
			hardware_interface::DemandType demand1_type_internal;
			double demand1_value;
			if (tc.demand1Changed(demand1_type_internal, demand1_value))
			{
				ts.setDemand1Type(demand1_type_internal);
				ts.setDemand1Value(demand1_value);

				ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" demand1 type / value");
			}
		}
		else
		{
			// Update talon state with requested setpoints for
			// debugging. Don't actually write them to the physical
			// Talons until the robot is re-enabled, though.
			ts.setSetpoint(tc.get());
			ts.setDemand1Type(tc.getDemand1Type());
			ts.setDemand1Value(tc.getDemand1Value());
			if (last_robot_enabled)
			{
				// On the switch from robot enabled to robot disabled, set Talons to ControlMode::Disabled
				// call resetMode() to queue up a change back to the correct mode / setpoint
				// when the robot switches from disabled back to enabled
				tc.resetMode();
				tc.resetDemand1();
				ts.setTalonMode(hardware_interface::TalonMode_Disabled);
				ROS_INFO_STREAM("Robot disabled - called Set(Disabled) on " << joint_id << "=" << can_ctre_mc_names_[joint_id]);
			}
		}

		bool close_loop_mode = false;
		bool motion_profile_mode = false;

		// Use requested next talon mode here to update
		// Talon config.  This way settings will be written
		// even if the robot is disabled.  It will also insure
		// that config relevant to the requested mode is
		// written before switching to that mode.
		if ((new_mode == hardware_interface::TalonMode_Position) ||
		    (new_mode == hardware_interface::TalonMode_Velocity) ||
		    (new_mode == hardware_interface::TalonMode_Current ))
		{
			close_loop_mode = true;
		}
		else if ((new_mode == hardware_interface::TalonMode_MotionProfile) ||
			     (new_mode == hardware_interface::TalonMode_MotionMagic))
		{
			close_loop_mode = true;
			motion_profile_mode = true;
		}

		hardware_interface::FeedbackDevice internal_feedback_device;
		double feedback_coefficient;
		if (tc.encoderFeedbackChanged(internal_feedback_device, feedback_coefficient))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " feedback");
			ts.setEncoderFeedback(internal_feedback_device);
			ts.setFeedbackCoefficient(feedback_coefficient);
		}

		// Only update PID settings if closed loop
		// mode has been requested
		if (close_loop_mode)
		{
			int slot;
			const bool slot_changed = tc.slotChanged(slot);

			double p;
			double i;
			double d;
			double f;
			int   iz;
			int   allowable_closed_loop_error;
			double max_integral_accumulator;
			double closed_loop_peak_output;
			int    closed_loop_period;
			if (tc.pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, closed_loop_peak_output, closed_loop_period, slot))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" PIDF slot " << slot << " config values");
				ts.setPidfP(p, slot);
				ts.setPidfI(i, slot);
				ts.setPidfD(d, slot);
				ts.setPidfF(f, slot);
				ts.setPidfIzone(iz, slot);
				ts.setAllowableClosedLoopError(allowable_closed_loop_error, slot);
				ts.setMaxIntegralAccumulator(max_integral_accumulator, slot);
				ts.setClosedLoopPeakOutput(closed_loop_peak_output, slot);
				ts.setClosedLoopPeriod(closed_loop_period, slot);
			}

			if (slot_changed)
			{
				ts.setSlot(slot);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " PIDF slot to " << slot);
			}
		}
		// Invert / sensor phase matters for all modes
		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" invert / phase");
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
		}

		hardware_interface::NeutralMode neutral_mode;
		if (tc.neutralModeChanged(neutral_mode))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" neutral mode");
			ts.setNeutralMode(neutral_mode);
		}

		if (tc.neutralOutputChanged())
		{
			ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" neutral output");
			ts.setNeutralOutput(true);
		}

		double iaccum;
		if (close_loop_mode && tc.integralAccumulatorChanged(iaccum))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" integral accumulator");
		}

		double closed_loop_ramp;
		double open_loop_ramp;
		double peak_output_forward;
		double peak_output_reverse;
		double nominal_output_forward;
		double nominal_output_reverse;
		double neutral_deadband;
		if (tc.outputShapingChanged(closed_loop_ramp,
									open_loop_ramp,
									peak_output_forward,
									peak_output_reverse,
									nominal_output_forward,
									nominal_output_reverse,
									neutral_deadband))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" output shaping");
			ts.setOpenloopRamp(open_loop_ramp);
			ts.setClosedloopRamp(closed_loop_ramp);
			ts.setPeakOutputForward(peak_output_forward);
			ts.setPeakOutputReverse(peak_output_reverse);
			ts.setNominalOutputForward(nominal_output_forward);
			ts.setNominalOutputReverse(nominal_output_reverse);
			ts.setNeutralDeadband(neutral_deadband);
		}
		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.voltageCompensationChanged(v_c_saturation,
									v_measurement_filter,
									v_c_enable))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" voltage compensation");
			ts.setVoltageCompensationSaturation(v_c_saturation);
			ts.setVoltageMeasurementFilter(v_measurement_filter);
			ts.setVoltageCompensationEnable(v_c_enable);
		}

		if (can_ctre_mc_is_talon_fx_[joint_id] || can_ctre_mc_is_talon_srx_[joint_id])
		{
			hardware_interface::VelocityMeasurementPeriod v_m_period;
			int v_m_window;

			if (tc.velocityMeasurementChanged(v_m_period, v_m_window))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" velocity measurement period / window");
				ts.setVelocityMeasurementPeriod(v_m_period);
				ts.setVelocityMeasurementWindow(v_m_window);
			}
		}

		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" selected sensor position");
			ts.setPosition(sensor_position);
		}

		if (can_ctre_mc_is_talon_fx_[joint_id] || can_ctre_mc_is_talon_srx_[joint_id])
		{
			hardware_interface::LimitSwitchSource internal_local_forward_source;
			hardware_interface::LimitSwitchNormal internal_local_forward_normal;
			hardware_interface::LimitSwitchSource internal_local_reverse_source;
			hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
			if (tc.limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
						internal_local_reverse_source, internal_local_reverse_normal))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" limit switches");
				ts.setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
				ts.setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
			}
		}

		double softlimit_forward_threshold;
		bool softlimit_forward_enable;
		double softlimit_reverse_threshold;
		bool softlimit_reverse_enable;
		bool softlimit_override_enable;
		if (tc.softLimitChanged(softlimit_forward_threshold,
				softlimit_forward_enable,
				softlimit_reverse_threshold,
				softlimit_reverse_enable,
				softlimit_override_enable))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" soft limits " <<
					std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
					std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
					std::endl << "\toverride_enable=" << softlimit_override_enable);
			ts.setForwardSoftLimitThreshold(softlimit_forward_threshold);
			ts.setForwardSoftLimitEnable(softlimit_forward_enable);
			ts.setReverseSoftLimitThreshold(softlimit_reverse_threshold);
			ts.setReverseSoftLimitEnable(softlimit_reverse_enable);
			ts.setOverrideSoftLimitsEnable(softlimit_override_enable);
		}

		if (can_ctre_mc_is_talon_srx_[joint_id])
		{
			int peak_amps;
			int peak_msec;
			int continuous_amps;
			bool enable;
			if (tc.currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" peak current");
				ts.setPeakCurrentLimit(peak_amps);
				ts.setPeakCurrentDuration(peak_msec);
				ts.setContinuousCurrentLimit(continuous_amps);
				ts.setCurrentLimitEnable(enable);
			}
		}

		if (can_ctre_mc_is_talon_fx_[joint_id])
		{
			double limit;
			double trigger_threshold_current;
			double trigger_threshold_time;
			double limit_enable;
			if (tc.supplyCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " supply current limit");
				ts.setSupplyCurrentLimit(limit);
				ts.setSupplyCurrentLimitEnable(limit_enable);
				ts.setSupplyCurrentTriggerThresholdCurrent(trigger_threshold_current);
				ts.setSupplyCurrentTriggerThresholdTime(trigger_threshold_time);
			}
			if (tc.supplyCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " stator current limit");
				ts.setStatorCurrentLimit(limit);
				ts.setStatorCurrentLimitEnable(limit_enable);
				ts.setStatorCurrentTriggerThresholdCurrent(trigger_threshold_current);
				ts.setStatorCurrentTriggerThresholdTime(trigger_threshold_time);
			}

			hardware_interface::MotorCommutation motor_commutation;
			if (tc.motorCommutationChanged(motor_commutation))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motor commutation");
				ts.setMotorCommutation(motor_commutation);
			}

			hardware_interface::AbsoluteSensorRange absolute_sensor_range;
			if (tc.absoluteSensorRangeChanged(absolute_sensor_range))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
				ts.setAbsoluteSensorRange(absolute_sensor_range);
			}

			hardware_interface::SensorInitializationStrategy sensor_initialization_strategy;
			if (tc.sensorInitializationStrategyChanged(sensor_initialization_strategy))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
				ts.setSensorInitializationStrategy(sensor_initialization_strategy);
			}
		}

		if (can_ctre_mc_is_talon_fx_[joint_id] || can_ctre_mc_is_talon_srx_[joint_id])
		{
			for (int i = hardware_interface::Status_1_General; i < hardware_interface::Status_Last; i++)
			{
				uint8_t period;
				const hardware_interface::StatusFrame status_frame = static_cast<hardware_interface::StatusFrame>(i);
				if (tc.statusFramePeriodChanged(status_frame, period) && (period != 0))
				{
					ts.setStatusFramePeriod(status_frame, period);
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" status_frame " << i << "=" << static_cast<int>(period) << "mSec");
				}
			}
		}

		if (motion_profile_mode)
		{
			double motion_cruise_velocity;
			double motion_acceleration;
			unsigned int motion_s_curve_strength;
			if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration, motion_s_curve_strength))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" cruise velocity / acceleration");
				ts.setMotionCruiseVelocity(motion_cruise_velocity);
				ts.setMotionAcceleration(motion_acceleration);
				ts.setMotionSCurveStrength(motion_s_curve_strength);
			}

			int motion_profile_trajectory_period;
			if (tc.motionProfileTrajectoryPeriodChanged(motion_profile_trajectory_period))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" motion profile trajectory period");
				ts.setMotionProfileTrajectoryPeriod(motion_profile_trajectory_period);
			}

			if (tc.clearMotionProfileTrajectoriesChanged())
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" motion profile trajectories");

			if (tc.clearMotionProfileHasUnderrunChanged())
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" motion profile underrun changed");

			std::vector<hardware_interface::TrajectoryPoint> trajectory_points;

			if (tc.motionProfileTrajectoriesChanged(trajectory_points))
				ROS_INFO_STREAM("Added " << trajectory_points.size() << " points to joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" motion profile trajectories");
		}

		hardware_interface::TalonMode simulate_mode = ts.getTalonMode();
		if (simulate_mode == hardware_interface::TalonMode_Position)
		{
			// Assume instant velocity
			double position;

			if (tc.commandChanged(position))
				ts.setSetpoint(position);

			ts.setPosition(position);
			ts.setSpeed(0);
		}
		else if (simulate_mode == hardware_interface::TalonMode_Velocity)
		{
			// Assume instant acceleration for now
			double speed;

			if (tc.commandChanged(speed))
				ts.setSetpoint(speed);

			ts.setPosition(ts.getPosition() + speed * period.toSec());
			ts.setSpeed(speed);
		}
		else if (simulate_mode == hardware_interface::TalonMode_MotionMagic)
		{
			double setpoint;

			if (tc.commandChanged(setpoint))
				ts.setSetpoint(setpoint);

			const double position = ts.getPosition();
			double velocity = ts.getSpeed();
			const double dt = period.toSec();

			//change the nextVelocity call to non existent as it does not work and throws an error from a non-existent package
			double next_pos = nextVelocity(position, setpoint, velocity, ts.getMotionCruiseVelocity(), ts.getMotionAcceleration(), dt);
			//ROS_WARN_STREAM("max vel: " <<ts.getMotionCruiseVelocity()<< " max accel: " << ts.getMotionAcceleration());

			//Talons don't allow overshoot, the profiling algorithm above does
			if ((position <= setpoint && setpoint < next_pos) || (position >= setpoint && setpoint > next_pos))
			{
				next_pos = setpoint;
				velocity = 0;
			}
			ts.setPosition(next_pos);
			ts.setSpeed(velocity);
		}
		else if (simulate_mode == hardware_interface::TalonMode_Disabled)
		{
			ts.setSpeed(0); // Don't know how to simulate decel, so just pretend we are stopped
		}
		else if (simulate_mode == hardware_interface::TalonMode_PercentOutput)
		{
			double percent;

			if (tc.commandChanged(percent))
				ts.setSetpoint(percent);

			ts.setPosition(ts.getPosition() + percent*2*M_PI * period.toSec());
			ts.setSpeed(percent*2*M_PI);
		}

		if (tc.clearStickyFaultsChanged())
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" sticky_faults");

		tc.unlock();
	}
	last_robot_enabled = robot_enabled;
	for (std::size_t joint_id = 0; joint_id < num_canifiers_; ++joint_id)
	{
		if (!canifier_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &cs = canifier_state_[joint_id];
		auto &cc = canifier_command_[joint_id];

		for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
		{
			const auto led_channel = static_cast<hardware_interface::canifier::LEDChannel>(i);
			double percent_output;
			if (cc.ledOutputChanged(led_channel, percent_output))
			{
				cs.setLEDOutput(led_channel, percent_output);
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set LED channel " << i << " to " << percent_output);
			}
		}
		for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
		{
			const auto general_pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
			bool value;
			bool output_enable;
			if (cc.generalPinOutputChanged(general_pin, value, output_enable))
			{
				cs.setGeneralPinOutput(general_pin, value);
				cs.setGeneralPinOutputEnable(general_pin, output_enable);
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set General Pin " << i <<
						" to enable=" << output_enable << " value=" << value);
			}
		}

		// Don't bother with all the changed/reset code here, just copy
		// from command to state each time through the write call
		cs.setEncoderTicksPerRotation(cc.getEncoderTicksPerRotation());
		cs.setConversionFactor(cc.getConversionFactor());
		double position;
		if (cc.quadraturePositionChanged(position))
		{
			// Don't set state encoder position, let it be read at the next read() call
			ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
					<< " : Set Quadrature Position to " << position);
		}

		hardware_interface::canifier::CANifierVelocityMeasPeriod period;
		if (cc.velocityMeasurementPeriodChanged(period))
		{
			ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
					<< " : Set velocity measurement Period to " << period);
			cs.setVelocityMeasurementPeriod(period);
		}

		int window;
		if (cc.velocityMeasurementWindowChanged(window))
		{
			ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
					<< " : Set velocity measurement window to " << window);
			cs.setVelocityMeasurementWindow(window);
		}

		bool clear_position_on_limit_f;
		if (cc.clearPositionOnLimitFChanged(clear_position_on_limit_f))
		{
			ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
					<< " : Set clear position on limit F to " << clear_position_on_limit_f);
			cs.setClearPositionOnLimitF(clear_position_on_limit_f);
		}

		bool clear_position_on_limit_r;
		if (cc.clearPositionOnLimitRChanged(clear_position_on_limit_r))
		{
			ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
					<< " : Set clear position on limit R to " << clear_position_on_limit_r);
			cs.setClearPositionOnLimitR(clear_position_on_limit_r);
		}

		bool clear_position_on_quad_idx;
		if (cc.clearPositionOnQuadIdxChanged(clear_position_on_quad_idx))
		{
			ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
					<< " : Set clear position on quad idx to " << clear_position_on_quad_idx);
			cs.setClearPositionOnQuadIdx(clear_position_on_quad_idx);
		}

		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
			bool output_enable;
			if (cc.pwmOutputEnableChanged(pwm_channel, output_enable))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set pwm channel " << pwm_channel << " output enable to " << static_cast<int>(output_enable));
				cs.setPWMOutputEnable(pwm_channel, output_enable);
			}
		}

		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
			double output;
			if (cc.pwmOutputChanged(pwm_channel, output))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set pwm channel " << pwm_channel << " output to " << output);
				cs.setPWMOutput(pwm_channel, output);
			}
		}

		for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
		{
			const auto frame_id = static_cast<hardware_interface::canifier::CANifierStatusFrame>(i);
			int period;
			if (cc.statusFramePeriodChanged(frame_id, period))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set frame_id " << i << " status period to " << period);
			}
		}

		for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
		{
			const auto frame_id = static_cast<hardware_interface::canifier::CANifierControlFrame>(i);
			int period;
			if (cc.controlFramePeriodChanged(frame_id, period))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set frame_id " << i << " control period to " << period);
			}
		}

		if (cc.clearStickyFaultsChanged())
		{
			ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id] << " : cleared sticky faults");
		}
	}

	for (std::size_t joint_id = 0; joint_id < num_cancoders_; ++joint_id)
	{
		if (!cancoder_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &cs = cancoder_state_[joint_id];
		auto &cc = cancoder_command_[joint_id];
		cs.setConversionFactor(cc.getConversionFactor());
		double position;
		if (cc.positionChanged(position))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set position to " << position);
			// Don't set state - it will be updated in next read() loop
		}
		if (cc.positionToAbsoluteChanged())
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set position to absolute");
		}
		hardware_interface::cancoder::SensorVelocityMeasPeriod velocity_meas_period;
		if (cc.velocityMeasPeriodChanged(velocity_meas_period))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set velocity measurement period to " << static_cast<int>(velocity_meas_period));
			cs.setVelocityMeasPeriod(velocity_meas_period);
		}

		int velocity_meas_window;
		if (cc.velocityMeasWindowChanged(velocity_meas_window))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set velocity measurement window to " << velocity_meas_window);
			cs.setVelocityMeasWindow(velocity_meas_window);
		}
		hardware_interface::cancoder::AbsoluteSensorRange absolute_sensor_range;
		if (cc.absoluteSensorRangeChanged(absolute_sensor_range))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set absolute sensor range to " << absolute_sensor_range);
			cs.setAbsoluteSensorRange(absolute_sensor_range);
		}

		double magnet_offset;
		if (cc.magnetOffsetChanged(magnet_offset))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set magnet offset to " << magnet_offset);
			cs.setMagnetOffset(magnet_offset);
		}

		hardware_interface::cancoder::SensorInitializationStrategy initialization_strategy;
		if (cc.InitializationStrategyChanged(initialization_strategy))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set sensor intitialization strategy to " << initialization_strategy);
			cs.setInitializationStrategy(initialization_strategy);
		}
		double feedback_coefficient;
		std::string unit_string;
		hardware_interface::cancoder::SensorTimeBase time_base;
		if (cc.feedbackCoefficientChanged(feedback_coefficient, unit_string, time_base))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set feedback coefficient to  " << feedback_coefficient << " " << unit_string << " " << time_base);
			cs.setFeedbackCoefficient(feedback_coefficient);
			cs.setUnitString(unit_string);
			cs.setTimeBase(time_base);
		}

		bool direction;
		if (cc.directionChanged(direction))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set direction to " << direction);
			cs.setDirection(direction);
		}

		int sensor_data_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(sensor_data_status_frame_period))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set sensor data status frame period to " << sensor_data_status_frame_period);
			cs.setSensorDataStatusFramePeriod(sensor_data_status_frame_period);
		}

		int vbat_and_faults_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(vbat_and_faults_status_frame_period))
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
					<< " : Set vbat and fault status frame period to " << vbat_and_faults_status_frame_period);
			cs.setVbatAndFaultsStatusFramePeriod(vbat_and_faults_status_frame_period);
		}

		if (cc.clearStickyFaultsChanged())
		{
			ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id] << " : Sticky faults cleared");
		}
	}

	for (size_t i = 0; i < num_as726xs_; i++)
	{
		auto &as = as726x_state_[i];
		auto &ac = as726x_command_[i];

		hardware_interface::as726x::IndLedCurrentLimits ind_led_current_limit;
		if (ac.indLedCurrentLimitChanged(ind_led_current_limit))
		{
			as.setIndLedCurrentLimit(ind_led_current_limit);
			ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " ind_led_current_limit = " << ind_led_current_limit);
		}

		bool ind_led_enable;
		if (ac.indLedEnableChanged(ind_led_enable))
		{
			as.setIndLedEnable(ind_led_enable);
			ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " ind_led_enable = " << ind_led_enable);
		}

		hardware_interface::as726x::DrvLedCurrentLimits drv_led_current_limit;
		if (ac.drvLedCurrentLimitChanged(drv_led_current_limit))
		{
			as.setDrvLedCurrentLimit(drv_led_current_limit);
			ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " drv_led_current_limit = " << drv_led_current_limit);
		}

		bool drv_led_enable;
		if (ac.drvLedEnableChanged(drv_led_enable))
		{
			as.setDrvLedEnable(drv_led_enable);
			ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " drv_led_enable = " << drv_led_enable);
		}

		hardware_interface::as726x::ConversionTypes conversion_type;
		if (ac.conversionTypeChanged(conversion_type))
		{
			as.setConversionType(conversion_type);
			ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " conversion_type = " << conversion_type);
		}

		hardware_interface::as726x::ChannelGain gain;
		if (ac.gainChanged(gain))
		{
			as.setGain(gain);
			ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " channel_gain = " << gain);
		}

		uint8_t integration_time;
		if (ac.integrationTimeChanged(integration_time))
		{
			as.setIntegrationTime(integration_time);
			ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " integration_time = "
					<< static_cast<int>(integration_time));
		}
	}

	for(size_t i = 0; i < num_talon_orchestras_; i++)
	{
		auto &oc = orchestra_command_[i];
		auto &os = orchestra_state_[i];
		std::string music_file_path;
		std::vector<std::string> instruments;
		if(oc.clearInstrumentsChanged())
		{
			ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " cleared instruments.");
		}
		if(oc.instrumentsChanged(instruments))
		{
				for(size_t j = 0; j < instruments.size(); j++)
				{
					size_t can_index = std::numeric_limits<size_t>::max();
					for(size_t k = 0; k < can_ctre_mc_names_.size(); k++)
					{
						if(can_ctre_mc_names_[k] == instruments[j])
						{
							can_index = k;
							break;
						}
					}
					if(can_index == std::numeric_limits<size_t>::max())
					{
						ROS_ERROR_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " failed to add " << instruments[j] << " because it does not exist");
					}
					else if(can_ctre_mc_is_talon_fx_[can_index])
					{
						ROS_INFO_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " added Falcon " << "falcon_name");
					}
					else
						ROS_INFO_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " failed to add " << instruments[j] << " because it is not a TalonFX");
				}
				os.setInstruments(instruments);
		}
		if(oc.musicChanged(music_file_path))
		{
				os.setChirpFilePath(music_file_path);
				ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " loaded music at " << music_file_path);
		}
		if(oc.pauseChanged())
		{
				os.setPaused();
				ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " pausing");
		}
		if(oc.playChanged())
		{
				os.setPlaying();
				ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " playing");
		}
		if(oc.stopChanged())
		{
				os.setStopped();
				ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " stopping");
		}
	}

	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		if (!dummy_joint_locals_[i])
			continue;
		//s << dummy_joint_command_[i] << " ";
		dummy_joint_effort_[i] = 0;
		//if (dummy_joint_names_[i].substr(2, std::string::npos) == "_angle")
		{
			// position mode
			dummy_joint_velocity_[i] = (dummy_joint_command_[i] - dummy_joint_position_[i]) / period.toSec();
			dummy_joint_position_[i] = dummy_joint_command_[i];
		}
		//else if (dummy_joint_names_[i].substr(2, std::string::npos) == "_drive")
		{
			// position mode
			//dummy_joint_position_[i] += dummy_joint_command_[i] * period.toSec();
			//dummy_joint_velocity_[i] = dummy_joint_command_[i];
		}
	}
	//ROS_INFO_STREAM_THROTTLE(1, s.str());
}

}  // namespace



