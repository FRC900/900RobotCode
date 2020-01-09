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

#include <ros_control_boilerplate/frcrobot_sim_interface.h>
#include <ros_control_boilerplate/nextVelocity.h>
#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define KEYCODE_a 0x61
#define KEYCODE_b 0x62
#define KEYCODE_c 0x63
#define KEYCODE_d 0x64
#define KEYCODE_e 0x65
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6a
#define KEYCODE_k 0x6b
#define KEYCODE_l 0x6c
#define KEYCODE_m 0x6d
#define KEYCODE_n 0x6e
#define KEYCODE_o 0x6f
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_s 0x73
#define KEYCODE_t 0x7
#define KEYCODE_u 0x75
#define KEYCODE_v 0x76
#define KEYCODE_w 0x77
#define KEYCODE_x 0x78
#define KEYCODE_y 0x79
#define KEYCODE_z 0x7a
#define KEYCODE_A 0x41
#define KEYCODE_B 0x42
#define KEYCODE_C 0x43
#define KEYCODE_D 0x44
#define KEYCODE_MINUS 0x2D
#define KEYCODE_EQUALS 0x3D
#define KEYCODE_ONE 0x31
#define KEYCODE_TWO 0x32
#define KEYCODE_THREE 0x33
#define KEYCODE_FOUR 0x34
#define KEYCODE_FIVE 0x35
#define KEYCODE_SIX 0x36
#define KEYCODE_SEVEN 0x37
#define KEYCODE_EIGHT 0x38
#define KEYCODE_LEFT_BRACKET 0x5B
#define KEYCODE_ESCAPE  0x1B
#define KEYCODE_CARROT 0x5E
#define KEYCODE_SPACE 0x20
#define KEYCODE_COMMA 0x2C

#include <ros_control_boilerplate/set_limit_switch.h>

namespace frcrobot_control
{

// TODO : Turn into a separate node - make it launched
// depending on sim mode being true and the key_or_joy setting
TeleopJointsKeyboard::TeleopJointsKeyboard(ros::NodeHandle &nh)
{
	// Hard-code this to frcrobot_rio namespace so that it matches
	// the real robot hardware, where joystick data comes from the
	// driver station via the Rio
	joints_pub_ = nh.advertise<sensor_msgs::Joy>("/frcrobot_rio/joystick_states_raw_key", 1);
}

TeleopJointsKeyboard::~TeleopJointsKeyboard()
{
}

// Code which waits for a set period of time for a keypress.  If
// the keyboard is pressed in that time, read the key press and set
// it equal to c, then return 1 character read.  If nothing is seen,
// return 0.  Return <0 on error.
int TeleopJointsKeyboard::pollKeyboard(int kfd, char &c) const
{
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 20*1000; // 20 mSec keyboard polling rate

	// read_fds is an array of file descriptors to wait for
	// In this case, we only want to wait for 1 - the keyboard
	// fd passed in as kfd
	fd_set read_fds;
	FD_ZERO(&read_fds);
	FD_SET(kfd, &read_fds);

	// Select returns when
	//   1. there is data present on one of the requested fds (returns > 0)
	//   2. the timeout is exceeded (returns 0)
	int rc = select(kfd + 1, &read_fds, NULL, NULL, &tv);
	if (rc < 0)
	{
		perror("select():");
	}
	else if (rc > 0)  // if select didn't timeout
	{
		rc = read(kfd, &c, 1);
		if (rc < 0)
		{
			perror("read():");
		}
	}
	return rc;
}

void TeleopJointsKeyboard::keyboardLoop()
{
	int kfd = 0; // stdin
	char c;
	// get the console in raw mode
	struct termios cooked;
	tcgetattr(kfd, &cooked);
	struct termios raw;
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	// Set read() to return immediately so we
	// can break out of the loop immediately rather than waiting
	// on a timeout from reading stdin
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;

	tcsetattr(kfd, TCSANOW, &raw);

	cmd_.axes.resize(8);
	cmd_.buttons.resize(11);

	cmd_.axes[0] = 0.0;
	cmd_.axes[1] = 0.0;
	cmd_.axes[2] = 0.0;
	cmd_.axes[3] = 0.0;
	cmd_.axes[4] = 0.0;
	cmd_.axes[5] = 0.0;
	cmd_.axes[6] = 0.0;
	cmd_.axes[7] = 0.0;

	cmd_.buttons[0] = false;
	cmd_.buttons[1] = false;
	cmd_.buttons[2] = false;
	cmd_.buttons[3] = false;
	cmd_.buttons[4] = false;
	cmd_.buttons[5] = false;
	cmd_.buttons[6] = false;
	cmd_.buttons[7] = false;
	cmd_.buttons[8] = false;
	cmd_.buttons[9] = false;
	cmd_.buttons[10] = false;

	bool processing_bracket = false;
	bool dirty = false;

	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		// Publish command
		if (!dirty)
		{
			cmd_.header.stamp = ros::Time::now();
			joints_pub_.publish(cmd_);
		}

		loop_rate.sleep();

		int rc = pollKeyboard(kfd, c);
		if (rc < 0)
			break;
		else if (rc == 0)
			continue;

		// Assume keypress will be a valid command / cause
		// to update the published joystick data. Set this to false
		// for cases where it isn't true.
		dirty = false;

		if (!processing_bracket)
		{
			ROS_INFO_STREAM("Processing axes and buttons");
			switch (c)
			{
				case KEYCODE_a:
					if(cmd_.axes[0] > -1.0)
					{
						cmd_.axes[0] -= 0.5;
					}
					break;
				case KEYCODE_d:
					if(cmd_.axes[0] < 1.0)
					{
						cmd_.axes[0] += 0.5;
					}
					break;
				case KEYCODE_w:
					if(cmd_.axes[1] < 1.0)
					{
						cmd_.axes[1] += 0.5;
					}
					break;
				case KEYCODE_s:
					if(cmd_.axes[1] > -1.0)
					{
						cmd_.axes[1] -= 0.5;
					}
					break;
				case KEYCODE_q:
					if(cmd_.axes[2] < 1.0)
					{
						cmd_.axes[2] += 0.5;
					}
					break;
				case KEYCODE_z:
					if(cmd_.axes[2] > 0.0)
					{
						cmd_.axes[2] -= 0.5;
					}
					break;
				case KEYCODE_o:
					if(cmd_.axes[3] < 1.0)
					{
						cmd_.axes[3] += 0.5;
					}
					break;
				case KEYCODE_COMMA:
					if(cmd_.axes[3] > 0.0)
					{
						cmd_.axes[3] -= 0.5;
					}
					break;
				case KEYCODE_j:
					if(cmd_.axes[4] > -1.0)
					{
						cmd_.axes[4] -= 0.5;
					}
					break;
				case KEYCODE_l:
					if(cmd_.axes[4] < 1.0)
					{
						cmd_.axes[4] += 0.5;
					}
					break;
				case KEYCODE_i:
					if(cmd_.axes[5] < 1.0)
					{
						cmd_.axes[5] += 0.5;
					}
					break;
				case KEYCODE_k:
					if(cmd_.axes[5] > -1.0)
					{
						cmd_.axes[5] -= 0.5;
					}
					break;

				case KEYCODE_ONE:
					cmd_.buttons[0] = true;
					break;
				case KEYCODE_TWO:
					cmd_.buttons[1] = true;
					break;
				case KEYCODE_THREE:
					cmd_.buttons[2] = true;
					break;
				case KEYCODE_FOUR:
					cmd_.buttons[3] = true;
					break;
				case KEYCODE_e:
					cmd_.buttons[4] = true;
					break;
				case KEYCODE_u:
					cmd_.buttons[5] = true;
					break;
				case KEYCODE_FIVE:
					cmd_.buttons[6] = true;
					break;
				case KEYCODE_SIX:
					cmd_.buttons[7] = true;
					break;
				case KEYCODE_x:
					cmd_.buttons[8] = true;
					break;
				case KEYCODE_m:
					cmd_.buttons[9] = true;
					break;
				case KEYCODE_SEVEN:
					cmd_.buttons[10] = true;
					break;

				case KEYCODE_r:
					ROS_INFO_STREAM("Resetting buttons to false!");
					for(size_t i = 0; i <= cmd_.buttons.size(); i++)
					{
						cmd_.buttons[i] = false;
					}
					break;

				case KEYCODE_LEFT_BRACKET:
					processing_bracket = true;
					dirty = true;
					break;
				case  KEYCODE_ESCAPE:
					//std::cout << std::endl;
					//std::cout << "Exiting " << std::endl;
					//quit(0);
					break;
				case KEYCODE_CARROT:
					ROS_WARN("It's a carrot");
					dirty = true;
					break;
				default:
					dirty = true;
					break;
			}
		}
		else // Processing bracket
		{
			ROS_INFO_STREAM("Running Dpad");
			switch (c)
			{
				case KEYCODE_B:
					if(cmd_.axes[7] > -1.0)
					{
						cmd_.axes[7] -= 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_A:
					if(cmd_.axes[7] < 1.0)
					{
						cmd_.axes[7] += 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_D:
					if(cmd_.axes[6] < 1.0)
					{
						cmd_.axes[6] += 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_C:
					if(cmd_.axes[6] > -1.0)
					{
						cmd_.axes[6] -= 1.0;
					}
					processing_bracket = false;
					dirty = false;
					break;
				case KEYCODE_LEFT_BRACKET:
					processing_bracket = true;
					break;
				default:
					dirty = true;
					break;
			}
		}
	}
	// Restore sanity to keyboard input
	tcsetattr(kfd, TCSANOW, &cooked);
}


FRCRobotSimInterface::FRCRobotSimInterface(ros::NodeHandle &nh,
		urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
    , teleop_joy_(nh)
{
}
FRCRobotSimInterface::~FRCRobotSimInterface()
{
    sim_joy_thread_.join();
}

void FRCRobotSimInterface::match_data_callback(const frc_msgs::MatchSpecificData &match_data) {
	std::lock_guard<std::mutex> l(match_data_mutex_);
	match_data_.setMatchTimeRemaining(match_data.matchTimeRemaining);
	match_data_.setGameSpecificData(match_data.gameSpecificData);
	match_data_.setEventName(match_data.eventName);
	match_data_.setAllianceColor(match_data.allianceColor);
	match_data_.setMatchType(match_data.matchType);
	match_data_.setDriverStationLocation(match_data.driverStationLocation);
	match_data_.setMatchNumber(match_data.matchNumber);
	match_data_.setReplayNumber(match_data.replayNumber);
	match_data_.setEnabled(match_data.Enabled);
	match_data_.setDisabled(match_data.Disabled);
	match_data_.setAutonomous(match_data.Autonomous);
	match_data_.setDSAttached(match_data.DSAttached);
	match_data_.setFMSAttached(match_data.FMSAttached);
	match_data_.setOperatorControl(match_data.OperatorControl);
	match_data_.setTest(match_data.Test);
	match_data_.setBatteryVoltage(match_data.BatteryVoltage);
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

void FRCRobotSimInterface::init(void)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	ROS_WARN("Passes");
	FRCRobotInterface::init();
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

	for (size_t i = 0; i < can_ctre_mc_names_.size(); i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << can_ctre_mc_names_[i] <<
							  (can_ctre_mc_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (can_ctre_mc_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as " << (can_ctre_mc_is_talon_[i] ? "TalonSRX" : "VictorSPX")
							  << " CAN id " << can_ctre_mc_can_ids_[i]);

		ROS_WARN_STREAM("fails here? 56789: " << i);
		// Loop through the list of joint names

	}
		ROS_WARN_STREAM("fails here? ~");
	// TODO : assert nidec_brushles_names_.size() == nidec_brushles_xxx_channels_.size()
	for (size_t i = 0; i < nidec_brushless_names_.size(); i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << nidec_brushless_names_[i] <<
							  (nidec_brushless_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (nidec_brushless_local_hardwares_[i] ? "local" : "remote") << " hardware " <<
							  " as PWM channel " << nidec_brushless_pwm_channels_[i] <<
							  " / DIO channel " << nidec_brushless_dio_channels_[i] <<
							  " invert " << nidec_brushless_inverts_[i]);

	for (size_t i = 0; i < num_digital_inputs_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << digital_input_names_[i] <<
							  " local = " << digital_input_locals_[i] <<
							  " as Digital Input " << digital_input_dio_channels_[i] <<
							  " invert " << digital_input_inverts_[i]);

	for (size_t i = 0; i < num_digital_outputs_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << digital_output_names_[i] <<
							  (digital_output_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (digital_output_local_hardwares_[i] ? "local" : "remote") << " hardware " <<
							  " as Digital Output " << digital_output_dio_channels_[i] <<
							  " invert " << digital_output_inverts_[i]);

	for (size_t i = 0; i < num_pwms_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << pwm_names_[i] <<
							  (pwm_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (pwm_local_hardwares_[i] ? "local" : "remote") << " hardware " <<
							  " as PWM " << pwm_pwm_channels_[i] <<
							  " invert " << pwm_inverts_[i]);

	ROS_WARN("fails here?5");
	for (size_t i = 0; i < num_solenoids_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << solenoid_names_[i] <<
							  (solenoid_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware " <<
							  " as Solenoid " << solenoid_ids_[i]);

	for (size_t i = 0; i < num_double_solenoids_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << double_solenoid_names_[i] <<
							  (double_solenoid_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (double_solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware " <<
							  " as Double Solenoid  forward " << double_solenoid_forward_ids_[i] <<
							  " reverse " << double_solenoid_reverse_ids_[i]);

	for(size_t i = 0; i < num_navX_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << navX_names_[i] <<
							  " local = " << navX_locals_[i] <<
							  " as navX id" << navX_ids_[i]);

	for (size_t i = 0; i < num_analog_inputs_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << analog_input_names_[i] <<
							  " local = " << analog_input_locals_[i] <<
							  " as Analog Input " << analog_input_analog_channels_[i]);

	for (size_t i = 0; i < num_compressors_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << compressor_names_[i] <<
							  (compressor_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (compressor_local_hardwares_[i] ? "local" : "remote") << " hardware " <<
							  " as Compressor with pcm " << compressor_pcm_ids_[i]);

	for (size_t i = 0; i < num_rumbles_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << rumble_names_[i] <<
							  (rumble_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (rumble_local_hardwares_[i] ? "local" : "remote") << " hardware " <<
							  " as Rumble with port" << rumble_ports_[i]);

	for (size_t i = 0; i < num_pdps_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << pdp_names_[i] <<
							  " local = " << pdp_locals_[i] <<
							  " as PDP");

	for(size_t i = 0; i < num_dummy_joints_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading dummy joint " << i << "=" << dummy_joint_names_[i]);

	// TODO : make this depend on joystick joints being defined
	if (run_hal_robot_)
		sim_joy_thread_ = std::thread(std::bind(&TeleopJointsKeyboard::keyboardLoop, &teleop_joy_));

	limit_switch_srv_ = nh_.advertiseService("set_limit_switch",&FRCRobotSimInterface::setlimit,this);
    match_data_sub_ = nh_.subscribe("/frcrobot_rio/match_data_in", 1, &FRCRobotSimInterface::match_data_callback, this);

	linebreak_sensor_srv_ = nh_.advertiseService("linebreak_service_set",&FRCRobotSimInterface::evaluateDigitalInput, this);
	ROS_INFO_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready.");
}

void FRCRobotSimInterface::read(ros::Duration &/*elapsed_time*/)
{
	for (std::size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		// Do nothing
    }
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		// Set using evaluateDigitalInput callback
		// TODO - make make threadsafe using a command buffer, if needed
	}

    // Simulated state is updated in write, so just
	// display it here for debugging
	// printState();

	// This is only used to test the stuff in hw_interface?
	if (!robot_code_ready_)
	{
		// Code is ready when all robot_ready_signals are set to non-zero values
		if (std::all_of(robot_ready_signals_.cbegin(),
						robot_ready_signals_.cend(),
						[](double d) { return d != 0.0;} ))
		{
			ROS_WARN("ROBOT CODE READY!");
			robot_code_ready_ = true;
		}
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
				digital_input_state_[i] = req.value ? 1 : 0;
				ROS_INFO_STREAM(digital_input_names_[i] << " set to " << (int)req.value);
				break;
			}
		}
	}
	else if (req.j < digital_input_names_.size())
	{
		digital_input_state_[req.j] = req.value ? 1 : 0;
		ROS_INFO_STREAM(digital_input_names_[req.j] << " set to " << (int)req.value);
	}
	else
	{
		ROS_INFO_STREAM(digital_input_names_[req.j] << " not set to " << (int)req.value << " index out of bounds");
	}
	return true;
}

void FRCRobotSimInterface::write(ros::Duration &elapsed_time)
{
#if 0
	ROS_INFO_STREAM_THROTTLE(1,
			std::endl << std::string(__FILE__) << ":" << __LINE__ <<
			std::endl << "Command" << std::endl << printCommandHelper());
#endif
	// Was the robot enabled last time write was run?
	static bool last_robot_enabled = false;

	// Is match data reporting the robot enabled now?
	bool robot_enabled = false;
	{
		std::lock_guard<std::mutex> l(match_data_mutex_);
		robot_enabled = match_data_.isEnabled();
	}

	for (std::size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
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
			ROS_INFO("feedback");
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

		hardware_interface::VelocityMeasurementPeriod v_m_period;
		int v_m_window;

		if (tc.velocityMeasurementChanged(v_m_period, v_m_window))
		{
			ts.setVelocityMeasurementPeriod(v_m_period);
			ts.setVelocityMeasurementWindow(v_m_window);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" velocity measurement period / window");
		}

		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" selected sensor position");
			ts.setPosition(sensor_position);
		}

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

			ts.setPosition(ts.getPosition() + speed * elapsed_time.toSec());
			ts.setSpeed(speed);
		}
		else if (simulate_mode == hardware_interface::TalonMode_MotionMagic)
		{
			double setpoint;

			if (tc.commandChanged(setpoint))
				ts.setSetpoint(setpoint);

			const double position = ts.getPosition();
			double velocity = ts.getSpeed();
			const double dt = elapsed_time.toSec();

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

			ts.setPosition(ts.getPosition() + percent*2*M_PI * elapsed_time.toSec());
			ts.setSpeed(percent*2*M_PI);
		}

		if (tc.clearStickyFaultsChanged())
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<" sticky_faults");

		tc.unlock();
	}
	last_robot_enabled = robot_enabled;

	for (std::size_t joint_id = 0; joint_id < num_nidec_brushlesses_; ++joint_id)
	{
		// Assume instant acceleration for now
		const double vel = brushless_command_[joint_id];
		brushless_vel_[joint_id] = vel;
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		bool converted_command = (digital_output_command_[i] > 0) ^ (digital_output_inverts_[i] && digital_output_local_updates_[i]);
		if (converted_command != digital_output_state_[i])
		{
			digital_output_state_[i] = converted_command;
			ROS_INFO_STREAM("DIO " << digital_output_names_[i] <<
					" at channel " <<  digital_output_dio_channels_[i] <<
					" set to " << converted_command);
		}
	}
	for (size_t i = 0; i < num_pwms_; i++)
	{
		const int setpoint = pwm_command_[i] * ((pwm_inverts_[i] & pwm_local_updates_[i]) ? -1 : 1);
		if (pwm_state_[i] != setpoint)
		{
			pwm_state_[i] = setpoint;
			ROS_INFO_STREAM("PWM " << pwm_names_[i] <<
					" at channel " <<  pwm_pwm_channels_[i] <<
					" set to " << pwm_state_[i]);
		}
	}

	for (size_t i = 0; i< num_solenoids_; i++)
	{
		const bool setpoint = solenoid_command_[i] > 0;
		if (solenoid_state_[i] != setpoint)
		{
			solenoid_state_[i] = setpoint;
			ROS_INFO_STREAM("Solenoid " << solenoid_names_[i] <<
							" at id " << solenoid_ids_[i] <<
							" / pcm " << solenoid_pcms_[i] <<
							" = " << setpoint);
		}
	}

	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		// TODO - maybe check for < 0, 0, >0 and map to forward/reverse?
		const double command = double_solenoid_command_[i];
		double setpoint;
		if (command >= 1.)
			setpoint = 1.;
		else if (command <= -1.)
			setpoint = -1.;
		else
			setpoint = 0.;

		if (double_solenoid_state_[i] != setpoint)
		{
			double_solenoid_state_[i] = setpoint;
			ROS_INFO_STREAM("Double solenoid " << double_solenoid_names_[i] <<
					" at forward id " << double_solenoid_forward_ids_[i] <<
					"/ reverse id " << double_solenoid_reverse_ids_[i] <<
					" / pcm " << double_solenoid_pcms_[i] <<
					" = " << setpoint);
		}
	}

	for (size_t i = 0; i < num_rumbles_; i++)
	{
		if (rumble_state_[i] != rumble_command_[i])
		{
			const unsigned int rumbles = *((unsigned int*)(&rumble_command_[i]));
			const unsigned int left_rumble  = (rumbles >> 16) & 0xFFFF;
			const unsigned int right_rumble = (rumbles      ) & 0xFFFF;
			rumble_state_[i] = rumble_command_[i];

			ROS_INFO_STREAM("Joystick at port " << rumble_ports_[i] <<
				" left rumble = " << std::dec << left_rumble << "(" << std::hex << left_rumble <<
				") right rumble = " << std::dec << right_rumble << "(" << std::hex << right_rumble <<  ")" << std::dec);
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
			dummy_joint_velocity_[i] = (dummy_joint_command_[i] - dummy_joint_position_[i]) / elapsed_time.toSec();
			dummy_joint_position_[i] = dummy_joint_command_[i];
		}
		//else if (dummy_joint_names_[i].substr(2, std::string::npos) == "_drive")
		{
			// position mode
			//dummy_joint_position_[i] += dummy_joint_command_[i] * elapsed_time.toSec();
			//dummy_joint_velocity_[i] = dummy_joint_command_[i];
		}
	}
	//ROS_INFO_STREAM_THROTTLE(1, s.str());
}

}  // namespace



