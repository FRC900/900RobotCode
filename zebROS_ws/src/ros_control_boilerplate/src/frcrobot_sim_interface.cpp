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
#include <memory> // for make_unique()
#include <ros/ros.h>

#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"
#include "frc/DriverStation.h"
#include "frc/simulation/BatterySim.h"
#include "frc/simulation/RoboRioSim.h"
#include "hal/HALBase.h"
#include "hal/simulation/DIOData.h"
#include "hal/simulation/DriverStationData.h"
#include "../sim/HALInitializer.h"

#include <ros_control_boilerplate/frcrobot_sim_interface.h>
#include <ros_control_boilerplate/set_limit_switch.h>

namespace ros_control_boilerplate
{

FRCRobotSimInterface::FRCRobotSimInterface(ros::NodeHandle &nh,
		urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
}
FRCRobotSimInterface::~FRCRobotSimInterface() = default;

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
	if (static_cast<size_t>(joystick_num) >= joystick_sim_write_mutex_.size())
	{
		ROS_WARN_STREAM_THROTTLE(2, "Sim joystick input for index " << joystick_num
				<< " is bigger than configured joystick count ( " << joystick_sim_write_mutex_.size() << ")");
		return;
	}

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

	HAL_JoystickPOVs hal_povs;
	hal_povs.count = 1;
	std::memset(hal_povs.povs, -1, sizeof(hal_povs.povs));
	if (msg->axes.size() >= 8)
	{
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
	}
	//TODO check default pov?
	//TODO do you need to set JoystickDescriptor?

	std::lock_guard<std::mutex> l(*(joystick_sim_write_mutex_[joystick_num]));
	HALSIM_SetJoystickPOVs(joystick_num, &hal_povs);
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
	ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " line: " << __LINE__);

	// Work around CTRE sim bug?
	skip_bus_voltage_temperature_ = true;
	if (!FRCRobotInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " failed");
		return false;
	}
	hal::init::InitializeDriverStationData();

	for (size_t i = 0; i < num_canifiers_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << canifier_names_[i] <<
							  (canifier_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (canifier_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << canifier_can_ids_[i]);
	}

	// TODO - merge me into frc robot interface, add a sim setting, etc.
	for (size_t i = 0; i < num_as726xs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
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
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << talon_orchestra_names_[i]);
	}

	for (size_t i = 0; i < num_spark_maxs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_sim_interface",
							  "Loading joint " << i << "=" << spark_max_names_[i] <<
							  (spark_max_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (spark_max_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as CAN id " << spark_max_can_ids_[i]);
	}

#if 0
	// Hard-code a simulation for the shooter flywheel
	shooter_sim_joint_index_ = std::numeric_limits<size_t>::max();
	for (size_t i = 0; i < num_can_ctre_mcs_; ++i)
	{
		if (can_ctre_mc_names_[i] == "shooter_joint")
		{
			shooter_sim_joint_index_ = i;
			break;
		}
	}
	// TODO - maybe also check for local hardware?
	if (shooter_sim_joint_index_ != std::numeric_limits<size_t>::max())
	{
		units::kilogram_square_meter_t moi{0.000471735121};
		//units::pound_square_inch_t moi_english{1.612};
		// This is in rad, so small values to line up with a small
		// amount of uncertainty in the encoder reading
		const std::array<double, 1> shooter_sim_std_dev = {0.0025};
		shooter_sim_ = std::make_unique<frc::sim::FlywheelSim>(frc::DCMotor::Falcon500(1), 24./34., moi, shooter_sim_std_dev);
	}
	else
	{
		// This will be expected for the Rio - we only want to run sim code in one
		// place, and that should be the Jetson
		ROS_WARN("Couldn't find joint 'shooter_joint' in config, not running shooter sim");
		shooter_sim_ = nullptr;
	}
#endif

	// Do these last to prevent callbacks from triggering before data
	// used by them is initialized
	ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " line: " << __LINE__);
	limit_switch_srv_ = root_nh.advertiseService("set_limit_switch",&FRCRobotSimInterface::setlimit, this);
	linebreak_sensor_srv_ = root_nh.advertiseService("linebreak_service_set",&FRCRobotSimInterface::evaluateDigitalInput, this);
    match_data_sub_ = root_nh.subscribe("/frcrobot_rio/match_data_in", 1, &FRCRobotSimInterface::match_data_callback, this);

    //TODO fix joystick topic names?
	for (size_t i = 0; i < num_joysticks_; i++)
	{
		std::stringstream s;
		s << "js" << i << "_in";
		joystick_subs_.push_back(root_nh.subscribe<sensor_msgs::Joy>(s.str(), 1, boost::bind(&FRCRobotSimInterface::joystickCallback, this, _1, i)));
	}


	ROS_INFO_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready.");
	return true;
}

void FRCRobotSimInterface::read(const ros::Time& time, const ros::Duration& period)
{
	// Run WPIlib physics sim for each mechanism.
	// Right now those mechanisms are hard-coded, but in the future make
	// them configurable via config file?
	// This could happen at the end of write() or beginning of read(),
	// shouldn't matter which just so long as the sim mechanisms are
	// TODO : needed for standalone robots, but not
	// updated once per control loop using the appropriate timestep

	read_tracer_.start_unique("FeedEnable");
	if (num_can_ctre_mcs_)
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(2 * 1000./ctre_mc_read_hz_);
	}
	read_tracer_.start_unique("HAL_SimPeriodicBefore");
	HAL_SimPeriodicBefore();
	read_tracer_.start_unique("ShooterSim");
#if 0
	// This needs work.  It kinda takes commands but ends up at the wrong
	// steady-state velocity.  Could be from the gearing, the unit conversion,
	// or something else (or a combo of all of them)
	// It is left here as an example of things we could do :
	//   - read motor output voltate
	//   - apply that voltage as input to a simulation
	//   - get the simulate state and feed that back to the motor controller sim
	if (shooter_sim_)
	{
		const auto talon_motor_voltage = talon_state_[shooter_sim_joint_index_].getOutputVoltage();
		//const auto talon_motor_voltage = ctre_mcs_[shooter_sim_joint_index_]->GetMotorOutputVoltage();
		// Get current state of motor controller commanded output voltage
		units::volt_t shooter_motor_voltage{talon_motor_voltage};
		// Update simulation with that voltage
		shooter_sim_->SetInputVoltage(shooter_motor_voltage);
		// Simulate one timestep using that voltage
		shooter_sim_->Update(units::second_t{period.toSec()});

		// Read state of simulated mechanism, update sim motor controller
		// with the output state
		// Convert from mechanism speed to motor speed, since that's where the encoder is
		const auto shooter_velocity{shooter_sim_->GetAngularVelocity().to<double>() * (24./34.)};
		// rad / sec * (4096. native units / 2PI rad) /(10. 100ms per sec) = native units / 100msec
		// TODO : need conversion function
		const auto shooter_velocity_native_units = shooter_velocity * (4096. / (2 * M_PI)) / 10.;

		auto talon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(ctre_mcs_[shooter_sim_joint_index_]);

		if (talon)
		{
			ROS_INFO_STREAM("Motor voltage = " << talon_motor_voltage);
			ROS_INFO_STREAM("Shooter velocity (rad/sec) = " << shooter_velocity);
			ROS_INFO_STREAM("Shooter velocity (native units / 100msec) = " << shooter_velocity_native_units);
			ROS_INFO_STREAM("Period = " << period.toSec());
			safeTalonCall(talon->GetSimCollection().AddQuadraturePosition(shooter_velocity_native_units * period.toSec()), "AddQuadraturePosition");
			safeTalonCall(talon->GetSimCollection().SetQuadratureVelocity(shooter_velocity_native_units), "SetQuadratureVelocity");

			const auto shooter_current{shooter_sim_->GetCurrentDraw().to<double>()};
			ROS_INFO_STREAM("shooter_current = " << shooter_current);
			safeTalonCall(talon->GetSimCollection().SetStatorCurrent(shooter_current), "SetStatorCurrent");
			// TODO : should collect currents for each sim mechanism and pass
			// them all in to BatterySim::Calculate at once
			const auto vin_voltage = frc::sim::BatterySim::Calculate({shooter_sim_->GetCurrentDraw()});
			// TODO : victor-> ?
			talon->GetSimCollection().SetBusVoltage(vin_voltage.to<double>());
			frc::sim::RoboRioSim::SetVInVoltage(vin_voltage);
		}
	}

	if (shooter_sim_ && false)
	{
		auto wpitalonsrx = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(ctre_mcs_[shooter_sim_joint_index_]);
		wpitalonsrx->GetSimCollection().AddQuadraturePosition(18000*.01);
		wpitalonsrx->GetSimCollection().SetQuadratureVelocity(18000);
		auto victor = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorController>(ctre_mcs_[shooter_sim_joint_index_]);
		auto talon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorControllerEnhanced>(ctre_mcs_[shooter_sim_joint_index_]);
		auto talonsrx = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonSRX>(ctre_mcs_[shooter_sim_joint_index_]);
		ROS_INFO_STREAM("IMotorController->getSelectedSensorPosition() = " << victor->GetSelectedSensorPosition(0));
		safeTalonCall(victor->GetLastError(), "GetSelectedSensorPosition");
		ROS_INFO_STREAM("IMotorControllerEnhance->getSelectedSensorPosition() = " << talon->GetSelectedSensorPosition(0));
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorPosition");
		ROS_INFO_STREAM("TalonSRX->getSelectedSensorPosition() = " << talonsrx->GetSelectedSensorPosition(0));
		safeTalonCall(talonsrx->GetLastError(), "GetSelectedSensorPosition");
		ROS_INFO_STREAM("WPITalonSRX->getSelectedSensorPosition() = " << wpitalonsrx->GetSelectedSensorPosition(0));
		safeTalonCall(wpitalonsrx->GetLastError(), "GetSelectedSensorPosition");
		ROS_INFO_STREAM("IMotorController->getSelectedSensorVelocity() = " << victor->GetSelectedSensorVelocity(0));
		safeTalonCall(victor->GetLastError(), "GetSelectedSensorVelocity");
		ROS_INFO_STREAM("IMotorControllerEnhance->getSelectedSensorVelocity() = " << talon->GetSelectedSensorVelocity(0));
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorPosition");
		ROS_INFO_STREAM("TalonSRX->getSelectedSensorVelocity() = " << talonsrx->GetSelectedSensorVelocity(0));
		safeTalonCall(talonsrx->GetLastError(), "GetSelectedSensorPosition");
		ROS_INFO_STREAM("WPITalonSRX->getSelectedSensorVelocity() = " << wpitalonsrx->GetSelectedSensorVelocity(0));
		safeTalonCall(wpitalonsrx->GetLastError(), "GetSelectedSensorPosition");
	}
#endif

	read_tracer_.start_unique("HAL_SimPeriodicAfter");
	HAL_SimPeriodicAfter();
	read_tracer_.stop();

	FRCRobotInterface::read(time, period);
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
	last_robot_enabled = robot_enabled;

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



