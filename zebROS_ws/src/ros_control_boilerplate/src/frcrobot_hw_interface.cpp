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

#include <iostream>
#include <thread>

#include <ros_control_boilerplate/frcrobot_hw_interface.h>
#include <ros_control_boilerplate/JoystickState.h>
#include "HAL/DriverStation.h"
#include "HAL/HAL.h"
#include "HAL/PDP.h"
#include "HAL/Ports.h"
#include "Joystick.h"
#include "ros_control_boilerplate/MatchSpecificData.h"
#include "ros_control_boilerplate/AutoMode.h"
#include "math.h"
#include <cmath>
#include <networktables/NetworkTable.h>
#include <SmartDashboard/SmartDashboard.h>
#include <SmartDashboard/SendableBuilder.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ctre/phoenix/MotorControl/SensorCollection.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ros_control_boilerplate/PDPData.h"
#include <PowerDistributionPanel.h>
#include <stdint.h>

namespace frcrobot_control
{
const int pidIdx = 0; //0 for primary closed-loop, 1 for cascaded closed-loop
const int timeoutMs = 0; //If nonzero, function will wait for config success and report an error if it times out. If zero, no blocking or checking is performed

FRCRobotHWInterface::FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model),
	  run_hal_thread_(true),
	  run_motion_profile_thread_(true)
{
}

FRCRobotHWInterface::~FRCRobotHWInterface()
{
	run_hal_thread_            = false;
	run_motion_profile_thread_ = false;
	hal_thread_.join();
	motion_profile_thread_.join();
}

void FRCRobotHWInterface::hal_keepalive_thread(void)
{
	run_hal_thread_ = true;

	// This will be written by the last controller to be
	// spawned - waiting here prevents the robot from
	// report robot code ready to the field until
	// all controllers are started
	ros::Rate rate(50);
	while (robot_code_ready_ == 0.0)
		rate.sleep();

	robot_.StartCompetition();

	Joystick joystick(0);
	realtime_tools::RealtimePublisher<ros_control_boilerplate::JoystickState> realtime_pub_joystick(nh_, "joystick_states", 4);
	realtime_tools::RealtimePublisher<ros_control_boilerplate::MatchSpecificData> realtime_pub_match_data(nh_, "match_data", 4);

	// Setup writing to a network table that already exists on the dashboard
	//std::shared_ptr<nt::NetworkTable> pubTable = NetworkTable::GetTable("String 9");
	std::shared_ptr<nt::NetworkTable> subTable = NetworkTable::GetTable("Custom");
	std::shared_ptr<nt::NetworkTable> driveTable = NetworkTable::GetTable("SmartDashboard");  //Access Smart Dashboard Variables
	realtime_tools::RealtimePublisher<ros_control_boilerplate::AutoMode> realtime_pub_nt(nh_, "Autonomous_Mode", 4);
    realtime_pub_nt.msg_.mode.resize(4);
    ros::Time time_now_t;
	while (run_hal_thread_ && ros::ok())
	{
		robot_.OneIteration();

        time_now_t = ros::Time::now();
		//ROS_INFO("%f", ros::Time::now().toSec());
		// Network tables work!
		//pubTable->PutString("String 9", "WORK");
		//subTable->PutString("Auto Selector", "Select Auto");
		if (realtime_pub_nt.trylock()) 
		{
			if (driveTable)
			{
				// SmartDashboard works!
				//frc::SmartDashboard::PutNumber("SmartDashboard Test", 999);

				//realtime_pub_nt.msg_.data = driveTable->GetString("Auto Selector", "0");
				realtime_pub_nt.msg_.mode[0] = (int)driveTable->GetNumber("auto_mode_0", 0);
				realtime_pub_nt.msg_.mode[1] = (int)driveTable->GetNumber("auto_mode_1", 0);
				realtime_pub_nt.msg_.mode[2] = (int)driveTable->GetNumber("auto_mode_2", 0);
				realtime_pub_nt.msg_.mode[3] = (int)driveTable->GetNumber("auto_mode_3", 0);
				realtime_pub_nt.msg_.position = (int)driveTable->GetNumber("robot_start_position", 0);
			}

			realtime_pub_nt.msg_.header.stamp = time_now_t;
			realtime_pub_nt.unlockAndPublish();
		}

		if (realtime_pub_joystick.trylock())
		{
			realtime_pub_joystick.msg_.header.stamp = time_now_t;

			realtime_pub_joystick.msg_.leftStickX = joystick.GetRawAxis(0);
			realtime_pub_joystick.msg_.leftStickY = joystick.GetRawAxis(1);
			realtime_pub_joystick.msg_.rightStickX = joystick.GetRawAxis(4);
			realtime_pub_joystick.msg_.rightStickY = joystick.GetRawAxis(5);

			realtime_pub_joystick.msg_.leftTrigger = joystick.GetRawAxis(2);
			realtime_pub_joystick.msg_.rightTrigger = joystick.GetRawAxis(3);

			realtime_pub_joystick.msg_.buttonAButton = joystick.GetRawButton(1);
			realtime_pub_joystick.msg_.buttonAPress = joystick.GetRawButtonPressed(1);
			realtime_pub_joystick.msg_.buttonARelease = joystick.GetRawButtonReleased(1);

			realtime_pub_joystick.msg_.buttonBButton = joystick.GetRawButton(2);
			realtime_pub_joystick.msg_.buttonBPress = joystick.GetRawButtonPressed(2);
			realtime_pub_joystick.msg_.buttonBRelease = joystick.GetRawButtonReleased(2);

			realtime_pub_joystick.msg_.buttonXButton = joystick.GetRawButton(3);
			realtime_pub_joystick.msg_.buttonXPress = joystick.GetRawButtonPressed(3);
			realtime_pub_joystick.msg_.buttonXRelease = joystick.GetRawButtonReleased(3);

			realtime_pub_joystick.msg_.buttonYButton = joystick.GetRawButton(4);
			realtime_pub_joystick.msg_.buttonYPress = joystick.GetRawButtonPressed(4);
			realtime_pub_joystick.msg_.buttonYRelease = joystick.GetRawButtonReleased(4);

			realtime_pub_joystick.msg_.bumperLeftButton = joystick.GetRawButton(5);
			realtime_pub_joystick.msg_.bumperLeftPress = joystick.GetRawButtonPressed(5);
			realtime_pub_joystick.msg_.bumperLeftRelease = joystick.GetRawButtonReleased(5);

			realtime_pub_joystick.msg_.bumperRightButton = joystick.GetRawButton(6);
			realtime_pub_joystick.msg_.bumperRightPress = joystick.GetRawButtonPressed(6);
			realtime_pub_joystick.msg_.bumperRightRelease = joystick.GetRawButtonReleased(6);

			realtime_pub_joystick.msg_.buttonBackButton = joystick.GetRawButton(7);
			realtime_pub_joystick.msg_.buttonBackPress = joystick.GetRawButtonPressed(7);
			realtime_pub_joystick.msg_.buttonBackRelease = joystick.GetRawButtonReleased(7);

			realtime_pub_joystick.msg_.buttonStartButton = joystick.GetRawButton(8);
			realtime_pub_joystick.msg_.buttonStartPress = joystick.GetRawButtonPressed(8);
			realtime_pub_joystick.msg_.buttonStartRelease = joystick.GetRawButtonReleased(8);

			realtime_pub_joystick.msg_.stickLeftButton = joystick.GetRawButton(9);
			realtime_pub_joystick.msg_.stickLeftPress = joystick.GetRawButtonPressed(9);
			realtime_pub_joystick.msg_.stickLeftRelease = joystick.GetRawButtonReleased(9);

			realtime_pub_joystick.msg_.stickRightButton = joystick.GetRawButton(10);
			realtime_pub_joystick.msg_.stickRightPress = joystick.GetRawButtonPressed(10);
			realtime_pub_joystick.msg_.stickRightRelease = joystick.GetRawButtonReleased(10);

			realtime_pub_joystick.unlockAndPublish();
		}

		if(realtime_pub_match_data.trylock())
		{
            
            //ROS_INFO("AA:%f", ros::Time::now().toSec());

			realtime_pub_match_data.msg_.matchTimeRemaining = DriverStation::GetInstance().GetMatchTime();

			realtime_pub_match_data.msg_.allianceData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
			realtime_pub_match_data.msg_.allianceColor = DriverStation::GetInstance().GetAlliance(); //returns int that corresponds to a DriverStation Alliance enum
			realtime_pub_match_data.msg_.driverStationLocation = DriverStation::GetInstance().GetLocation();
			realtime_pub_match_data.msg_.matchNumber = DriverStation::GetInstance().GetMatchNumber();
			realtime_pub_match_data.msg_.matchType = DriverStation::GetInstance().GetMatchType(); //returns int that corresponds to a DriverStation matchType enum

			realtime_pub_match_data.msg_.isEnabled = DriverStation::GetInstance().IsEnabled();
			realtime_pub_match_data.msg_.isDisabled = DriverStation::GetInstance().IsDisabled();
			realtime_pub_match_data.msg_.isAutonomous = DriverStation::GetInstance().IsAutonomous();

			realtime_pub_match_data.msg_.header.stamp = ros::Time::now();
			realtime_pub_match_data.unlockAndPublish();
		}
	}
}

void FRCRobotHWInterface::process_motion_profile_buffer_thread(ros::Rate rate)
{
	while (run_motion_profile_thread_ && ros::ok())
	{
		for (size_t i = 0; i < num_can_talon_srxs_; i++)
			can_talons_[i]->ProcessMotionProfileBuffer();

		rate.sleep();
	}
}

void FRCRobotHWInterface::init(void)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	FRCRobotInterface::init();
	//ROS_INFO_STREAM("init is running");

	// Make sure to initialize WPIlib code before creating
	// a CAN Talon object to avoid NIFPGA: Resource not initialized
	// errors? See https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
	hal_thread_ = std::thread(&FRCRobotHWInterface::hal_keepalive_thread, this);

	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << can_talon_srx_names_[i] <<
							  " as CAN id " << can_talon_srx_can_ids_[i]);
		can_talons_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(can_talon_srx_can_ids_[i]));
		can_talons_[i]->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 50); // Make sure motor is stopped, use a long timeout just in case
		safeTalonCall(can_talons_[i]->GetLastError(), "Initial Set(Disabled, 0)");
		safeTalonCall(can_talons_[i]->ClearStickyFaults(timeoutMs), "ClearStickyFaults()");
		// TODO : if the talon doesn't initialize - maybe known
		// by -1 from firmware version read - somehow tag
		// the entry in can_talons_[] as uninitialized.
		// This probably should be a fatal error
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "\tTalon SRX firmware version " << can_talons_[i]->GetFirmwareVersion());
		// Clear sticky faults
		// safeTalonCall(can_talons_[1]->ClearStickyFaults(timeoutMs), "Clear sticky faults.");
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << nidec_brushless_names_[i] <<
							  " as PWM channel " << nidec_brushless_pwm_channels_[i] <<
							  " / DIO channel " << nidec_brushless_dio_channels_[i] <<
							  " invert " << nidec_brushless_inverts_[i]);

		nidec_brushlesses_.push_back(std::make_shared<frc::NidecBrushless>(nidec_brushless_pwm_channels_[i], nidec_brushless_dio_channels_[i]));
		nidec_brushlesses_[i]->SetInverted(nidec_brushless_inverts_[i]);
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << digital_input_names_[i] <<
							  " as Digital Input " << digital_input_dio_channels_[i] <<
							  " invert " << digital_input_inverts_[i]);

		digital_inputs_.push_back(std::make_shared<frc::DigitalInput>(digital_input_dio_channels_[i]));
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << digital_output_names_[i] <<
							  " as Digital Output " << digital_output_dio_channels_[i] <<
							  " invert " << digital_output_inverts_[i]);

		digital_outputs_.push_back(std::make_shared<frc::DigitalOutput>(digital_output_dio_channels_[i]));
	}
	for (size_t i = 0; i < num_pwm_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << pwm_names_[i] <<
							  " as Digitial Output " << pwm_pwm_channels_[i] <<
							  " invert " << pwm_inverts_[i]);

		PWMs_.push_back(std::make_shared<frc::SafePWM>(pwm_pwm_channels_[i]));
		PWMs_[i]->SetSafetyEnabled(true);
	}
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << solenoid_names_[i] <<
							  " as Solenoid " << solenoid_ids_[i]
							  << " with pcm " << solenoid_pcms_[i]);

		solenoids_.push_back(std::make_shared<frc::Solenoid>(solenoid_pcms_[i], solenoid_ids_[i]));
	}
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << double_solenoid_names_[i] <<
							  " as Double Solenoid  forward " << double_solenoid_forward_ids_[i] <<
							  " reverse " << double_solenoid_reverse_ids_[i]
							  << " with pcm " << double_solenoid_pcms_[i]);

		double_solenoids_.push_back(std::make_shared<frc::DoubleSolenoid>(double_solenoid_pcms_[i], double_solenoid_forward_ids_[i], double_solenoid_reverse_ids_[i]));
	}

	//RIGHT NOW THIS WILL ONLY WORK IF THERE IS ONLY ONE NAVX INSTANTIATED
	for(size_t i = 0; i < num_navX_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
				"Loading joint " << i << "=" << navX_names_[i] <<
				" as navX id " << navX_ids_[i]);
		//TODO: fix how we use ids

		navXs_.push_back(std::make_shared<AHRS>(SPI::Port::kMXP));

		// This is a guess so TODO : get better estimates
		imu_orientation_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		imu_angular_velocity_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
	   	imu_linear_acceleration_covariances_[i] ={0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
	}
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << analog_input_names_[i] <<
							  " as Analog Input " << analog_input_analog_channels_[i]);
		analog_inputs_.push_back(std::make_shared<frc::AnalogInput>(analog_input_analog_channels_[i]));
	}
	for (size_t i = 0; i < num_compressors_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << compressor_names_[i] <<
							  " as Compressor with pcm " << compressor_pcm_ids_[i]);

		compressors_.push_back(std::make_shared<frc::Compressor>(compressor_pcm_ids_[i]));
	}

	for(size_t i = 0; i < num_dummy_joints_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading dummy joint " << i << "=" << dummy_joint_names_[i]);

	//pdp_joint_.ClearStickyFaults();
	//pdp_joint_.ResetTotalEnergy();

	HAL_InitializePDP(0,0);

	//HAL_ObserveUserProgramStarting();

	motion_profile_thread_ = std::thread(&FRCRobotHWInterface::process_motion_profile_buffer_thread, this, ros::Rate(200));
	ROS_INFO_NAMED("frcrobot_hw_interface", "FRCRobotHWInterface Ready.");
}

void FRCRobotHWInterface::read(ros::Duration &/*elapsed_time*/)
{
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		auto &ts = talon_state_[joint_id];
		auto &talon = can_talons_[joint_id];

		if (!talon) // skip unintialized Talons
			continue;
		// read position and velocity from can_talons_[joint_id]
		// convert to whatever units make sense

		const hardware_interface::FeedbackDevice encoder_feedback = ts.getEncoderFeedback();
		const hardware_interface::TalonMode talon_mode = ts.getTalonMode();
		const int encoder_ticks_per_rotation = ts.getEncoderTicksPerRotation();
		const double conversion_factor = ts.getConversionFactor();

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position, joint_id) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity, joint_id)* conversion_factor;
		double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode, joint_id)* conversion_factor;

		const double position = talon->GetSelectedSensorPosition(pidIdx) * radians_scale;
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorPosition");
		ts.setPosition(position);

		const double speed = talon->GetSelectedSensorVelocity(pidIdx) * radians_per_second_scale;
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorVelocity");
		ts.setSpeed(speed);

		const double bus_voltage = talon->GetBusVoltage();
		safeTalonCall(talon->GetLastError(), "GetBusVoltage");
		ts.setBusVoltage(bus_voltage);

		const double motor_output_percent = talon->GetMotorOutputPercent();
		safeTalonCall(talon->GetLastError(), "GetMotorOutputPercent");
		ts.setMotorOutputPercent(motor_output_percent);

		const double output_voltage = talon->GetMotorOutputVoltage();
		safeTalonCall(talon->GetLastError(), "GetMotorOutputVoltage");
		ts.setOutputVoltage(output_voltage);

		const double output_current = talon->GetOutputCurrent();
		safeTalonCall(talon->GetLastError(), "GetOutputCurrent");
		ts.setOutputCurrent(output_current);

		const double temperature = talon->GetTemperature(); //returns in Celsius
		safeTalonCall(talon->GetLastError(), "GetTemperature");
		ts.setTemperature(temperature);

		//closed-loop
		if ((talon_mode == hardware_interface::TalonMode_Position) ||
		    (talon_mode == hardware_interface::TalonMode_Velocity) ||
		    (talon_mode == hardware_interface::TalonMode_Current ) ||
			(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic))
		{
			const double closed_loop_error = talon->GetClosedLoopError(pidIdx) * closed_loop_scale;
			safeTalonCall(talon->GetLastError(), "GetClosedLoopError");
			ts.setClosedLoopError(closed_loop_error);

			const double integral_accumulator = talon->GetIntegralAccumulator(pidIdx) * closed_loop_scale;
			safeTalonCall(talon->GetLastError(), "GetIntegralAccumulator");
			ts.setIntegralAccumulator(integral_accumulator);

			const double error_derivative = talon->GetErrorDerivative(pidIdx) * closed_loop_scale;
			safeTalonCall(talon->GetLastError(), "GetErrorDerivative");
			ts.setErrorDerivative(error_derivative);

			const double closed_loop_target = talon->GetClosedLoopTarget(pidIdx) * closed_loop_scale;
			safeTalonCall(talon->GetLastError(), "GetClosedLoopTarget");
			ts.setClosedLoopTarget(closed_loop_target);
		}

		if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic))
		{
			const double active_trajectory_position = talon->GetActiveTrajectoryPosition() * radians_scale;
			safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryPosition");
			ts.setActiveTrajectoryPosition(active_trajectory_position);
			const double active_trajectory_velocity = talon->GetActiveTrajectoryVelocity() * radians_per_second_scale;
			safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryVelocity");
			ts.setActiveTrajectoryVelocity(active_trajectory_velocity);
			const double active_trajectory_heading = talon->GetActiveTrajectoryHeading() * 2.*M_PI / 360.; //returns in degrees
			safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryHeading");
			ts.setActiveTrajectoryHeading(active_trajectory_heading);
			ts.setMotionProfileTopLevelBufferCount(talon->GetMotionProfileTopLevelBufferCount());
			safeTalonCall(talon->GetLastError(), "GetMotionProfileTopLevelBufferCount");
			ts.setMotionProfileTopLevelBufferFull(talon->IsMotionProfileTopLevelBufferFull());
			safeTalonCall(talon->GetLastError(), "IsMotionProfileTopLevelBufferFull");
			ctre::phoenix::motion::MotionProfileStatus talon_status;
			safeTalonCall(talon->GetMotionProfileStatus(talon_status), "GetMotionProfileStatus");

			hardware_interface::MotionProfileStatus internal_status;
			internal_status.topBufferRem = talon_status.topBufferRem;
			internal_status.topBufferCnt = talon_status.topBufferCnt;
			internal_status.btmBufferCnt = talon_status.btmBufferCnt;
			internal_status.hasUnderrun = talon_status.hasUnderrun;
			internal_status.isUnderrun = talon_status.isUnderrun;
			internal_status.activePointValid = talon_status.activePointValid;
			internal_status.isLast = talon_status.isLast;
			internal_status.profileSlotSelect0 = talon_status.profileSlotSelect0;
			internal_status.profileSlotSelect1 = talon_status.profileSlotSelect1;
			internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);
			internal_status.timeDurMs = talon_status.timeDurMs;

			ts.setMotionProfileStatus(internal_status);
		}

		ctre::phoenix::motorcontrol::Faults faults;
		safeTalonCall(talon->GetFaults(faults), "GetFaults");
		ts.setFaults(faults.ToBitfield());

		// Grab limit switch and softlimit here
		auto sensor_collection = talon->GetSensorCollection();
		ts.setForwardLimitSwitch(sensor_collection.IsFwdLimitSwitchClosed());
		ts.setReverseLimitSwitch(sensor_collection.IsRevLimitSwitchClosed());

		ts.setForwardSoftlimitHit(faults.ForwardSoftLimit);
		ts.setReverseSoftlimitHit(faults.ReverseSoftLimit);

		ctre::phoenix::motorcontrol::StickyFaults sticky_faults;
		safeTalonCall(talon->GetStickyFaults(sticky_faults), "GetStickyFaults");
		ts.setStickyFaults(sticky_faults.ToBitfield());
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		brushless_vel_[i] = nidec_brushlesses_[i]->Get();
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		digital_input_state_[i] = (digital_inputs_[i]->Get()^digital_input_inverts_[i]) ? 1 : 0;
		//State should really be a bool - but we're stuck using
		//ROS control code which thinks everything to and from
		//hardware are doubles
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		digital_output_state_[i] = (digital_outputs_[i]->Get()^digital_output_inverts_[i]) ? 1 : 0;
		//State should really be a bool
		//This isn't strictly neccesary, it just reads what the DIO is currently set to
	}
	for (size_t i = 0; i < num_pwm_; i++)
	{
		// Just reflect state of output in status
		//pwm_state_[i] = PWMs_[i]->GetSpeed();
	}
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		solenoid_state_[i] = solenoids_[i]->Get();
	}
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		double_solenoid_state_[i] = double_solenoids_[i]->Get();
	}
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		ROS_INFO_STREAM("base: " << analog_inputs_[i]->GetValue() << " a: " << analog_input_a_[i] << " b:"
		<< analog_input_b_[i]);
		analog_input_state_[i] = (analog_inputs_[i]->GetValue())*analog_input_a_[i] + analog_input_b_[i];
	}
 	//navX read here
	for (size_t i = 0; i < num_navX_; i++)
	{
		// TODO : double check we're reading
		// the correct data

		// navXs_[i]->GetFusedHeading();
		// navXs_[i]->GetPitch();
		// navXs_[i]->GetRoll();

		// TODO : Fill in imu_angular_velocity[i][]

		//navXs_[i]->IsCalibrating();
		//navXs_[i]->IsConnected();
		//navXs_[i]->GetLastSensorTimestamp();
		//
		imu_linear_accelerations_[i][0] = navXs_[i]->GetWorldLinearAccelX();
		imu_linear_accelerations_[i][1] = navXs_[i]->GetWorldLinearAccelY();
		imu_linear_accelerations_[i][2] = navXs_[i]->GetWorldLinearAccelZ();

		//navXs_[i]->IsMoving();
		//navXs_[i]->IsRotating();
		//navXs_[i]->IsMagneticDisturbance();
		//navXs_[i]->IsMagnetometerCalibrated();
		//
		tf2::Quaternion tempQ;
		tempQ.setRPY(navXs_[i]->GetRoll() / -360 * 2 * M_PI, navXs_[i]->GetPitch() / -360 * 2 * M_PI, navXs_[i]->GetFusedHeading() / -360 * 2 * M_PI - navX_command_[i] + M_PI);

		imu_orientations_[i][3] = tempQ.w();
		imu_orientations_[i][0] = tempQ.x();
		imu_orientations_[i][1] = tempQ.y();
		imu_orientations_[i][2] = tempQ.z();

		imu_angular_velocities_[i][0] = navXs_[i]->GetVelocityX();
		imu_angular_velocities_[i][1] = navXs_[i]->GetVelocityY();
		imu_angular_velocities_[i][2] = navXs_[i]->GetVelocityZ();

		//navXs_[i]->GetDisplacementX();
		//navXs_[i]->GetDisplacementY();
		//navXs_[i]->GetDisplacementZ();
		//navXs_[i]->GetAngle(); //continous
		//TODO: add setter functions
		
		navX_state_[i] = navX_command_[i];
	}
	for (size_t i = 0; i < num_compressors_; i++)
	{
		compressor_state_[i] = compressors_[i]->GetCompressorCurrent();
	}
	//navX read here
	
	//read info from the PDP hardware
	auto &ps = pdp_state_;
	static int32_t status = 0;
	ps.setVoltage(HAL_GetPDPVoltage(0, &status));
	ps.setTemperature(HAL_GetPDPTemperature(0, &status));
	ps.setTotalCurrent(HAL_GetPDPTotalCurrent(0, &status));
	ps.setTotalPower(HAL_GetPDPTotalPower(0, &status));
	//ROS_INFO_STREAM("status after total power is" << status);
	//ROS_INFO_STREAM("status after setting to zero is" << status);
	ps.setTotalEnergy(HAL_GetPDPTotalEnergy(0, &status));
	//ROS_INFO_STREAM("status RIIIIIGHT before current is: " << status << ".........................");
	for(int channel = 0; channel <= 15; channel++)
	{
		ps.setCurrent(HAL_GetPDPChannelCurrent(0, channel, &status), channel);
	}

	//ROS_INFO_STREAM("status is: " << status << ".........................");

	/*ps.setVoltage(pdp_joint_.GetVoltage());
	ps.setTemperature(pdp_joint_.GetTemperature());
	ps.setTotalCurrent(pdp_joint_.GetTotalCurrent());
	ps.setTotalPower(pdp_joint_.GetTotalPower());
	ps.setTotalEnergy(pdp_joint_.GetTotalEnergy());
	for(int channel = 0; channel <= 15; channel++)
	{
		ps.setCurrent(pdp_joint_.GetCurrent(channel), channel);
	}*/
}

double FRCRobotHWInterface::getConversionFactor(int encoder_ticks_per_rotation,
						hardware_interface::FeedbackDevice encoder_feedback,
						hardware_interface::TalonMode talon_mode,
						int joint_id)
{
	if(talon_mode == hardware_interface::TalonMode_Position)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2 * M_PI / encoder_ticks_per_rotation;
			case hardware_interface::FeedbackDevice_Analog:
				return 2 * M_PI / 1024;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
				return 1.;
		}
	}
	else if(talon_mode == hardware_interface::TalonMode_Velocity)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2 * M_PI / encoder_ticks_per_rotation / .1;
			case hardware_interface::FeedbackDevice_Analog:
				return 2 * M_PI / 1024 / .1;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
				return 1.;
		}
	}
	else
	{
		//ROS_WARN_STREAM("Unable to convert closed loop units.");
		return 1.;
	}
}

bool FRCRobotHWInterface::safeTalonCall(ctre::phoenix::ErrorCode error_code, const std::string &talon_method_name)
{
	std::string error_name;
	switch (error_code)
	{
		case ctre::phoenix::OK :
			return true; // Yay us!

		case ctre::phoenix::CAN_MSG_STALE :
			error_name = "CAN_MSG_STALE/CAN_TX_FULL/TxFailed";
			break;
		case ctre::phoenix::InvalidParamValue :
			error_name = "InvalidParamValue/CAN_INVALID_PARAM";
			break;

		case ctre::phoenix::RxTimeout :
			error_name = "RxTimeout/CAN_MSG_NOT_FOUND";
			break;
		case ctre::phoenix::TxTimeout :
			error_name = "TxTimeout/CAN_NO_MORE_TX_JOBS";
			break;
		case ctre::phoenix::UnexpectedArbId :
			error_name = "UnexpectedArbId/CAN_NO_SESSIONS_AVAIL";
			break;
		case ctre::phoenix::BufferFull :
			error_name = "BufferFull/CAN_OVERFLOW";
			break;
		case ctre::phoenix::SensorNotPresent :
			error_name = "SensorNotPresent";
			break;
		case ctre::phoenix::FirmwareTooOld :
			error_name = "FirmwareTooOld";
			break;
		case ctre::phoenix::CouldNotChangePeriod :
			error_name = "CouldNotChangePeriod";
			break;

		case ctre::phoenix::GENERAL_ERROR :
			error_name = "GENERAL_ERROR";
			break;

		case ctre::phoenix::SIG_NOT_UPDATED :
			error_name = "SIG_NOT_UPDATED";
			break;
		case ctre::phoenix::NotAllPIDValuesUpdated :
			error_name = "NotAllPIDValuesUpdated";
			break;

		case ctre::phoenix::GEN_PORT_ERROR :
			error_name = "GEN_PORT_ERROR";
			break;
		case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH :
			error_name = "PORT_MODULE_TYPE_MISMATCH";
			break;

		case ctre::phoenix::GEN_MODULE_ERROR :
			error_name = "GEN_MODULE_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR :
			error_name = "MODULE_NOT_INIT_SET_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR :
			error_name = "MODULE_NOT_INIT_GET_ERROR";
			break;

		case ctre::phoenix::WheelRadiusTooSmall :
			error_name = "WheelRadiusTooSmall";
			break;
		case ctre::phoenix::TicksPerRevZero :
			error_name = "TicksPerRevZero";
			break;
		case ctre::phoenix::DistanceBetweenWheelsTooSmall :
			error_name = "DistanceBetweenWheelsTooSmall";
			break;
		case ctre::phoenix::GainsAreNotSet :
			error_name = "GainsAreNotSet";
			break;
		case ctre::phoenix::IncompatibleMode :
			error_name = "IncompatibleMode";
			break;
		case ctre::phoenix::InvalidHandle :
			error_name = "InvalidHandle";
			break;


		case ctre::phoenix::PulseWidthSensorNotPresent :
			error_name = "PulseWidthSensorNotPresent";
			break;
		case ctre::phoenix::GeneralWarning :
			error_name = "GeneralWarning";
			break;
		case ctre::phoenix::FeatureNotSupported :
			error_name = "FeatureNotSupported";
			break;
		case ctre::phoenix::NotImplemented :
			error_name = "NotImplemented";
			break;
		case ctre::phoenix::FirmVersionCouldNotBeRetrieved :
			error_name = "FirmVersionCouldNotBeRetrieved";
			break;
		case ctre::phoenix::FeaturesNotAvailableYet :
			error_name = "FeaturesNotAvailableYet";
			break;
		case ctre::phoenix::ControlModeNotValid :
			error_name = "ControlModeNotValid";
			break;

		case ctre::phoenix::ControlModeNotSupportedYet :
			error_name = "case";
			break;
		case ctre::phoenix::CascadedPIDNotSupporteYet:
			error_name = "CascadedPIDNotSupporteYet";
			break;
		case ctre::phoenix::RemoteSensorsNotSupportedYet:
			error_name = "RemoteSensorsNotSupportedYet";
			break;

	}
	//ROS_ERROR_STREAM("Error calling " << talon_method_name << " : " << error_name);
	return false;
}

void FRCRobotHWInterface::write(ros::Duration &elapsed_time)
{
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		//TODO : skip over most or all of this if the talon is in follower mode
		//       Only do the Set() call and then
		//       never do anything else?  Need to make sure things like inverts
		//       and so on are copied from the talon it is following - RG inverts shouldn't
		//       be copied, we may need to run a slave inverted relative to master
		//
		// Save some typing by making references to commonly
		// used variables
		auto &talon = can_talons_[joint_id];

		if (!talon) // skip unintialized Talons
			continue;

		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];

		hardware_interface::FeedbackDevice internal_feedback_device;
		ctre::phoenix::motorcontrol::FeedbackDevice talon_feedback_device;
		if (tc.encoderFeedbackChanged(internal_feedback_device) &&
			convertFeedbackDevice(internal_feedback_device, talon_feedback_device))
		{
			safeTalonCall(talon->ConfigSelectedFeedbackSensor(talon_feedback_device, pidIdx, timeoutMs),"ConfigSelectedFeedbackSensor");
			ts.setEncoderFeedback(internal_feedback_device);
		}

		const hardware_interface::TalonMode talon_mode = ts.getTalonMode();
		const int encoder_ticks_per_rotation = tc.getEncoderTicksPerRotation();
		ts.setEncoderTicksPerRotation(encoder_ticks_per_rotation);

		double conversion_factor;
		if (tc.conversionFactorChanged(conversion_factor))
			ts.setConversionFactor(conversion_factor);

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Position, joint_id) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Velocity, joint_id) * conversion_factor;
		const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, talon_mode, joint_id) * conversion_factor;

		bool close_loop_mode = false;
		bool motion_profile_mode = false;

		if ((talon_mode == hardware_interface::TalonMode_Position) ||
		    (talon_mode == hardware_interface::TalonMode_Velocity) ||
		    (talon_mode == hardware_interface::TalonMode_Current ))
		{
			close_loop_mode = true;
		}
		else if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			     (talon_mode == hardware_interface::TalonMode_MotionMagic))
		{
			close_loop_mode = true;
			motion_profile_mode = true;
		}

		if (close_loop_mode)
		{
			int slot;
			const bool slot_changed = tc.slotChanged(slot);
			
			double p;
			double i;
			double d;
			double f;
			int    iz;
			int    allowable_closed_loop_error;
			double max_integral_accumulator;

			if (tc.pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, slot))
			{
				safeTalonCall(talon->Config_kP(slot, p, timeoutMs),"Config_kP");
				safeTalonCall(talon->Config_kI(slot, i, timeoutMs),"Config_kI");
				safeTalonCall(talon->Config_kD(slot, d, timeoutMs),"Config_kD");
				safeTalonCall(talon->Config_kF(slot, f, timeoutMs),"Config_kF");
				safeTalonCall(talon->Config_IntegralZone(slot, iz, timeoutMs),"Config_IntegralZone");
				// TODO : Scale these two?
				safeTalonCall(talon->ConfigAllowableClosedloopError(slot, allowable_closed_loop_error, timeoutMs),"ConfigAllowableClosedloopError");
				safeTalonCall(talon->ConfigMaxIntegralAccumulator(slot, max_integral_accumulator, timeoutMs),"ConfigMaxIntegralAccumulator");

				ts.setPidfP(p, slot);
				ts.setPidfI(i, slot);
				ts.setPidfD(d, slot);
				ts.setPidfF(f, slot);
				ts.setPidfIzone(iz, slot);
				ts.setAllowableClosedLoopError(allowable_closed_loop_error, slot);
				ts.setMaxIntegralAccumulator(max_integral_accumulator, slot);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" PIDF slot " << slot << " config values");
			}

			if (slot_changed)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF slot to " << slot << std::endl);

				safeTalonCall(talon->SelectProfileSlot(slot, timeoutMs),"SelectProfileSlot");
				ts.setSlot(slot);
			}
		}

		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			talon->SetInverted(invert);
			safeTalonCall(talon->GetLastError(), "SetInverted");
			talon->SetSensorPhase(sensor_phase);
			safeTalonCall(talon->GetLastError(), "SetSensorPhase");
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" invert / phase");
		}

		hardware_interface::NeutralMode neutral_mode;
		ctre::phoenix::motorcontrol::NeutralMode ctre_neutral_mode;
		if (tc.neutralModeChanged(neutral_mode) &&
			convertNeutralMode(neutral_mode, ctre_neutral_mode))
		{
			talon->SetNeutralMode(ctre_neutral_mode);
			safeTalonCall(talon->GetLastError(), "SetNeutralMode");
			ts.setNeutralMode(neutral_mode);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral mode");
		}

		if (tc.neutralOutputChanged())
		{
			talon->NeutralOutput();
			safeTalonCall(talon->GetLastError(), "NeutralOutput");
			ts.setNeutralOutput(true);
			ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral output");
		}

		double iaccum;
		if (close_loop_mode && tc.integralAccumulatorChanged(iaccum))
		{
			safeTalonCall(talon->SetIntegralAccumulator(iaccum / closed_loop_scale, pidIdx, timeoutMs),"SetIntegralAccumulator");
			// Do not set talon state - this changes
			// dynamically so read it in read() above instead
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" integral accumulator");
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
			safeTalonCall(talon->ConfigOpenloopRamp(open_loop_ramp, timeoutMs),"ConfigOpenloopRamp");
			safeTalonCall(talon->ConfigClosedloopRamp(closed_loop_ramp, timeoutMs),"ConfigClosedloopRamp");
			safeTalonCall(talon->ConfigPeakOutputForward(peak_output_forward, timeoutMs),"ConfigPeakOutputForward");          // 100
			safeTalonCall(talon->ConfigPeakOutputReverse(peak_output_reverse, timeoutMs),"ConfigPeakOutputReverse");          // -100
			safeTalonCall(talon->ConfigNominalOutputForward(nominal_output_forward, timeoutMs),"ConfigNominalOutputForward"); // 0
			safeTalonCall(talon->ConfigNominalOutputReverse(nominal_output_reverse, timeoutMs),"ConfigNominalOutputReverse"); // 0
			safeTalonCall(talon->ConfigNeutralDeadband(neutral_deadband, timeoutMs),"ConfigNeutralDeadband");                 // 0

			ts.setOpenloopRamp(open_loop_ramp);
			ts.setClosedloopRamp(closed_loop_ramp);
			ts.setPeakOutputForward(peak_output_forward);
			ts.setPeakOutputReverse(peak_output_reverse);
			ts.setNominalOutputForward(nominal_output_forward);
			ts.setNominalOutputReverse(nominal_output_reverse);
			ts.setNeutralDeadband(neutral_deadband);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" output shaping");
		}
		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.VoltageCompensationChanged(v_c_saturation,
										  v_measurement_filter,
										  v_c_enable))
		{
			safeTalonCall(talon->ConfigVoltageCompSaturation(v_c_saturation, timeoutMs),"ConfigVoltageCompSaturation");
			safeTalonCall(talon->ConfigVoltageMeasurementFilter(v_measurement_filter, timeoutMs),"ConfigVoltageMeasurementFilter");
			talon->EnableVoltageCompensation(v_c_enable);
			safeTalonCall(talon->GetLastError(), "EnableVoltageCompensation");

			ts.setVoltageCompensationSaturation(v_c_saturation);
			ts.setVoltageMeasurementFilter(v_measurement_filter);
			ts.setVoltageCompensationEnable(v_c_enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" voltage compensation");
		}

		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			safeTalonCall(talon->SetSelectedSensorPosition(sensor_position / radians_scale, pidIdx, timeoutMs),
					"SetSelectedSensorPosition");
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" selected sensor position");

		}

		hardware_interface::LimitSwitchSource internal_local_forward_source;
		hardware_interface::LimitSwitchNormal internal_local_forward_normal;
		hardware_interface::LimitSwitchSource internal_local_reverse_source;
		hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_forward_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_forward_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_reverse_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_reverse_normal;
		if (tc.limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
										  internal_local_reverse_source, internal_local_reverse_normal) &&
				convertLimitSwitchSource(internal_local_forward_source, talon_local_forward_source) &&
				convertLimitSwitchNormal(internal_local_forward_normal, talon_local_forward_normal) &&
				convertLimitSwitchSource(internal_local_reverse_source, talon_local_reverse_source) &&
				convertLimitSwitchNormal(internal_local_reverse_normal, talon_local_reverse_normal) )
		{
			safeTalonCall(talon->ConfigForwardLimitSwitchSource(talon_local_forward_source, talon_local_forward_normal, timeoutMs),"ConfigForwardLimitSwitchSource");
			safeTalonCall(talon->ConfigReverseLimitSwitchSource(talon_local_reverse_source, talon_local_reverse_normal, timeoutMs),"ConfigReverseLimitSwitchSource");
			ts.setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
			ts.setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" limit switches");
		}

		double softlimit_forward_threshold;
		bool softlimit_forward_enable;
		double softlimit_reverse_threshold;
		bool softlimit_reverse_enable;
		bool softlimit_override_enable;
		if (tc.SoftLimitChanged(softlimit_forward_threshold,
				softlimit_forward_enable,
				softlimit_reverse_threshold,
				softlimit_reverse_enable,
				softlimit_override_enable))
		{
			double softlimit_forward_threshold_NU = softlimit_forward_threshold / radians_scale; //native units
			double softlimit_reverse_threshold_NU = softlimit_reverse_threshold / radians_scale;
			talon->OverrideSoftLimitsEnable(softlimit_override_enable);
			safeTalonCall(talon->GetLastError(), "OverrideSoftLimitsEnable");
			safeTalonCall(talon->ConfigForwardSoftLimitThreshold(softlimit_forward_threshold_NU, timeoutMs),"ConfigForwardSoftLimitThreshold");
			safeTalonCall(talon->ConfigForwardSoftLimitEnable(softlimit_forward_enable, timeoutMs),"ConfigForwardSoftLimitEnable");
			safeTalonCall(talon->ConfigReverseSoftLimitThreshold(softlimit_reverse_threshold_NU, timeoutMs),"ConfigReverseSoftLimitThreshold");
			safeTalonCall(talon->ConfigReverseSoftLimitEnable(softlimit_reverse_enable, timeoutMs),"ConfigReverseSoftLimitEnable");

			ts.setOverrideSoftLimitsEnable(softlimit_override_enable);
			ts.setForwardSoftLimitThreshold(softlimit_forward_threshold);
			ts.setForwardSoftLimitEnable(softlimit_forward_enable);
			ts.setReverseSoftLimitThreshold(softlimit_reverse_threshold);
			ts.setReverseSoftLimitEnable(softlimit_reverse_enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" soft limits " <<
					std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
					std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
					std::endl << "\toverride_enable=" << softlimit_override_enable);
		}

		int peak_amps;
		int peak_msec;
		int continuous_amps;
		bool enable;
		if (tc.currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
		{
			safeTalonCall(talon->ConfigPeakCurrentLimit(peak_amps, timeoutMs),"ConfigPeakCurrentLimit");
			safeTalonCall(talon->ConfigPeakCurrentDuration(peak_msec, timeoutMs),"ConfigPeakCurrentDuration");
			safeTalonCall(talon->ConfigContinuousCurrentLimit(continuous_amps, timeoutMs),"ConfigContinuousCurrentLimit");
			talon->EnableCurrentLimit(enable);
			safeTalonCall(talon->GetLastError(), "EnableCurrentLimit");

			ts.setPeakCurrentLimit(peak_amps);
			ts.setPeakCurrentDuration(peak_msec);
			ts.setContinuousCurrentLimit(continuous_amps);
			ts.setCurrentLimitEnable(enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" peak current");
		}

		if (motion_profile_mode)
		{
			double motion_cruise_velocity;
			double motion_acceleration;
			if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration))
			{
				//converted from rad/sec to native units
				safeTalonCall(talon->ConfigMotionCruiseVelocity((motion_cruise_velocity / radians_per_second_scale), timeoutMs),"ConfigMotionCruiseVelocity(");
				safeTalonCall(talon->ConfigMotionAcceleration((motion_acceleration / radians_per_second_scale), timeoutMs),"ConfigMotionAcceleration(");

				ts.setMotionCruiseVelocity(motion_cruise_velocity);
				ts.setMotionAcceleration(motion_acceleration);

				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" cruise velocity / acceleration");
			}
			// Do this before rest of motion profile stuff
			// so it takes effect before starting a buffer?
			int motion_control_frame_period;
			if (tc.motionControlFramePeriodChanged(motion_control_frame_period))
			{
				safeTalonCall(talon->ChangeMotionControlFramePeriod(motion_control_frame_period),"ChangeMotionControlFramePeriod");
				ts.setMotionControlFramePeriod(motion_control_frame_period);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion control frame period");
			}

			if (tc.clearMotionProfileTrajectoriesChanged())
			{
				talon->ClearMotionProfileTrajectories();
				safeTalonCall(talon->GetLastError(), "ClearMotionProfileTrajectories");

				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");
			}

			if (tc.clearMotionProfileHasUnderrunChanged())
			{
				safeTalonCall(talon->ClearMotionProfileHasUnderrun(timeoutMs),"ClearMotionProfileHasUnderrun");
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile underrun changed");

			}

			// TODO : check that Talon motion buffer is not full
			// before writing, communicate how many have been written
			// - and thus should be cleared - from the talon_command
			// list of requests.
			std::vector<hardware_interface::TrajectoryPoint> trajectory_points;
			if (tc.motionProfileTrajectoriesChanged(trajectory_points))
			{
				for (auto it = trajectory_points.cbegin(); it != trajectory_points.cend(); ++it)
				{
					//ROS_WARN("SENDING_POINT");

					//This loop doesn't work for whatever reason
					ctre::phoenix::motion::TrajectoryPoint pt;
					pt.position = it->position / radians_scale;
					pt.velocity = it->velocity / radians_per_second_scale;
					pt.headingDeg = it->headingRad * 180. / M_PI;
					pt.profileSlotSelect0 = it->profileSlotSelect0;
					pt.profileSlotSelect1 = it->profileSlotSelect1;
					pt.isLastPoint = it->isLastPoint;
					pt.zeroPos = it->zeroPos;
					pt.timeDur = static_cast<ctre::phoenix::motion::TrajectoryDuration>(it->trajectoryDuration);
					safeTalonCall(talon->PushMotionProfileTrajectory(pt),"PushMotionProfileTrajectory");
				}
				// Copy the 1st profile trajectory point from
				// the top level buffer to the talon
				// Subsequent points will be copied by
				// the process_motion_profile_buffer_thread code
				talon->ProcessMotionProfileBuffer();

				ROS_INFO_STREAM("Added joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");
			}
		}

		// Set new motor setpoint if either the mode or
		// the setpoint has been changed
		double command;
		hardware_interface::TalonMode in_mode;
		ctre::phoenix::motorcontrol::ControlMode out_mode;
		// Note thie has to be | rather than ||
		// Using || gives a chance of it being short-circuted ...
		// that is, if newMode is true commandChanged won't
		// be called.  That' bad because then command would
		// be undefined
		if ((tc.newMode(in_mode) | tc.commandChanged(command) ) &&
			convertControlMode(in_mode, out_mode))
		{
			ts.setTalonMode(in_mode);
			ts.setSetpoint(command);
			ts.setNeutralOutput(false); // maybe make this a part of setSetpoint?

			switch (out_mode)
			{
				case ctre::phoenix::motorcontrol::ControlMode::Velocity:
					command /= radians_per_second_scale;
					break;
				case ctre::phoenix::motorcontrol::ControlMode::Position:
					command /= radians_scale;
					break;
				case ctre::phoenix::motorcontrol::ControlMode::MotionMagic:
					command /= radians_scale;
					break;
			}

			//ROS_INFO_STREAM("in mode: " << in_mode);
			talon->Set(out_mode, command);
			safeTalonCall(talon->GetLastError(), "Set");
		}

		if (tc.clearStickyFaultsChanged())
		{
			safeTalonCall(talon->ClearStickyFaults(timeoutMs), "ClearStickyFaults");
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" sticky_faults");
		}
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		nidec_brushlesses_[i]->Set(brushless_command_[i]);
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		const bool converted_command = (digital_output_command_[i] > 0) ^ digital_output_inverts_[i];
		digital_outputs_[i]->Set(converted_command);
	}
	for (size_t i = 0; i < num_pwm_; i++)

	{
		const int inverter = (pwm_inverts_[i]) ? -1 : 1;
		PWMs_[i]->SetSpeed(pwm_command_[i]*inverter);
	}
	for (size_t i = 0; i< num_solenoids_; i++)
	{
		const bool setpoint = solenoid_command_[i] > 0;
		solenoids_[i]->Set(setpoint);
	}

	for (size_t i = 0; i< num_double_solenoids_; i++)
	{
		DoubleSolenoid::Value setpoint = DoubleSolenoid::Value::kOff;
		if (double_solenoid_command_[i] >= 1.0)
			setpoint = DoubleSolenoid::Value::kForward;
		else if (double_solenoid_command_[i] <= -1.0)
			setpoint = DoubleSolenoid::Value::kReverse;

		double_solenoids_[i]->Set(setpoint);
	}
	for (size_t i = 0; i < num_rumble_; i++)
	{
		const unsigned int rumbles = *((unsigned int*)(&rumble_command_[i]));
		const unsigned int left_rumble  = (rumbles >> 16) & 0xFFFF;
		const unsigned int right_rumble = (rumbles      ) & 0xFFFF;
		HAL_SetJoystickOutputs(rumble_ports_[i], 0, left_rumble, right_rumble);
	}
	for (size_t i = 0; i< num_compressors_; i++)
	{
		const bool setpoint = compressor_command_[i] > 0;
		compressors_[i]->SetClosedLoopControl(setpoint);
	}
}

// Convert from internal version of hardware mode ID
// to one to write to actual Talon hardware
// Return true if conversion is OK, false if
// an unknown mode is hit.
bool FRCRobotHWInterface::convertControlMode(
	const hardware_interface::TalonMode input_mode,
	ctre::phoenix::motorcontrol::ControlMode &output_mode)
{
	switch (input_mode)
	{
	case hardware_interface::TalonMode_PercentOutput:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
		break;
	case hardware_interface::TalonMode_Position:      // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Position;
		break;
	case hardware_interface::TalonMode_Velocity:      // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
		break;
	case hardware_interface::TalonMode_Current:       // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Current;
		break;
	case hardware_interface::TalonMode_Follower:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Follower;
		break;
	case hardware_interface::TalonMode_MotionProfile:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionProfile;
		break;
	case hardware_interface::TalonMode_MotionMagic:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
		break;
	//case hardware_interface::TalonMode_TimedPercentOutput:
		//output_mode = ctre::phoenix::motorcontrol::ControlMode::TimedPercentOutput;
		//output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		//ROS_WARN("TimedPercentOutput mode seen in HW interface");
		//break;
	case hardware_interface::TalonMode_Disabled:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		break;
	default:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		ROS_WARN("Unknown mode seen in HW interface");
		return false;
	}

	return true;
}

bool FRCRobotHWInterface::convertNeutralMode(
	const hardware_interface::NeutralMode input_mode,
	ctre::phoenix::motorcontrol::NeutralMode &output_mode)
{
	switch (input_mode)
	{
	case hardware_interface::NeutralMode_EEPROM_Setting:
		output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
		break;
	case hardware_interface::NeutralMode_Coast:
		output_mode = ctre::phoenix::motorcontrol::Coast;
		break;
	case hardware_interface::NeutralMode_Brake:
		output_mode = ctre::phoenix::motorcontrol::Brake;
		break;
	default:
		output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
		ROS_WARN("Unknown neutral mode seen in HW interface");
		return false;
	}

	return true;
}

bool FRCRobotHWInterface::convertFeedbackDevice(
	const hardware_interface::FeedbackDevice input_fd,
	ctre::phoenix::motorcontrol::FeedbackDevice &output_fd)
{
	switch (input_fd)
	{
	case hardware_interface::FeedbackDevice_QuadEncoder:
		output_fd = ctre::phoenix::motorcontrol::QuadEncoder;
		break;
	case hardware_interface::FeedbackDevice_Analog:
		output_fd = ctre::phoenix::motorcontrol::Analog;
		break;
	case hardware_interface::FeedbackDevice_Tachometer:
		output_fd = ctre::phoenix::motorcontrol::Tachometer;
		break;
	case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
		output_fd = ctre::phoenix::motorcontrol::PulseWidthEncodedPosition;
		break;
	case hardware_interface::FeedbackDevice_SensorSum:
		output_fd = ctre::phoenix::motorcontrol::SensorSum;
		break;
	case hardware_interface::FeedbackDevice_SensorDifference:
		output_fd = ctre::phoenix::motorcontrol::SensorDifference;
		break;
	case hardware_interface::FeedbackDevice_RemoteSensor0:
		output_fd = ctre::phoenix::motorcontrol::RemoteSensor0;
		break;
	case hardware_interface::FeedbackDevice_RemoteSensor1:
		output_fd = ctre::phoenix::motorcontrol::RemoteSensor1;
		break;
	case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
		output_fd = ctre::phoenix::motorcontrol::SoftwareEmulatedSensor;
		break;
	default:
		ROS_WARN("Unknown feedback device seen in HW interface");
		return false;
	}
	return true;
}

bool FRCRobotHWInterface::convertLimitSwitchSource(
	const hardware_interface::LimitSwitchSource input_ls,
	ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls)
{
	switch (input_ls)
	{
	case hardware_interface::LimitSwitchSource_FeedbackConnector:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
		break;
	case hardware_interface::LimitSwitchSource_RemoteTalonSRX:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
		break;
	case hardware_interface::LimitSwitchSource_RemoteCANifier:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteCANifier;
		break;
	case hardware_interface::LimitSwitchSource_Deactivated:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
		break;
	default:
		ROS_WARN("Unknown limit switch source seen in HW interface");
		return false;
	}
	return true;
}

bool FRCRobotHWInterface::convertLimitSwitchNormal(
	const hardware_interface::LimitSwitchNormal input_ls,
	ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls)
{
	switch (input_ls)
	{
	case hardware_interface::LimitSwitchNormal_NormallyOpen:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
		break;
	case hardware_interface::LimitSwitchNormal_NormallyClosed:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
		break;
	case hardware_interface::LimitSwitchNormal_Disabled:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
		break;
	default:
		ROS_WARN("Unknown limit switch normal seen in HW interface");
		return false;
	}
	return true;

}

}
