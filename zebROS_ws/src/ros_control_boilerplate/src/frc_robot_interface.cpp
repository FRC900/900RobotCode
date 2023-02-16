/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
   Desc:   Helper ros_control hardware interface that loads configurations
*/
#include <ros/ros.h>
#include <angles/angles.h>
#include <ros_control_boilerplate/frc_robot_interface.h>
#include "hardware_interface/joint_mode_interface.h"  // for JointCommandModes
#ifdef __linux__
#include <pthread.h>                                  // for pthread_self
//#include <sched.h>                                    // for sched_get_prior...
#endif
#include <algorithm>                                  // for max, all_of
#include <cmath>                                      // for M_PI
#include <cerrno>                                     // for errno
#include <cstring>                                    // for size_t, strerror
#include <cstdint>                                    // for uint8_t, int32_t
#include <iostream>                                   // for operator<<, bas...
//#include "AHRS.h"                                     // for AHRS
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h"
#include "ctre/phoenix/led/CANdle.h"
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include "CTREPDP.h"
#include "frc/AnalogInput.h"                          // for AnalogInput
#include "frc/DigitalInput.h"                         // for DigitalInput
#include "frc/DigitalOutput.h"                        // for DigitalOutput
#include "frc/DoubleSolenoid.h"                       // for DoubleSolenoid
#include "frc/DriverStation.h"                        // for DriverStation
#include "frc/Joystick.h"                             // for Joystick
#include "frc/motorcontrol/NidecBrushless.h"          // for NidecBrushless
#include "frc/PneumaticHub.h"
#include "frc/PneumaticsControlModule.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/Solenoid.h"
#include "frc/PWM.h"                                  // for PWM
#include "hal/CAN.h"                                  // for HAL_CAN_GetCANS...
#include "hal/DriverStation.h"                        // for HAL_GetAlliance...
#include "hal/DriverStationTypes.h"                   // for HAL_ControlWord
#include "hal/HALBase.h"                              // for HAL_GetErrorMes...
#include "hal/Power.h"                                // for HAL_GetVinVoltage
#include "REVPDH.h"
//#include "tf2/LinearMath/Quaternion.h"                // for Quaternion
#include <FRC_NetworkCommunication/FRCComm.h>

#include "ros_control_boilerplate/ros_math_shared.hpp"

struct HAL_JoystickAxesInt {
  int16_t count;
  int16_t axes[HAL_kMaxJoystickAxes];
};

// #define JOYSTICK_LOCK
// #define MATCH_DATA_LOCK

//PURPOSE: Stuff used by to run both hw and sim interfaces
namespace ros_control_boilerplate
{

FRCRobotInterface::FRCRobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) :
	  name_("generic_hw_interface")
	, read_tracer_("FRCRobotInterface " + nh.getNamespace() + "::read()")
	, write_tracer_("FRCRobotInterface " + nh.getNamespace() + "::write()")
{
	// Check if the URDF model needs to be loaded
	if (urdf_model == nullptr)
		loadURDF(nh, "robot_description");
	else
		urdf_model_ = urdf_model;

	// Load rosparams
	ros::NodeHandle rpnh(nh, "hardware_interface"); // TODO(davetcoleman): change the namespace to "frc_robot_interface" aka name_

	readConfig(rpnh);
}

FRCRobotInterface::~FRCRobotInterface()
{
	// Simple lambda function to wait for
	// all valid threads in a vector to exit
	auto join_threads = [](std::vector<std::thread> &threads)
	{
		for (auto &t : threads)
		{
			if (t.joinable())
			{
				t.join();
			}
		}
	};
	join_threads(ctre_mc_read_threads_);
	join_threads(cancoder_read_threads_);
	join_threads(pigeon2_read_threads_);
	join_threads(pcm_threads_);
	join_threads(pdp_threads_);
	join_threads(pdh_threads_);
	join_threads(ph_threads_);
}

bool FRCRobotInterface::initDevices(ros::NodeHandle root_nh)
{
	wpi::math::MathSharedStore::SetMathShared(std::make_unique<ROSMathShared>());

	if (run_hal_robot_)
	{
		// Make sure to initialize WPIlib code before creating
		// a CAN Talon object to avoid NIFPGA: Resource not initialized
		// errors? See https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
		robot_ = std::make_unique<ROSIterativeRobot>();

		/**
		 * Calling this function will load and start
		 * the Phoenix background tasks.
		 *
		 * This can be useful if you need the
		 * Enable/Disable functionality for CAN devices
		 * but aren't using any of the CAN device classes.
		 **/
		ctre::phoenix::unmanaged::Unmanaged::LoadPhoenix();
	}
	else
	{
		// Only run Phoenix tuner server on the Rio, disable it here for the Jetson
		ctre::phoenix::unmanaged::Unmanaged::SetPhoenixDiagnosticsStartTime(-1);
	}
	can_error_count_ = 0;

	//Stuff below is from frcrobot_hw_interface
	for (size_t i = 0; i < num_can_ctre_mcs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << can_ctre_mc_names_[i] <<
							  (can_ctre_mc_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (can_ctre_mc_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as " << (can_ctre_mc_is_talon_fx_[i] ? "TalonFX" : (can_ctre_mc_is_talon_srx_[i] ? "TalonSRX" : "VictorSPX"))
							  << " CAN id " << can_ctre_mc_can_ids_[i] << " on CAN bus \"" << can_ctre_mc_can_busses_[i] << "\"");

		if (can_ctre_mc_local_hardwares_[i])
		{
			if (can_ctre_mc_is_talon_fx_[i])
			{
				ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(can_ctre_mc_can_ids_[i], can_ctre_mc_can_busses_[i]));
			}
			else if (can_ctre_mc_is_talon_srx_[i])
			{
				ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(can_ctre_mc_can_ids_[i]));
			}
			else
			{
				ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(can_ctre_mc_can_ids_[i]));
			}
		}
	}
	ROS_INFO_STREAM("Pausing for CTRE init");
	ros::Duration(.25).sleep();
	ctre_mc_init_time_ = ros::Time::now();
	ROS_INFO_STREAM("Resuming after CTRE init");

	for (size_t i = 0; i < num_can_ctre_mcs_; i++)
	{
		if (can_ctre_mc_local_hardwares_[i])
		{
			ctre_mcs_[i]->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0,
							  ctre::phoenix::motorcontrol::DemandType::DemandType_Neutral, 0);

			// Clear this out so we only get resets that occur after the controllers
			// have been initialized
			ctre_mcs_[i]->HasResetOccurred();

			// Clear sticky faults
			//safeTalonCall(ctre_mcs_[i]->ClearStickyFaults(timeoutMs), "ClearStickyFaults()", can_ctre_mc_can_ids_[i]);

			// TODO : if the motor controller doesn't initialize - maybe known
			// by -1 from firmware version read - somehow tag
			// the entry in ctre_mcs_[] as uninitialized.
			// This probably should be a fatal error
			ROS_INFO_STREAM_NAMED("frc_robot_interface",
								  "Motor " << can_ctre_mc_names_[i] <<
								  " controller firmware version " << ctre_mcs_[i]->GetFirmwareVersion());

			ctre_mc_read_state_mutexes_.push_back(std::make_shared<std::mutex>());
			ctre_mc_read_thread_states_.push_back(std::make_shared<hardware_interface::TalonHWState>(can_ctre_mc_can_ids_[i]));
			ctre_mc_read_threads_.emplace_back(std::thread(&FRCRobotInterface::ctre_mc_read_thread, this,
											   ctre_mcs_[i], ctre_mc_read_thread_states_[i],
											   ctre_mc_read_state_mutexes_[i],
											   std::make_unique<Tracer>("ctre_mc_read_" + can_ctre_mc_names_[i] + " " + root_nh.getNamespace()),
											   i,
											   ctre_mc_read_hz_));
		}
		else
		{
			// Add a null pointer as the can ctre_mc for this index - no
			// actual local hardware identified for it so nothing to create.
			// Just keep the indexes of all the various can_ctre_mc arrays in sync
			ctre_mcs_.push_back(nullptr);
			ctre_mc_read_state_mutexes_.push_back(nullptr);
			ctre_mc_read_thread_states_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_cancoders_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << cancoder_names_[i] <<
							  (cancoder_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (cancoder_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << cancoder_can_ids_[i] << " on CAN bus \"" << cancoder_can_busses_[i] << "\"");

		if (cancoder_local_hardwares_[i])
		{
			cancoders_.emplace_back(std::make_shared<ctre::phoenix::sensors::CANCoder>(cancoder_can_ids_[i], cancoder_can_busses_[i]));
			cancoder_read_state_mutexes_.emplace_back(std::make_shared<std::mutex>());
			cancoder_read_thread_states_.emplace_back(std::make_shared<hardware_interface::cancoder::CANCoderHWState>(cancoder_can_ids_[i]));
			cancoder_read_threads_.emplace_back(std::thread(&FRCRobotInterface::cancoder_read_thread, this,
												cancoders_[i], cancoder_read_thread_states_[i],
												cancoder_read_state_mutexes_[i],
												std::make_unique<Tracer>("cancoder_read_" + cancoder_names_[i] + " " + root_nh.getNamespace()),
												cancoder_read_hz_));
		}
		else
		{
			cancoders_.push_back(nullptr);
			cancoder_read_state_mutexes_.push_back(nullptr);
			cancoder_read_thread_states_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < this->num_candles_; i++) {
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
			"Loading joint (CANdle) " << i << "=" << this->candle_names_[i] <<
			(this->candle_local_updates_[i] ? " local" : " remote") << " update, " <<
			(this->candle_local_hardwares_[i] ? "local" : "remote") << " hardware"
		);

		if (candle_local_hardwares_[i]) {
			this->candles_.emplace_back(std::make_unique<ctre::phoenix::led::CANdle>(this->candle_can_ids_[i], this->candle_can_busses_[i]));
		} else {
			this->candles_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_pigeon2s_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << pigeon2_names_[i] <<
							  (pigeon2_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (pigeon2_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << pigeon2_can_ids_[i] << " on CAN bus \"" << pigeon2_can_busses_[i] << "\"");

		if (pigeon2_local_hardwares_[i])
		{
			pigeon2s_.emplace_back(std::make_shared<ctre::phoenix::sensors::Pigeon2>(pigeon2_can_ids_[i], pigeon2_can_busses_[i]));
			pigeon2_read_state_mutexes_.emplace_back(std::make_shared<std::mutex>());
			pigeon2_read_thread_states_.emplace_back(std::make_shared<hardware_interface::pigeon2::Pigeon2HWState>(pigeon2_can_ids_[i]));
			pigeon2_read_threads_.emplace_back(std::thread(&FRCRobotInterface::pigeon2_read_thread, this,
												pigeon2s_[i], pigeon2_read_thread_states_[i],
												pigeon2_read_state_mutexes_[i],
												std::make_unique<Tracer>("pigeon2_read_" + pigeon2_names_[i] + " " + root_nh.getNamespace()),
												pigeon2_read_hz_));
		}
		else
		{
			pigeon2s_.push_back(nullptr);
			pigeon2_read_state_mutexes_.push_back(nullptr);
			pigeon2_read_thread_states_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << nidec_brushless_names_[i] <<
							  (nidec_brushless_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (nidec_brushless_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as PWM channel " << nidec_brushless_pwm_channels_[i] <<
							  " / DIO channel " << nidec_brushless_dio_channels_[i] <<
							  " invert " << nidec_brushless_inverts_[i]);

		if (nidec_brushless_local_hardwares_[i])
		{
			nidec_brushlesses_.push_back(std::make_shared<frc::NidecBrushless>(nidec_brushless_pwm_channels_[i], nidec_brushless_dio_channels_[i]));
			nidec_brushlesses_[i]->SetInverted(nidec_brushless_inverts_[i]);
		}
		else
			nidec_brushlesses_.push_back(nullptr);
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << digital_input_names_[i] <<
							  " local = " << digital_input_locals_[i] <<
							  " as Digital Input " << digital_input_dio_channels_[i] <<
							  " invert " << digital_input_inverts_[i]);

		if (digital_input_locals_[i])
		{
			bool need_new_hal_din = true;
			for (size_t j = 0; (j < i) && need_new_hal_din; j++)
			{
				if (digital_input_dio_channels_[i] == digital_input_dio_channels_[j])
				{
					digital_inputs_.push_back(digital_inputs_[j]);
					need_new_hal_din = false;
						ROS_WARN_STREAM("DIn " << digital_input_names_[i] <<
								" uses same channel (" << digital_input_dio_channels_[i] <<
								") as " <<  digital_input_names_[j]);
				}
			}
			if (need_new_hal_din)
				digital_inputs_.push_back(std::make_shared<frc::DigitalInput>(digital_input_dio_channels_[i]));
		}
		else
		{
			digital_inputs_.push_back(nullptr);
		}
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << digital_output_names_[i] <<
							  (digital_output_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (digital_output_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Digital Output " << digital_output_dio_channels_[i] <<
							  " invert " << digital_output_inverts_[i]);

		if (digital_output_local_hardwares_[i])
			digital_outputs_.push_back(std::make_shared<frc::DigitalOutput>(digital_output_dio_channels_[i]));
		else
			digital_outputs_.push_back(nullptr);
	}
	for (size_t i = 0; i < num_pwms_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << pwm_names_[i] <<
							  (pwm_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (pwm_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Digitial Output " << pwm_pwm_channels_[i] <<
							  " invert " << pwm_inverts_[i]);

		if (pwm_local_hardwares_[i])
		{
			PWMs_.push_back(std::make_shared<frc::PWM>(pwm_pwm_channels_[i]));
			//PWMs_[i]->SetSafetyEnabled(true);
		}
		else
			PWMs_.push_back(nullptr);
	}

	for (size_t i = 0; i < num_pcms_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << pcm_names_[i] <<
							  (pcm_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (pcm_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as PCM " << pcm_ids_[i]);

		if (pcm_local_hardwares_[i])
		{
			pcms_.push_back(std::make_unique<frc::PneumaticsControlModule>(pcm_ids_[i]));
		    //pcms_.back()->ClearAllStickyFaults();
			ROS_INFO_STREAM("ClearAllStickyFaults() passed");
			if (pcms_[i])
			{
				pcm_read_thread_mutexes_.push_back(std::make_shared<std::mutex>());
				pcm_read_thread_state_.push_back(std::make_shared<hardware_interface::PCMState>(pcm_ids_[i]));
				pcm_threads_.emplace_back(std::thread(&FRCRobotInterface::pcm_read_thread, this,
										  pcms_[i], pcm_read_thread_state_[i],
										  pcm_read_thread_mutexes_[i],
										  std::make_unique<Tracer>("PCM " + pcm_names_[i] + " " + root_nh.getNamespace()),
										  pcm_read_hz_));
				//HAL_Report(HALUsageReporting::kResourceType_Compressor, pcm_ids_[i]);
				//HAL_Report(HALUsageReporting::kResourceType_PCM, pcm_ids_[i]);
			}
			else
			{
				ROS_ERROR_STREAM("PCM init error");
			}
		}
		else
		{
			pcm_read_thread_mutexes_.push_back(nullptr);
			pcm_read_thread_state_.push_back(nullptr);
			pcms_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_phs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << ph_names_[i] <<
							  (ph_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (ph_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as PH " << ph_ids_[i]);

		if (ph_local_hardwares_[i])
		{
			phs_.push_back(std::make_unique<frc::PneumaticHub>(ph_ids_[i]));
			if (phs_[i])
			{
				ph_read_thread_mutexes_.push_back(std::make_shared<std::mutex>());
				ph_read_thread_state_.push_back(std::make_shared<hardware_interface::PHHWState>(ph_ids_[i]));
				ph_threads_.emplace_back(std::thread(&FRCRobotInterface::ph_read_thread, this,
										  phs_[i], ph_read_thread_state_[i],
										  ph_read_thread_mutexes_[i],
										  std::make_unique<Tracer>("PH " + ph_names_[i] + " " + root_nh.getNamespace()),
										  ph_read_hz_));
				//HAL_Report(HALUsageReporting::kResourceType_Compressor, ph_ids_[i]);
				//HAL_Report(HALUsageReporting::kResourceType_PH, ph_ids_[i]);
			}
			else
			{
				ROS_ERROR_STREAM("PH init error");
			}
		}
		else
		{
			ph_read_thread_mutexes_.push_back(nullptr);
			ph_read_thread_state_.push_back(nullptr);
			phs_.push_back(nullptr);
		}
	}


	for (size_t i = 0; i < num_solenoids_; i++)
	{
		std::stringstream s;
		s << "Loading joint " << i << "=" << solenoid_names_[i] <<
			(solenoid_local_updates_[i] ? " local" : " remote") << " update, " <<
			(solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware";
		if (solenoid_local_hardwares_[i])
		{
			s << " as solenoid at channel " << solenoid_channels_[i] <<
				" at " << (solenoid_module_types_[i] == frc::PneumaticsModuleType::CTREPCM ? "ctrepcm" : "revph") <<
				" id " << solenoid_module_ids_[i];
		}
		else
		{
			s << " on remote hardware";
		}

		ROS_INFO_STREAM_NAMED("frc_robot_interface", s.str());

		// Need to have 1 solenoid instantiated on the Rio to get
		// support for compressor and so on loaded?
		if (solenoid_local_hardwares_[i])
		{
			solenoids_.emplace_back(std::make_unique<frc::Solenoid>(solenoid_module_ids_[i], solenoid_module_types_[i], solenoid_channels_[i]));
		}
		else
		{
			solenoids_.emplace_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		std::stringstream s;
		s << "Loading joint " << i << "=" << double_solenoid_names_[i] <<
			(double_solenoid_local_updates_[i] ? " local" : " remote") << " update, " <<
			(double_solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware";

		if (double_solenoid_local_hardwares_[i])
		{
			s << " as double solenoid at forward channel " << double_solenoid_forward_channels_[i] <<
				" & reverse channel " << double_solenoid_reverse_channels_[i] <<
				" at " << (double_solenoid_module_types_[i] == frc::PneumaticsModuleType::CTREPCM ? "ctrepcm" : "revph") <<
				" id " << double_solenoid_module_ids_[i];
		}
		else
		{
			s << " on remote hardware";
		}
		ROS_INFO_STREAM_NAMED("frc_robot_interface", s.str());

		if (double_solenoid_local_hardwares_[i])
		{
			double_solenoids_.emplace_back(std::make_unique<frc::DoubleSolenoid>(double_solenoid_module_ids_[i], double_solenoid_module_types_[i], double_solenoid_forward_channels_[i], double_solenoid_reverse_channels_[i]));
		}
		else
		{
			double_solenoids_.emplace_back(nullptr);
		}
	}

#if 0
	//RIGHT NOW THIS WILL ONLY WORK IF THERE IS ONLY ONE NAVX INSTANTIATED
	for(size_t i = 0; i < num_navX_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
				"Loading joint " << i << "=" << navX_names_[i] <<
				" as navX id " << navX_ids_[i] <<
				" local = " << navX_locals_[i]);
		//TODO: fix how we use ids

		if (navX_locals_[i])
			navXs_.push_back(std::make_shared<AHRS>(frc::SPI::Port::kMXP));
		else
			navXs_.push_back(nullptr);

		// This is a guess so TODO : get better estimates
		imu_orientation_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		imu_angular_velocity_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		imu_linear_acceleration_covariances_[i] ={0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		break; // TODO : only support 1 for now - if we need more, need to define
		       // the interface in config files somehow
	}
#endif

	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << analog_input_names_[i] <<
							  " local = " << analog_input_locals_[i] <<
							  " as Analog Input " << analog_input_analog_channels_[i]);
		if (analog_input_locals_[i])
		{
			bool need_new_hal_ain = true;
			for (size_t j = 0; (j < i) && need_new_hal_ain; j++)
			{
				if (analog_input_analog_channels_[i] == analog_input_analog_channels_[j])
				{
					analog_inputs_.push_back(analog_inputs_[j]);
					need_new_hal_ain = false;
					ROS_WARN_STREAM("AIn " << analog_input_names_[i] <<
							" uses same channel (" << analog_input_analog_channels_[i] <<
							") as " <<  analog_input_names_[j]);
				}
			}
			if (need_new_hal_ain)
				analog_inputs_.push_back(std::make_shared<frc::AnalogInput>(analog_input_analog_channels_[i]));
		}
		else
			analog_inputs_.push_back(nullptr);
	}

	// No real init needed here, just report the config loaded for them
	for (size_t i = 0; i < num_rumbles_; i++)
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << rumble_names_[i] <<
							  (rumble_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (rumble_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Rumble with port " << rumble_ports_[i]);

	for (size_t i = 0; i < num_pdhs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << pdh_names_[i] <<
							  (pdh_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (pdh_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as PDH");

		if (pdh_local_hardwares_[i])
		{
			if (!HAL_CheckREVPDHModuleNumber(pdh_modules_[i]))
			{
				ROS_ERROR_STREAM("Invalid PDH module number " << pdh_modules_[i]);
				pdhs_.push_back(HAL_kInvalidHandle);
			}
			else
			{
				int32_t status = 0;
				const auto pdh_handle = HAL_InitializeREVPDH(pdh_modules_[i], __FUNCTION__, &status);
				pdhs_.push_back(pdh_handle);
				pdh_read_thread_state_.push_back(std::make_shared<hardware_interface::PDHHWState>(pdh_modules_[i]));
				if ((pdh_handle == HAL_kInvalidHandle) || status)
				{
					ROS_ERROR_STREAM("Could not initialize PDH module, status = " << status);
				}
				else
				{
					pdh_read_thread_mutexes_.push_back(std::make_shared<std::mutex>());
					pdh_threads_.emplace_back(std::thread(&FRCRobotInterface::pdh_read_thread, this,
											  pdh_handle, pdh_read_thread_state_[i], pdh_read_thread_mutexes_[i],
											  std::make_unique<Tracer>("PDH " + pdh_names_[i] + " " + root_nh.getNamespace()),
											  pdh_read_hz_));
					//HAL_Report(HALUsageReporting::kResourceType_PDH, pdh_modules_[i]);
				}
			}
		}
		else
		{
			pdh_read_thread_state_.push_back(nullptr);
			pdh_read_thread_mutexes_.push_back(nullptr);
			pdhs_.push_back(HAL_kInvalidHandle);
		}
	}
	for (size_t i = 0; i < num_pdps_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << pdp_names_[i] <<
							  " local=" << pdp_locals_[i] <<
							  " as PDP module " << pdp_modules_[i]);

		if (pdp_locals_[i])
		{
			int32_t status = 0;
			const auto pdp_handle = HAL_InitializePDP(pdp_modules_[i], __FUNCTION__, &status);
			pdp_read_thread_state_.push_back(std::make_shared<hardware_interface::PDPHWState>());
			if ((pdp_handle == HAL_kInvalidHandle) || status)
			{
				ROS_ERROR_STREAM("Could not initialize PDP module, status = " << status);
			}
			else
			{
				pdp_read_thread_mutexes_.push_back(std::make_shared<std::mutex>());
				pdp_threads_.emplace_back(std::thread(&FRCRobotInterface::pdp_read_thread, this,
										  pdp_handle, pdp_read_thread_state_[i], pdp_read_thread_mutexes_[i],
										  std::make_unique<Tracer>("PDP " + pdp_names_[i] + " " + root_nh.getNamespace()),
										  pdp_read_hz_));
				HAL_Report(HALUsageReporting::kResourceType_PDP, pdp_modules_[i]);
			}
		}
		else
		{
			pdp_read_thread_mutexes_.push_back(nullptr);
			pdp_read_thread_state_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_joysticks_; i++)
	{
		ROS_INFO_STREAM_NAMED("frc_robot_interface",
							  "Loading joint " << i << "=" << joystick_names_[i] <<
							  " as joystick with ID " << joystick_ids_[i]);
		joysticks_.push_back(std::make_shared<frc::Joystick>(joystick_ids_[i]));
		joystick_sim_write_mutex_.push_back(std::make_shared<std::mutex>());
	}
	return true;
}

bool FRCRobotInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop");
	if(! param_nh.param("ctre_mc_read_hz", ctre_mc_read_hz_, ctre_mc_read_hz_)) {
		ROS_ERROR("Failed to read ctre_mc_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("cancoder_read_hz", cancoder_read_hz_, cancoder_read_hz_)) {
		ROS_ERROR("Failed to read cancoder_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("canifier_read_hz", canifier_read_hz_, canifier_read_hz_)) {
		ROS_ERROR("Failed to read canifier_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("pigeon2_read_hz", pigeon2_read_hz_, pigeon2_read_hz_)) {
		ROS_ERROR("Failed to read pigeon2_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("spark_max_read_hz", spark_max_read_hz_, spark_max_read_hz_)) {
		ROS_ERROR("Failed to read spark_max_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("pcm_read_hz", pcm_read_hz_, pcm_read_hz_)) {
		ROS_ERROR("Failed to read pcm_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("pdh_read_hz", pdh_read_hz_, pdh_read_hz_)) {
		ROS_ERROR("Failed to read pdh_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("pdp_read_hz", pdp_read_hz_, pdp_read_hz_)) {
		ROS_ERROR("Failed to read pdp_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("ph_read_hz", ph_read_hz_, ph_read_hz_)) {
		ROS_ERROR("Failed to read ph_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("as726x_read_hz", as726x_read_hz_, as726x_read_hz_)) {
		ROS_ERROR("Failed to read as726x_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("robot_iteration_hz", robot_iteration_hz_, robot_iteration_hz_)) {
		ROS_ERROR("Failed to read robot_iteration_hz in frc_robot_interface");
	}
	if(! param_nh.param("joystick_read_hz", joystick_read_hz_, joystick_read_hz_)) {
		ROS_ERROR("Failed to read joystick_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("match_data_read_hz", match_data_read_hz_, match_data_read_hz_)) {
		ROS_ERROR("Failed to read match_data_read_hz in frc_robot_interface");
	}
	if(! param_nh.param("robot_controller_read_hz", robot_controller_read_hz_, robot_controller_read_hz_)) {
		ROS_ERROR("Failed to read robot_controller_read_hz in frc_robot_interface");
	}
	joystick_read_interval_ = std::make_unique<PeriodicIntervalCounter>(joystick_read_hz_);
	match_data_read_interval_ = std::make_unique<PeriodicIntervalCounter>(match_data_read_hz_);
	robot_controller_read_interval_ = std::make_unique<PeriodicIntervalCounter>(robot_controller_read_hz_);
	robot_iteration_interval_ = std::make_unique<PeriodicIntervalCounter>(robot_iteration_hz_);

	ROS_INFO_STREAM("Controller Frequencies:" << std::endl <<
			"\tctre_mc_read : " << ctre_mc_read_hz_ << std::endl <<
			"\tcancoder_read : " << cancoder_read_hz_ << std::endl <<
			"\tcanifier_read : " << canifier_read_hz_ << std::endl <<
			"\tpigeon2_read : " << pigeon2_read_hz_ << std::endl <<
			"\tpcm_read : " << pcm_read_hz_ << std::endl <<
			"\tpdh_read : " << pdh_read_hz_ << std::endl <<
			"\tpdp_read : " << pdp_read_hz_ << std::endl <<
			"\tph_read : " << ph_read_hz_ << std::endl <<
			"\tas726x_read : " << as726x_read_hz_ << std::endl <<
			"\trobot_iteration : " << robot_iteration_hz_ << std::endl <<
			"\tjoystick_read : " << joystick_read_hz_ << std::endl <<
			"\tmatch_data_read : " << match_data_read_hz_ << std::endl <<
			"\trobot_controller_read : " << robot_controller_read_hz_);

	if(! param_nh.param("talon_config_count_limit", talon_config_count_limit_, talon_config_count_limit_)) {
		ROS_ERROR("Failed to read talon_config_count_limit_ in frc_robot_interface");
	}
#ifdef __linux__
#if 0
	struct sched_param schedParam{};

	schedParam.sched_priority = sched_get_priority_min(SCHED_RR);
	const auto rc = pthread_setschedparam(pthread_self(), SCHED_RR, &schedParam);
	if (rc)
	{
		ROS_WARN_STREAM("pthread_setschedparam() returned " << rc
				<< " priority = " << schedParam.sched_priority
				<< " errno = " << errno << " (" << strerror(errno) << ") : Run me as root?");
	}
	else
	{
		ROS_INFO_STREAM("pthread_setschedparam() succeeded");
	}
#endif
	if (pthread_setname_np(pthread_self(), "hwi_main_loop"))
	{
		ROS_ERROR_STREAM("Error setting thread name hwi_main_loop " << errno);
	}
#endif
	createInterfaces();
	ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface Create Interfaces Ready.");
	if (!initDevices(root_nh))
		return false;
	ROS_INFO_STREAM_NAMED(name_, "FRCRobotInterface Devices Ready.");

	ROS_INFO_NAMED("frc_robot_interface", "FRCRobotInterface Ready.");

	return true;
}

void FRCRobotInterface::read(const ros::Time &time, const ros::Duration &period)
{
#if 0
	// TODO : needed for standalone robots, but not
	// when we have a rio attached. Config item?
	read_tracer_.start_unique("FeedEnable");
	if (!run_hal_robot_ && num_can_ctre_mcs_)
	{
		c_FeedEnable(100);
	}
#endif

	read_tracer_.start_unique("Check for ready");
	if (run_hal_robot_ && !robot_code_ready_)
	{
		// This will be written by the last controller to be
		// spawned - waiting here prevents the robot from
		// reporting robot code ready to the field until
		// all other controllers are started
		if (std::all_of(robot_ready_signals_.cbegin(),
						robot_ready_signals_.cend(),
						[](double d) { return d != 0.0;} ))
		{
			robot_->StartCompetition();
			robot_code_ready_ = true;
		}
	}

	if (robot_code_ready_)
	{
		read_tracer_.start_unique("OneIteration");
		//check if sufficient time has passed since last read
		if(robot_iteration_interval_->update(period))
		{
			robot_->OneIteration();
		}

		read_tracer_.start_unique("joysticks");
		if (joystick_read_interval_->update(period))
		{
			// Only update the time count if all joystick state date is updated
			// This will force another update next time through this loop if some
			// of the joystick data wasn't published due to the sim layer holding
			// the mutex for them.
			bool updated_all = true;
			for (size_t joystick = 0; joystick < num_joysticks_; joystick++)
			{
				// In sim, the joystick input code will lock this mutex while
				// it is writing the sim joystick values. If that is in progress
				// skip the read of the joystick data this iteration
#ifdef JOYSTICK_LOCK
				std::unique_lock<std::mutex> l(*(joystick_sim_write_mutex_[joystick]), std::try_to_lock);
				if (l.owns_lock())
#endif
				{
#if 0
					ROS_INFO_STREAM_THROTTLE(0.25, "Reading joystick index " << joystick <<
							" name = " << joystick_names_[joystick] <<
							" id = " << joystick_ids_[joystick]);
#endif
					auto &state = joystick_state_[joystick];
					const auto &stick = joysticks_[joystick];
					state.clear();
					const auto axis_count = stick->GetAxisCount();
					for (auto i = 0; i < axis_count; i++)
						state.addAxis(stick->GetRawAxis(i));
					const auto button_count = stick->GetButtonCount();
					for (auto i = 0; i < button_count; i++)
						state.addButton(stick->GetRawButton(i+1));
					const auto pov_count = stick->GetPOVCount();
					for (auto i = 0; i < pov_count; i++)
						state.addPOV(stick->GetPOV(i));

					// wpilib struct: Declared near top
					HAL_JoystickAxesInt axesInt;
					int retVal = FRC_NetworkCommunication_getJoystickAxes(
									joystick_ids_[joystick], reinterpret_cast<JoystickAxes_t*>(&axesInt),
									HAL_kMaxJoystickAxes);
					for (auto i = 0; i < axesInt.count; i++) {
						uint8_t value = axesInt.axes[i];
						//ROS_INFO_STREAM("Stick=" << joystick_ids_[joystick] << " axis=" << i << " value=" << std::hex << (((unsigned int)value) &0xff));
						state.addRawAxis(value);
					}
				}
#ifdef JOYSTICK_LOCK
				else
				{
					updated_all = false;
				}
#endif
			}
			if (!updated_all)
			{
				joystick_read_interval_->force_publish();
			}
		}

		int32_t status = 0;
		read_tracer_.start_unique("match data");
		//check if sufficient time has passed since last read
#ifdef MATCH_DATA_LOCK
		if (match_data_mutex_.try_lock())
		{
#endif
			if (match_data_read_interval_->update(period))
			{
				status = 0;
				match_data_.setMatchTimeRemaining(HAL_GetMatchTime(&status));
				match_data_.setGetMatchTimeStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
				HAL_MatchInfo info{};
				HAL_GetMatchInfo(&info);

				match_data_.setGameSpecificData(std::vector<uint8_t>(info.gameSpecificMessage, info.gameSpecificMessage + info.gameSpecificMessageSize));
				match_data_.setEventName(info.eventName);

				status = 0;
				auto allianceStationID = HAL_GetAllianceStation(&status);
				match_data_.setGetAllianceStationStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
				frc::DriverStation::Alliance color;
				switch (allianceStationID) {
					case HAL_AllianceStationID_kRed1:
					case HAL_AllianceStationID_kRed2:
					case HAL_AllianceStationID_kRed3:
						color = frc::DriverStation::kRed;
						break;
					case HAL_AllianceStationID_kBlue1:
					case HAL_AllianceStationID_kBlue2:
					case HAL_AllianceStationID_kBlue3:
						color = frc::DriverStation::kBlue;
						break;
					default:
						color = frc::DriverStation::kInvalid;
				}
				match_data_.setAllianceColor(color);

				match_data_.setMatchType(static_cast<frc::DriverStation::MatchType>(info.matchType));

				int station_location;
				switch (allianceStationID) {
					case HAL_AllianceStationID_kRed1:
					case HAL_AllianceStationID_kBlue1:
						station_location = 1;
						break;
					case HAL_AllianceStationID_kRed2:
					case HAL_AllianceStationID_kBlue2:
						station_location = 2;
						break;
					case HAL_AllianceStationID_kRed3:
					case HAL_AllianceStationID_kBlue3:
						station_location = 3;
						break;
					default:
						station_location = 0;
				}
				match_data_.setDriverStationLocation(station_location);

				match_data_.setMatchNumber(info.matchNumber);
				match_data_.setReplayNumber(info.replayNumber);
				status = 0;
				match_data_.setBatteryVoltage(HAL_GetVinVoltage(&status));
				match_data_.setGetVinVoltageStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
			}
			//read control word match data at full speed - contains enable info, and reads should be v fast
			HAL_ControlWord controlWord;
			HAL_GetControlWord(&controlWord);
			match_data_.setEnabled(controlWord.enabled && controlWord.dsAttached);
			match_data_.setDisabled(!(controlWord.enabled && controlWord.dsAttached));
			match_data_.setAutonomous(controlWord.autonomous);
			match_data_.setOperatorControl(!(controlWord.autonomous || controlWord.test));
			match_data_.setTest(controlWord.test);
			match_data_.setDSAttached(controlWord.dsAttached);
			match_data_.setFMSAttached(controlWord.fmsAttached);
			match_data_.setEStopped(controlWord.eStop);
#ifdef MATCH_DATA_LOCK
			match_data_mutex_.unlock();
		}
#endif

		read_tracer_.start_unique("robot controller data");
		//check if sufficient time has passed since last read
		if (robot_controller_read_interval_->update(period))
		{
			status = 0;
			robot_controller_state_.SetFPGAVersion(HAL_GetFPGAVersion(&status));
			robot_controller_state_.SetFPGAVersionStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFPGARevision(HAL_GetFPGARevision(&status));
			robot_controller_state_.SetFPGARevisionStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFPGATime(HAL_GetFPGATime(&status));
			robot_controller_state_.SetFPGATimeStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetUserButton(HAL_GetFPGAButton(&status));
			robot_controller_state_.SetUserButtonStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetIsSysActive(HAL_GetSystemActive(&status));
			robot_controller_state_.SetIsSysActiveStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetIsBrownedOut(HAL_GetBrownedOut(&status));
			robot_controller_state_.SetIsBrownedOutStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetInputVoltage(HAL_GetVinVoltage(&status));
			robot_controller_state_.SetInputVoltageStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetInputCurrent(HAL_GetVinCurrent(&status));
			robot_controller_state_.SetInputCurrentStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetVoltage3V3(HAL_GetUserVoltage3V3(&status));
			robot_controller_state_.SetVoltage3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetCurrent3V3(HAL_GetUserCurrent3V3(&status));
			robot_controller_state_.SetCurrent3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetEnabled3V3(HAL_GetUserActive3V3(&status));
			robot_controller_state_.SetEnabled3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFaultCount3V3(HAL_GetUserCurrentFaults3V3(&status));
			robot_controller_state_.SetFaultCount3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetVoltage5V(HAL_GetUserVoltage5V(&status));
			robot_controller_state_.SetVoltage5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetCurrent5V(HAL_GetUserCurrent5V(&status));
			robot_controller_state_.SetCurrent5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetEnabled5V(HAL_GetUserActive5V(&status));
			robot_controller_state_.SetEnabled5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFaultCount5V(HAL_GetUserCurrentFaults5V(&status));
			robot_controller_state_.SetFaultCount5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetVoltage6V(HAL_GetUserVoltage6V(&status));
			robot_controller_state_.SetVoltage6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetCurrent6V(HAL_GetUserCurrent6V(&status));
			robot_controller_state_.SetCurrent6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetEnabled6V(HAL_GetUserActive6V(&status));
			robot_controller_state_.SetEnabled6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFaultCount6V(HAL_GetUserCurrentFaults6V(&status));
			robot_controller_state_.SetFaultCount6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			float percent_bus_utilization;
			uint32_t bus_off_count;
			uint32_t tx_full_count;
			uint32_t receive_error_count;
			uint32_t transmit_error_count;
			status = 0;
			HAL_CAN_GetCANStatus(&percent_bus_utilization, &bus_off_count,
					&tx_full_count, &receive_error_count,
					&transmit_error_count, &status);

			robot_controller_state_.SetCANPercentBusUtilization(percent_bus_utilization);
			robot_controller_state_.SetCANBusOffCount(bus_off_count);
			robot_controller_state_.SetCANTxFullCount(tx_full_count);
			robot_controller_state_.SetCANReceiveErrorCount(receive_error_count);
			robot_controller_state_.SetCANTransmitErrorCount(transmit_error_count);

			robot_controller_state_.SetCANDataStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
		}
	}
	read_tracer_.start_unique("can talons");
	for (size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		if (can_ctre_mc_local_hardwares_[joint_id])
		{
			std::unique_lock<std::mutex> l(*ctre_mc_read_state_mutexes_[joint_id], std::try_to_lock);
			if (!l.owns_lock())
				continue;
			auto &ts   = talon_state_[joint_id];
			auto &trts = ctre_mc_read_thread_states_[joint_id];

			// Copy config items from talon state to talon_read_thread_state
			// This makes sure config items set by controllers is
			// eventually reflected in the state unique to the
			// talon_read_thread code
			trts->setTalonMode(ts.getTalonMode());
			trts->setEncoderFeedback(ts.getEncoderFeedback());
			trts->setEncoderTicksPerRotation(ts.getEncoderTicksPerRotation());
			trts->setConversionFactor(ts.getConversionFactor());
			trts->setEnableReadThread(ts.getEnableReadThread());
			// There looks like a bug in sim which requires us to read these
			// more slowly.  Pass the previously-read value in to use as
			// a default for iterations where the value isn't read
			trts->setBusVoltage(ts.getBusVoltage());
			trts->setTemperature(ts.getTemperature());

			// Copy talon state values read in the read thread into the
			// talon state shared globally with the rest of the hardware
			// interface code
			ts.setPosition(trts->getPosition());
			ts.setSpeed(trts->getSpeed());
			ts.setOutputCurrent(trts->getOutputCurrent());
			ts.setBusVoltage(trts->getBusVoltage());
			ts.setMotorOutputPercent(trts->getMotorOutputPercent());
			ts.setOutputVoltage(trts->getOutputVoltage());
			ts.setTemperature(trts->getTemperature());
			ts.setClosedLoopError(trts->getClosedLoopError());
			ts.setIntegralAccumulator(trts->getIntegralAccumulator());
			ts.setErrorDerivative(trts->getErrorDerivative());
			ts.setClosedLoopTarget(trts->getClosedLoopTarget());
			ts.setActiveTrajectoryPosition(trts->getActiveTrajectoryPosition());
			ts.setActiveTrajectoryVelocity(trts->getActiveTrajectoryVelocity());
			ts.setActiveTrajectoryHeading(trts->getActiveTrajectoryHeading());
			ts.setMotionProfileTopLevelBufferCount(trts->getMotionProfileTopLevelBufferCount());
			ts.setMotionProfileStatus(trts->getMotionProfileStatus());
			ts.setFaults(trts->getFaults());
			ts.setForwardLimitSwitch(trts->getForwardLimitSwitch());
			ts.setReverseLimitSwitch(trts->getReverseLimitSwitch());
			ts.setForwardSoftlimitHit(trts->getForwardSoftlimitHit());
			ts.setReverseSoftlimitHit(trts->getReverseSoftlimitHit());
			ts.setStickyFaults(trts->getStickyFaults());
			ts.setFirmwareVersion(trts->getFirmwareVersion());
		}
	}

	read_tracer_.start_unique("cancoder");
	for (size_t joint_id = 0; joint_id < num_cancoders_; ++joint_id)
	{
		if (cancoder_local_hardwares_[joint_id])
		{
			std::unique_lock<std::mutex> l(*cancoder_read_state_mutexes_[joint_id], std::try_to_lock);
			if (!l.owns_lock())
				continue;
			auto &cs   = cancoder_state_[joint_id];
			auto &crts = cancoder_read_thread_states_[joint_id];

			// These are used to convert position and velocity units - make sure the
			// read thread's local copy of state is kept up to date
			crts->setConversionFactor(cs.getConversionFactor());

			cs.setPosition(crts->getPosition());
			cs.setVelocity(crts->getVelocity());
			cs.setAbsolutePosition(crts->getAbsolutePosition());
			cs.setBusVoltage(crts->getBusVoltage());
			cs.setMagnetFieldStrength(crts->getMagnetFieldStrength());
			cs.setLastTimestamp(crts->getLastTimestamp());
			cs.setFirmwareVersion(crts->getFirmwareVersion());
			cs.setFaults(crts->getFaults());
			cs.setStickyFaults(crts->getStickyFaults());
		}
	}

	read_tracer_.start_unique("pigeon2");
	for (size_t joint_id = 0; joint_id < num_pigeon2s_; ++joint_id)
	{
		if (pigeon2_local_hardwares_[joint_id])
		{
			// Fill in the full pigeon2 state using the last
			// data read from the pigeon2 read thread for this hardware
			{
				std::unique_lock<std::mutex> l(*pigeon2_read_state_mutexes_[joint_id], std::try_to_lock);
				if (!l.owns_lock())
				{
					continue;
				}
				pigeon2_state_[joint_id] = *pigeon2_read_thread_states_[joint_id];
			}

			// also, fill in the standard ROS imu state info
			// for this.  We really only care about orientation,
			// but do a best effort for the rest
			size_t i = num_navX_ + joint_id;
			const auto ps = &pigeon2_state_[joint_id];

			const auto q = ps->get6dQuatention();
			imu_orientations_[i][3] = q[0];
			imu_orientations_[i][0] = q[1];
			imu_orientations_[i][1] = q[2];
			imu_orientations_[i][2] = q[3];

			auto from_q2_14 = [](uint16_t in)
			{
				double ret = in >> 14;
				ret += (in & 0x3fff) / static_cast<double>(1U << 14);

				return in / 9.80665;
			};
			const auto biased_accelerometer = ps->getBiasedAccelerometer();
			imu_linear_accelerations_[i][0] = from_q2_14(biased_accelerometer[0]);
			imu_linear_accelerations_[i][1] = from_q2_14(biased_accelerometer[1]);
			imu_linear_accelerations_[i][2] = from_q2_14(biased_accelerometer[2]);

			// TODO - figure out how to read this
			imu_angular_velocities_[i][0] = 0;
			imu_angular_velocities_[i][1] = 0;
			imu_angular_velocities_[i][2] = 0;
		}
	}

	read_tracer_.start_unique("nidec");
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		if (nidec_brushless_local_updates_[i])
			brushless_vel_[i] = nidec_brushlesses_[i]->Get();
	}

	read_tracer_.start_unique("digital in");
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		//State should really be a bool - but we're stuck using
		//ROS control code which thinks everything to and from
		//hardware are doubles
		if (digital_input_locals_[i])
			digital_input_state_[i] = (digital_inputs_[i]->Get()^digital_input_inverts_[i]) ? 1 : 0;
	}
#if 0
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		if (!digital_output_local_updates_[i])
			digital_output_state_[i] = digital_output_command_[i];
	}
	for (size_t i = 0; i < num_pwms_; i++)
	{
		// Just reflect state of output in status
		if (!pwm_local_updates_[i])
			pwm_state_[i] = pwm_command_[i];
	}
#endif
	read_tracer_.start_unique("solenoid");
#if 0
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		if (solenoid_local_hardwares_[i])
		{
			//ROS_INFO_STREAM("read : solenoid = " << solenoids_[i]->Get() <<  " " << solenoids_[i]->GetChannel());
			// TODO - only works if robot is enabled, so not sure how best to handle this
			solenoid_state_[i] = solenoids_[i]->Get();
			//if (pcms_.size() && pcms_[0])
				//ROS_INFO_STREAM("pcms_[0]->GetSolenoids() = " << pcms_[0]->GetSolenoids());
		}
	}
#endif

#if 0
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		if (!double_solenoid_local_updates_[i])
			double_solenoid_state_[i] = double_solenoid_command_[i];
	}
#endif
	read_tracer_.start_unique("analog in");
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		if (analog_input_locals_[i])
			analog_input_state_[i] = analog_inputs_[i]->GetValue() *analog_input_a_[i] + analog_input_b_[i];
	}
#if 0
	read_tracer_.start_unique("navX");
	//navX read here
	for (size_t i = 0; i < num_navX_; i++)
	{
		if (navX_locals_[i])
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
			tempQ.setRPY(navXs_[i]->GetRoll()  / -360. * 2. * M_PI,
						 navXs_[i]->GetPitch() / -360. * 2. * M_PI,
						 navXs_[i]->GetYaw()   /  360. * 2. * M_PI);

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
		}
	}
#endif

	read_tracer_.start_unique("pcms");
	for (size_t i = 0; i < num_pcms_; i++)
	{
		if (pcm_local_updates_[i])
		{
			std::unique_lock<std::mutex> l(*pcm_read_thread_mutexes_[i], std::try_to_lock);
			if (l.owns_lock())
			{
				pcm_state_[i] = *pcm_read_thread_state_[i];
			}
		}
	}

	read_tracer_.start_unique("pdhs");
	for (size_t i = 0; i < num_pdhs_; i++)
	{
		if (pdh_local_updates_[i])
		{
			std::unique_lock<std::mutex> l(*pdh_read_thread_mutexes_[i], std::try_to_lock);
			if (l.owns_lock())
			{
				pdh_state_[i] = *pdh_read_thread_state_[i];
			}
		}
	}

	read_tracer_.start_unique("pdps");
	for (size_t i = 0; i < num_pdps_; i++)
	{
		if (pdp_locals_[i])
		{
			std::unique_lock<std::mutex> l(*pdp_read_thread_mutexes_[i], std::try_to_lock);
			if (l.owns_lock())
			{
				pdp_state_[i] = *pdp_read_thread_state_[i];
			}
		}
	}

	read_tracer_.start_unique("phs");
	for (size_t i = 0; i < num_phs_; i++)
	{
		if (ph_local_updates_[i])
		{
			std::unique_lock<std::mutex> l(*ph_read_thread_mutexes_[i], std::try_to_lock);
			if (l.owns_lock())
			{
				ph_state_[i] = *ph_read_thread_state_[i];
			}
		}
	}

	read_tracer_.report(60);
}

void FRCRobotInterface::write(const ros::Time& time, const ros::Duration& period)
{
	// Was the robot enabled last time write was run?
	write_tracer_.start_unique("read match data");
	bool robot_enabled = false;
	{
		std::unique_lock<std::mutex> l(match_data_mutex_, std::try_to_lock);
		if (l.owns_lock())
			robot_enabled = match_data_.isEnabled();
		else
			robot_enabled = last_robot_enabled_;
	}

	write_tracer_.start_unique("ctre mc");
	talon_config_count_ = 0;
	for (size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		if (!can_ctre_mc_local_hardwares_[joint_id])
		{
			continue;
		}
		std::unique_lock<std::mutex> l(*(talon_command_[joint_id].mutex()), std::try_to_lock);
		if (!l.owns_lock())
		{
			continue;
		}

		//TODO : skip over most or all of this if the talon is in follower mode
		//       Only do the Set() call and then never do anything else?

		// Victor is really the common parent of all other motor controllers, functions called via this
		// pointer are common to all controllers
		auto victor = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorController>(ctre_mcs_[joint_id]);
		// This pointer is used to call features which don't exist on victor but do exist on both versions
		// of the talons (feedback sources, current data, etc)
		auto mc_enhanced = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorControllerEnhanced>(ctre_mcs_[joint_id]);
		// Pointers to access specific features from each of these motor controllers
		auto falcon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::WPI_TalonFX>(ctre_mcs_[joint_id]);
		auto talon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>(ctre_mcs_[joint_id]);
#if 0
		if (victor)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "victor OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "victor NOT OK for id " << joint_id);
		}
		if (mc_enhanced)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "mc_enhanced OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "mc_enhanced NOT OK for id " << joint_id);
		}
		if (falcon)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "falcon OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "falcon NOT OK for id " << joint_id);
		}
		if (talon)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "talon OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "talon NOT OK for id " << joint_id);
		}
#endif

		if (!victor && !talon && !mc_enhanced && !falcon) // skip unintialized Talons
		{
			continue;
		}

		// Save some typing by making references to commonly
		// used variables
		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];

		// If the motor controller has been reset since the last write()
		// call, reset all of the flags indicating that commands have been
		// written to the controllers. This will force the config data
		// to be re-written by the rest of the write() function
		if (victor->HasResetOccurred())
		{
			ROS_WARN_STREAM("Detected reset on CTRE mc " << can_ctre_mc_names_[joint_id]);
			for (size_t i = 0; i < hardware_interface::TALON_PIDF_SLOTS; i++)
				tc.resetPIDF(i);
			tc.resetAuxPidPolarity();
			tc.resetIntegralAccumulator();
			tc.resetMode();
			tc.resetDemand1();
			tc.resetPidfSlot();
			tc.resetEncoderFeedback();
			// This should be deprecated anyway -
			// for now, comment it out to prevent it from overwriting
			// the main encoder feedback source above
			// tc.resetRemoteEncoderFeedback();
			tc.resetRemoteFeedbackFilters();
			tc.resetSensorTerms();
			tc.resetOutputShaping();
			tc.resetVoltageCompensation();
			tc.resetVelocityMeasurement();
			//tc.resetSensorPosition();
			tc.resetLimitSwitchesSource();
			tc.resetRemoteLimitSwitchesSource();
			tc.resetSoftLimit();
			tc.resetCurrentLimit();
			tc.resetMotionCruise();
			for (int i = hardware_interface::Status_1_General; i < hardware_interface::Status_Last; i++)
				tc.resetStatusFramePeriod(static_cast<hardware_interface::StatusFrame>(i));
			for (int i = hardware_interface::Control_3_General; i < hardware_interface::Control_Last; i++)
				tc.resetControlFramePeriod(static_cast<hardware_interface::ControlFrame>(i));
			tc.resetMotionProfileTrajectoryPeriod();
			tc.resetSupplyCurrentLimit();
			tc.resetStatorCurrentLimit();
			tc.resetMotorCommutation();
			tc.resetAbsoluteSensorRange();
			tc.resetSensorInitializationStrategy();
			tc.resetClearPositionOnLimitF();
			tc.resetClearPositionOnLimitR();
		}

		if (bool enable_read_thread; tc.enableReadThreadChanged(enable_read_thread))
			ts.setEnableReadThread(enable_read_thread);

		hardware_interface::FeedbackDevice internal_feedback_device = hardware_interface::FeedbackDevice_Uninitialized;
		double feedback_coefficient;

		ctre::phoenix::motorcontrol::FeedbackDevice talon_feedback_device;
		if (tc.encoderFeedbackChanged(internal_feedback_device, feedback_coefficient) &&
			talon_convert_.feedbackDevice(internal_feedback_device, talon_feedback_device))
		{
			// Check for errors on Talon writes. If it fails, used the reset() call to
			// set the changed var for the config items to true. This will trigger a re-try
			// the next time through the loop.
			bool rc = true;
			// Only actually set this on the hardware for Talon devices. But set it in
			// talon_states for both types of motor controllers. This allows the conversion
			// functions to work properly?
			if (mc_enhanced)
			{
				rc = safeTalonConfigCall(mc_enhanced->ConfigSelectedFeedbackSensor(talon_feedback_device, pidIdx, configTimeoutMs),"ConfigSelectedFeedbackSensor", ts.getCANID());
				if (rc)
				{
					rc = safeTalonConfigCall(mc_enhanced->ConfigSelectedFeedbackCoefficient(feedback_coefficient, pidIdx, configTimeoutMs),"ConfigSelectedFeedbackCoefficient", ts.getCANID());
				}
			}
			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " feedback");
				ts.setEncoderFeedback(internal_feedback_device);
				ts.setFeedbackCoefficient(feedback_coefficient);
			}
			else
			{
				ROS_WARN_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " feedback");
				tc.resetEncoderFeedback();
				break;
			}
		}

		ctre::phoenix::motorcontrol::RemoteFeedbackDevice talon_remote_feedback_device;
		hardware_interface::RemoteFeedbackDevice internal_remote_feedback_device;
		if (tc.remoteEncoderFeedbackChanged(internal_remote_feedback_device) &&
			talon_convert_.remoteFeedbackDevice(internal_remote_feedback_device, talon_remote_feedback_device))
		{
			// Check for errors on Talon writes. If it fails, used the reset() call to
			// set the changed var for the config items to true. This will trigger a re-try
			// the next time through the loop.
			if (safeTalonConfigCall(victor->ConfigSelectedFeedbackSensor(talon_remote_feedback_device, pidIdx, configTimeoutMs), "ConfigSelectedFeedbackSensor (Remote)", ts.getCANID()))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " remote feedback sensor");
				ts.setRemoteEncoderFeedback(internal_remote_feedback_device);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " remote feedback sensor");
				tc.resetRemoteEncoderFeedback();
				break;
			}
		}

		std::array<int, 2>                                             remote_feedback_device_ids;
		std::array<hardware_interface::RemoteSensorSource, 2>          internal_remote_feedback_filters;
		std::array<ctre::phoenix::motorcontrol::RemoteSensorSource, 2> victor_remote_feedback_filters;
		if (tc.remoteFeedbackFiltersChanged(remote_feedback_device_ids, internal_remote_feedback_filters) &&
			talon_convert_.remoteSensorSource(internal_remote_feedback_filters[0], victor_remote_feedback_filters[0]) &&
			talon_convert_.remoteSensorSource(internal_remote_feedback_filters[1], victor_remote_feedback_filters[1]))
		{
			bool rc = safeTalonConfigCall(victor->ConfigRemoteFeedbackFilter(remote_feedback_device_ids[0], victor_remote_feedback_filters[0], 0, configTimeoutMs), "ConfigRemoteFeedbackFilter (0)", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigRemoteFeedbackFilter(remote_feedback_device_ids[1], victor_remote_feedback_filters[1], 1, configTimeoutMs), "ConfigRemoteFeedbackFilter (1)", ts.getCANID());
			}
			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " remote feedback filters" <<
						" id[0] = " << remote_feedback_device_ids[0] <<
						" id[1] = " << remote_feedback_device_ids[1] <<
						" filter[0] = " << internal_remote_feedback_filters[0] <<
						" filter[1] = " << internal_remote_feedback_filters[1]);
				ts.setRemoteFeedbackDeviceIds(remote_feedback_device_ids);
				ts.setRemoteFeedbackFilters(internal_remote_feedback_filters);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " remote feedback filters");
				tc.resetRemoteFeedbackFilters();
				break;
			}
		}

		std::array<hardware_interface::FeedbackDevice, hardware_interface::SensorTerm_Last> internal_sensor_terms;
		std::array<ctre::phoenix::motorcontrol::FeedbackDevice, hardware_interface::SensorTerm_Last> victor_sensor_terms;
		if (tc.sensorTermsChanged(internal_sensor_terms) &&
			talon_convert_.feedbackDevice(internal_sensor_terms[0], victor_sensor_terms[0]) &&
			talon_convert_.feedbackDevice(internal_sensor_terms[1], victor_sensor_terms[1]) &&
			talon_convert_.feedbackDevice(internal_sensor_terms[2], victor_sensor_terms[2]) &&
			talon_convert_.feedbackDevice(internal_sensor_terms[3], victor_sensor_terms[3]))
		{
			// Check for errors on Talon writes. If it fails, used the reset() call to
			// set the changed var for the config items to true. This will trigger a re-try
			// the next time through the loop.
			bool rc = safeTalonConfigCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Sum0, victor_sensor_terms[0], configTimeoutMs),"ConfigSensorTerm Sum0", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Sum1, victor_sensor_terms[1], configTimeoutMs),"ConfigSensorTerm Sum1", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Diff0, victor_sensor_terms[2], configTimeoutMs),"ConfigSensorTerm Diff0", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Diff1, victor_sensor_terms[3], configTimeoutMs),"ConfigSensorTerm Diff1", ts.getCANID());
			}
			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " sensor terms");
				ts.setSensorTerms(internal_sensor_terms);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " sensor terms");
				tc.resetSensorTerms();
				break;
			}
		}

		// Get mode that is about to be commanded
		const hardware_interface::TalonMode talon_mode = tc.getMode();
		const int encoder_ticks_per_rotation = tc.getEncoderTicksPerRotation();
		ts.setEncoderTicksPerRotation(encoder_ticks_per_rotation);

		const double conversion_factor = tc.getConversionFactor();
		// No point doing changed() since it's quicker just to just copy a double
		// rather than query a bool and do it conditionally
		ts.setConversionFactor(conversion_factor);

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Velocity) * conversion_factor;
		const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, talon_mode) * conversion_factor;

		int slot;
		const bool slot_changed = tc.slotChanged(slot);

		double pval;
		double ival;
		double dval;
		double fval;
		int    iz;
		int    allowable_closed_loop_error;
		double max_integral_accumulator;
		double closed_loop_peak_output;
		int    closed_loop_period;

		if (tc.pidfChanged(pval, ival, dval, fval, iz, allowable_closed_loop_error, max_integral_accumulator, closed_loop_peak_output, closed_loop_period, slot))
		{
			bool rc = safeTalonConfigCall(victor->Config_kP(slot, pval, configTimeoutMs), "Config_kP", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->Config_kI(slot, ival, configTimeoutMs), "Config_kI", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->Config_kD(slot, dval, configTimeoutMs), "Config_kD", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->Config_kF(slot, fval, configTimeoutMs), "Config_kF", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->Config_IntegralZone(slot, iz, configTimeoutMs), "Config_IntegralZone", ts.getCANID());
			}
			// TODO : Scale these two?
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigAllowableClosedloopError(slot, allowable_closed_loop_error, configTimeoutMs), "ConfigAllowableClosedloopError", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigMaxIntegralAccumulator(slot, max_integral_accumulator, configTimeoutMs), "ConfigMaxIntegralAccumulator", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigClosedLoopPeakOutput(slot, closed_loop_peak_output, configTimeoutMs), "ConfigClosedLoopPeakOutput", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigClosedLoopPeriod(slot, closed_loop_period, configTimeoutMs), "ConfigClosedLoopPeriod", ts.getCANID());
			}

			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " PIDF slot " << slot << " config values");
				ts.setPidfP(pval, slot);
				ts.setPidfI(ival, slot);
				ts.setPidfD(dval, slot);
				ts.setPidfF(fval, slot);
				ts.setPidfIzone(iz, slot);
				ts.setAllowableClosedLoopError(allowable_closed_loop_error, slot);
				ts.setMaxIntegralAccumulator(max_integral_accumulator, slot);
				ts.setClosedLoopPeakOutput(closed_loop_peak_output, slot);
				ts.setClosedLoopPeriod(closed_loop_period, slot);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " PIDF slot " << slot << " config values");
				tc.resetPIDF(slot);
				break;
			}
		}

		bool aux_pid_polarity;
		if (tc.auxPidPolarityChanged(aux_pid_polarity))
		{
			if (safeTalonConfigCall(victor->ConfigAuxPIDPolarity(aux_pid_polarity, configTimeoutMs), "ConfigAuxPIDPolarity", ts.getCANID()))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<
						" AUX PIDF polarity to " << aux_pid_polarity << std::endl);
				ts.setAuxPidPolarity(aux_pid_polarity);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " AUX PIDF polarity");
				tc.resetAuxPidPolarity();
				break;
			}
		}

		if (slot_changed)
		{
			if (safeTalonConfigCall(victor->SelectProfileSlot(slot, pidIdx), "SelectProfileSlot", ts.getCANID()))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<
						" PIDF slot to " << slot << std::endl);
				ts.setSlot(slot);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " PIDF slot");
				tc.resetPidfSlot();
				break;
			}
		}

		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<
					" invert = " << invert << " phase = " << sensor_phase);
			// TODO : can these calls fail? If so, what to do if they do?
			victor->SetInverted(invert);
			victor->SetSensorPhase(sensor_phase);
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
		}

		hardware_interface::NeutralMode neutral_mode;
		ctre::phoenix::motorcontrol::NeutralMode ctre_neutral_mode;
		if (tc.neutralModeChanged(neutral_mode) &&
			talon_convert_.neutralMode(neutral_mode, ctre_neutral_mode))
		{

			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " neutral mode");
			// TODO : can this call fail? If so, what to do if they do?
			victor->SetNeutralMode(ctre_neutral_mode);
			ts.setNeutralMode(neutral_mode);
		}

		double iaccum;
		if (tc.integralAccumulatorChanged(iaccum))
		{
			//The units on this aren't really right?
			if (safeTalonConfigCall(victor->SetIntegralAccumulator(iaccum / closed_loop_scale, pidIdx, configTimeoutMs), "SetIntegralAccumulator", ts.getCANID()))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " integral accumulator");
				// Do not set talon state - this changes
				// dynamically so read it in read() above instead
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " integral accumulator");
				tc.resetIntegralAccumulator();
				break;
			}
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
			bool rc = safeTalonConfigCall(victor->ConfigOpenloopRamp(open_loop_ramp, configTimeoutMs),"ConfigOpenloopRamp", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigClosedloopRamp(closed_loop_ramp, configTimeoutMs),"ConfigClosedloopRamp", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigPeakOutputForward(peak_output_forward, configTimeoutMs),"ConfigPeakOutputForward", ts.getCANID());          // 100
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigPeakOutputReverse(peak_output_reverse, configTimeoutMs),"ConfigPeakOutputReverse", ts.getCANID());          // -100
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigNominalOutputForward(nominal_output_forward, configTimeoutMs),"ConfigNominalOutputForward", ts.getCANID()); // 0
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigNominalOutputReverse(nominal_output_reverse, configTimeoutMs),"ConfigNominalOutputReverse", ts.getCANID()); // 0
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigNeutralDeadband(neutral_deadband, configTimeoutMs),"ConfigNeutralDeadband", ts.getCANID());                 // 0
			}

			if (rc)
			{
				ts.setOpenloopRamp(open_loop_ramp);
				ts.setClosedloopRamp(closed_loop_ramp);
				ts.setPeakOutputForward(peak_output_forward);
				ts.setPeakOutputReverse(peak_output_reverse);
				ts.setNominalOutputForward(nominal_output_forward);
				ts.setNominalOutputReverse(nominal_output_reverse);
				ts.setNeutralDeadband(neutral_deadband);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " output shaping" <<
						" closed_loop_ramp = " << closed_loop_ramp <<
						" open_loop_ramp = " << open_loop_ramp <<
						" peak_output_forward = " << peak_output_forward <<
						" peak_output_reverse = " << peak_output_reverse <<
						" nominal_output_forward = " << nominal_output_forward <<
						" nominal_output_reverse = " << nominal_output_reverse <<
						" neutral_deadband = " << neutral_deadband);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " output shaping");
				tc.resetOutputShaping();
				break;
			}
		}

		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.voltageCompensationChanged(v_c_saturation,
			v_measurement_filter,
			v_c_enable))
		{
			bool rc = safeTalonConfigCall(victor->ConfigVoltageCompSaturation(v_c_saturation, configTimeoutMs),"ConfigVoltageCompSaturation", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigVoltageMeasurementFilter(v_measurement_filter, configTimeoutMs),"ConfigVoltageMeasurementFilter", ts.getCANID());
			}

			if (rc)
			{
				// Only enable once settings are correctly written to the Talon
				victor->EnableVoltageCompensation(v_c_enable);

				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " voltage compensation");

				ts.setVoltageCompensationSaturation(v_c_saturation);
				ts.setVoltageMeasurementFilter(v_measurement_filter);
				ts.setVoltageCompensationEnable(v_c_enable);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " voltage compensation");
				tc.resetVoltageCompensation();
				break;
			}
		}

		if (mc_enhanced)
		{
			hardware_interface::VelocityMeasurementPeriod internal_v_m_period;
			ctre::phoenix::motorcontrol::VelocityMeasPeriod phoenix_v_m_period;
			int v_m_window;

			if (tc.velocityMeasurementChanged(internal_v_m_period, v_m_window) &&
				talon_convert_.velocityMeasurementPeriod(internal_v_m_period, phoenix_v_m_period))
			{
				bool rc = safeTalonConfigCall(mc_enhanced->ConfigVelocityMeasurementPeriod(phoenix_v_m_period, configTimeoutMs),"ConfigVelocityMeasurementPeriod", ts.getCANID());
				if (rc)
				{
					rc = safeTalonConfigCall(mc_enhanced->ConfigVelocityMeasurementWindow(v_m_window, configTimeoutMs),"ConfigVelocityMeasurementWindow", ts.getCANID());
				}

				if (rc)
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " velocity measurement period / window");
					ts.setVelocityMeasurementPeriod(internal_v_m_period);
					ts.setVelocityMeasurementWindow(v_m_window);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " velocity measurement period / window");
					tc.resetVelocityMeasurement();
					break;
				}
			}
		}

		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			if (safeTalonConfigCall(victor->SetSelectedSensorPosition(sensor_position / radians_scale, pidIdx, configTimeoutMs),
						"SetSelectedSensorPosition", ts.getCANID()))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " selected sensor position to " << sensor_position / radians_scale);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " selected sensor position to " << sensor_position / radians_scale);
				tc.resetSensorPosition();
				break;
			}
		}

		if (mc_enhanced)
		{
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
				talon_convert_.limitSwitchSource(internal_local_forward_source, talon_local_forward_source) &&
				talon_convert_.limitSwitchNormal(internal_local_forward_normal, talon_local_forward_normal) &&
				talon_convert_.limitSwitchSource(internal_local_reverse_source, talon_local_reverse_source) &&
				talon_convert_.limitSwitchNormal(internal_local_reverse_normal, talon_local_reverse_normal) )
			{
				bool rc = safeTalonConfigCall(mc_enhanced->ConfigForwardLimitSwitchSource(talon_local_forward_source, talon_local_forward_normal, configTimeoutMs),"ConfigForwardLimitSwitchSource", ts.getCANID());
				if (rc)
				{
					rc = safeTalonConfigCall(mc_enhanced->ConfigReverseLimitSwitchSource(talon_local_reverse_source, talon_local_reverse_normal, configTimeoutMs),"ConfigReverseLimitSwitchSource", ts.getCANID());
				}

				if (rc)
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
							<< " limit switches "
							<< talon_local_forward_source << " " << talon_local_forward_normal << " "
							<< talon_local_reverse_source << " " << talon_local_reverse_normal);
					ts.setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
					ts.setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " limit switches");
					tc.resetLimitSwitchesSource();
					break;
				}
			}
		}

		hardware_interface::RemoteLimitSwitchSource internal_remote_forward_source;
		hardware_interface::LimitSwitchNormal internal_remote_forward_normal;
		unsigned int remote_forward_id;
		hardware_interface::RemoteLimitSwitchSource internal_remote_reverse_source;
		hardware_interface::LimitSwitchNormal internal_remote_reverse_normal;
		unsigned int remote_reverse_id;
		ctre::phoenix::motorcontrol::RemoteLimitSwitchSource talon_remote_forward_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_remote_forward_normal;
		ctre::phoenix::motorcontrol::RemoteLimitSwitchSource talon_remote_reverse_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_remote_reverse_normal;
		if (tc.remoteLimitSwitchesSourceChanged(internal_remote_forward_source, internal_remote_forward_normal, remote_forward_id,
											    internal_remote_reverse_source, internal_remote_reverse_normal, remote_reverse_id) &&
			talon_convert_.remoteLimitSwitchSource(internal_remote_forward_source, talon_remote_forward_source) &&
			talon_convert_.limitSwitchNormal(internal_remote_forward_normal, talon_remote_forward_normal) &&
			talon_convert_.remoteLimitSwitchSource(internal_remote_reverse_source, talon_remote_reverse_source) &&
			talon_convert_.limitSwitchNormal(internal_remote_reverse_normal, talon_remote_reverse_normal) )
		{
			bool rc = safeTalonConfigCall(victor->ConfigForwardLimitSwitchSource(talon_remote_forward_source, talon_remote_forward_normal, remote_forward_id, configTimeoutMs),"ConfigForwardLimitSwitchSource(Remote)", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigReverseLimitSwitchSource(talon_remote_reverse_source, talon_remote_reverse_normal, remote_reverse_id, configTimeoutMs),"ConfigReverseLimitSwitchSource(Remote)", ts.getCANID());
			}

			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
						<< " remote limit switches "
						<< talon_remote_forward_source << " " << talon_remote_forward_normal << " " << remote_forward_id << " "
						<< talon_remote_reverse_source << " " << talon_remote_reverse_normal << " " << remote_reverse_id);
				ts.setRemoteForwardLimitSwitchSource(internal_remote_forward_source, internal_remote_forward_normal, remote_forward_id);
				ts.setRemoteReverseLimitSwitchSource(internal_remote_reverse_source, internal_remote_reverse_normal, remote_reverse_id);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " remote limit switches");
				tc.resetRemoteLimitSwitchesSource();
				break;
			}
		}

		bool clear_position_on_limit_f;
		if (tc.clearPositionOnLimitFChanged(clear_position_on_limit_f))
		{
			if (safeTalonConfigCall(victor->ConfigClearPositionOnLimitF(clear_position_on_limit_f, configTimeoutMs), "ConfigClearPositionOnLimitF", ts.getCANID()))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
						<< " clear position on limit F = " << clear_position_on_limit_f);
				ts.setClearPositionOnLimitF(clear_position_on_limit_f);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
						<< " clear position on limit F");
				tc.resetClearPositionOnLimitF();
				break;
			}

		}

		bool clear_position_on_limit_r;
		if (tc.clearPositionOnLimitRChanged(clear_position_on_limit_r))
		{
			if (safeTalonConfigCall(victor->ConfigClearPositionOnLimitR(clear_position_on_limit_f, configTimeoutMs), "ConfigClearPositionOnLimitR", ts.getCANID()))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
						<< " clear position on limit R = " << clear_position_on_limit_r);
				ts.setClearPositionOnLimitR(clear_position_on_limit_r);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
						<< " clear position on limit R");
				tc.resetClearPositionOnLimitR();
				break;
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
			const double softlimit_forward_threshold_NU = softlimit_forward_threshold / radians_scale; //native units
			const double softlimit_reverse_threshold_NU = softlimit_reverse_threshold / radians_scale;
			bool rc = safeTalonConfigCall(victor->ConfigForwardSoftLimitThreshold(softlimit_forward_threshold_NU, configTimeoutMs),"ConfigForwardSoftLimitThreshold", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigForwardSoftLimitEnable(softlimit_forward_enable, configTimeoutMs),"ConfigForwardSoftLimitEnable", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigReverseSoftLimitThreshold(softlimit_reverse_threshold_NU, configTimeoutMs),"ConfigReverseSoftLimitThreshold", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigReverseSoftLimitEnable(softlimit_reverse_enable, configTimeoutMs),"ConfigReverseSoftLimitEnable", ts.getCANID());
			}

			if (rc)
			{
				// Only set override enable if all config calls succeed
				victor->OverrideSoftLimitsEnable(softlimit_override_enable);
				ts.setOverrideSoftLimitsEnable(softlimit_override_enable);
				ts.setForwardSoftLimitThreshold(softlimit_forward_threshold);
				ts.setForwardSoftLimitEnable(softlimit_forward_enable);
				ts.setReverseSoftLimitThreshold(softlimit_reverse_threshold);
				ts.setReverseSoftLimitEnable(softlimit_reverse_enable);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " soft limits " <<
						std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
						std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
						std::endl << "\toverride_enable=" << softlimit_override_enable);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " soft limits");
				tc.resetSoftLimit();
				break;
			}
		}

		if (talon)
		{
			int peak_amps;
			int peak_msec;
			int continuous_amps;
			bool enable;
			if (tc.currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
			{
				bool rc = safeTalonConfigCall(talon->ConfigPeakCurrentLimit(peak_amps, configTimeoutMs),"ConfigPeakCurrentLimit", ts.getCANID());
				if (rc)
				{
					rc = safeTalonConfigCall(talon->ConfigPeakCurrentDuration(peak_msec, configTimeoutMs),"ConfigPeakCurrentDuration", ts.getCANID());
				}
				if (rc)
				{
					rc = safeTalonConfigCall(talon->ConfigContinuousCurrentLimit(continuous_amps, configTimeoutMs),"ConfigContinuousCurrentLimit", ts.getCANID());
				}
				if (rc)
				{
					// Only enable current limit if all config calls succeed
					talon->EnableCurrentLimit(enable);
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " peak current");
					ts.setPeakCurrentLimit(peak_amps);
					ts.setPeakCurrentDuration(peak_msec);
					ts.setContinuousCurrentLimit(continuous_amps);
					ts.setCurrentLimitEnable(enable);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " peak current");
					tc.resetCurrentLimit();
					break;
				}
			}
		}

		if (falcon)
		{
			double limit;
			double trigger_threshold_current;
			double trigger_threshold_time;
			bool   limit_enable;
			if (tc.supplyCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
			{
				if (safeTalonConfigCall(falcon->ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(limit_enable, limit, trigger_threshold_current, trigger_threshold_time), configTimeoutMs), "ConfigSupplyCurrentLimit", ts.getCANID()))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " supply current limit");
					ts.setSupplyCurrentLimit(limit);
					ts.setSupplyCurrentLimitEnable(limit_enable);
					ts.setSupplyCurrentTriggerThresholdCurrent(trigger_threshold_current);
					ts.setSupplyCurrentTriggerThresholdTime(trigger_threshold_time);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " supply current limit");
					tc.resetSupplyCurrentLimit();
					break;
				}
			}
			if (tc.statorCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
			{
				if (safeTalonConfigCall(falcon->ConfigStatorCurrentLimit(ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration(limit_enable, limit, trigger_threshold_current, trigger_threshold_time), configTimeoutMs), "ConfigStatorCurrentLimit", ts.getCANID()))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " stator current limit");
					ts.setStatorCurrentLimit(limit);
					ts.setStatorCurrentLimitEnable(limit_enable);
					ts.setStatorCurrentTriggerThresholdCurrent(trigger_threshold_current);
					ts.setStatorCurrentTriggerThresholdTime(trigger_threshold_time);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " stator current limit");
					tc.resetStatorCurrentLimit();
					break;
				}
			}

			hardware_interface::MotorCommutation motor_commutation;
			ctre::phoenix::motorcontrol::MotorCommutation motor_commutation_ctre;
			if (tc.motorCommutationChanged(motor_commutation) &&
				talon_convert_.motorCommutation(motor_commutation, motor_commutation_ctre))
			{
				if (safeTalonConfigCall(falcon->ConfigMotorCommutation(motor_commutation_ctre, configTimeoutMs), "ConfigMotorCommutation", ts.getCANID()))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motor commutation");
					ts.setMotorCommutation(motor_commutation);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motor commutation");
					tc.resetMotorCommutation();
					break;
				}
			}

			hardware_interface::AbsoluteSensorRange absolute_sensor_range;
			ctre::phoenix::sensors::AbsoluteSensorRange absolute_sensor_range_ctre;
			if (tc.absoluteSensorRangeChanged(absolute_sensor_range) &&
				talon_convert_.absoluteSensorRange(absolute_sensor_range, absolute_sensor_range_ctre))
			{
				if (safeTalonConfigCall(falcon->ConfigIntegratedSensorAbsoluteRange(absolute_sensor_range_ctre, configTimeoutMs), "ConfigIntegratedSensorAbsoluteRange", ts.getCANID()))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
					ts.setAbsoluteSensorRange(absolute_sensor_range);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
					tc.resetAbsoluteSensorRange();
					break;
				}
			}

			hardware_interface::SensorInitializationStrategy sensor_initialization_strategy;
			ctre::phoenix::sensors::SensorInitializationStrategy sensor_initialization_strategy_ctre;
			if (tc.sensorInitializationStrategyChanged(sensor_initialization_strategy) &&
				talon_convert_.sensorInitializationStrategy(sensor_initialization_strategy, sensor_initialization_strategy_ctre))
			{
				if (safeTalonConfigCall(falcon->ConfigIntegratedSensorInitializationStrategy(sensor_initialization_strategy_ctre, configTimeoutMs), "ConfigIntegratedSensorInitializationStrategy", ts.getCANID()))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
					ts.setSensorInitializationStrategy(sensor_initialization_strategy);
				}
				else
				{
					ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
					tc.resetSensorInitializationStrategy();
					break;
				}
			}
		}

		if (mc_enhanced)
		{
			bool exit_loop = false;
			// TODO : fix for Victor non-enhanced status frames
			for (int i = hardware_interface::Status_1_General; !exit_loop && (i < hardware_interface::Status_Last); i++)
			{
				uint8_t status_frame_period;
				const auto status_frame = static_cast<hardware_interface::StatusFrame>(i);
				if (tc.statusFramePeriodChanged(status_frame, status_frame_period) && (status_frame_period != 0))
				{
					ctre::phoenix::motorcontrol::StatusFrameEnhanced status_frame_enhanced;
					if (talon_convert_.statusFrame(status_frame, status_frame_enhanced))
					{
						if (safeTalonConfigCall(mc_enhanced->SetStatusFramePeriod(status_frame_enhanced, status_frame_period), "SetStatusFramePeriod", ts.getCANID()))
						{
							ts.setStatusFramePeriod(status_frame, status_frame_period);
							ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " status_frame " << i << "=" << static_cast<int>(status_frame_period) << "mSec");
						}
						else
						{
							ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " status_frame " << i);
							tc.resetStatusFramePeriod(status_frame);
							exit_loop = true;
						}
					}
				}
			}
			if (exit_loop)
			{
				break;
			}
		}

		bool exit_loop = false;
		for (int i = hardware_interface::Control_3_General; !exit_loop && (i < hardware_interface::Control_Last); i++)
		{
			uint8_t control_frame_period;
			const auto control_frame = static_cast<hardware_interface::ControlFrame>(i);
			if (tc.controlFramePeriodChanged(control_frame, control_frame_period) && (control_frame_period != 0))
			{
				ctre::phoenix::motorcontrol::ControlFrame control_frame_phoenix;
				if (talon_convert_.controlFrame(control_frame, control_frame_phoenix))
				{
					if (safeTalonConfigCall(victor->SetControlFramePeriod(control_frame_phoenix, control_frame_period), "SetControlFramePeriod", ts.getCANID()))
					{
						ts.setControlFramePeriod(control_frame, control_frame_period);
						ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " control_frame " << i << "=" << static_cast<int>(control_frame_period) << "mSec");
					}
					else
					{
						ROS_INFO_STREAM("Faile to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " control_frame " << i);
						tc.resetControlFramePeriod(control_frame);
						exit_loop = true;
					}
				}
			}
		}
		if (exit_loop)
		{
			break;
		}

		double motion_cruise_velocity;
		double motion_acceleration;
		int motion_s_curve_strength;
		if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration, motion_s_curve_strength))
		{
			//converted from rad/sec to native units
			bool rc = safeTalonConfigCall(victor->ConfigMotionCruiseVelocity(motion_cruise_velocity / radians_per_second_scale, configTimeoutMs),"ConfigMotionCruiseVelocity(", ts.getCANID());
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigMotionAcceleration(motion_acceleration / radians_per_second_scale, configTimeoutMs),"ConfigMotionAcceleration(", ts.getCANID());
			}
			if (rc)
			{
				rc = safeTalonConfigCall(victor->ConfigMotionSCurveStrength(motion_s_curve_strength, configTimeoutMs), "ConfigMotionSCurveStrength", ts.getCANID());
			}

			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " cruise velocity / acceleration");
				ts.setMotionCruiseVelocity(motion_cruise_velocity);
				ts.setMotionAcceleration(motion_acceleration);
				ts.setMotionSCurveStrength(motion_s_curve_strength);
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " cruise velocity / acceleration");
				tc.resetMotionCruise();
				break;
			}
		}
		int motion_profile_trajectory_period;
		if (tc.motionProfileTrajectoryPeriodChanged(motion_profile_trajectory_period))
		{
			if (safeTalonConfigCall(victor->ConfigMotionProfileTrajectoryPeriod(motion_profile_trajectory_period, configTimeoutMs),"ConfigMotionProfileTrajectoryPeriod", ts.getCANID()))
			{
				ts.setMotionProfileTrajectoryPeriod(motion_profile_trajectory_period);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile trajectory period");
			}
			else
			{
				ROS_INFO_STREAM("Failed to update joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile trajectory period");
				tc.resetMotionProfileTrajectoryPeriod();
				break;
			}
		}

		if (tc.clearMotionProfileTrajectoriesChanged())
		{
			if (safeTalonConfigCall(victor->ClearMotionProfileTrajectories(), "ClearMotionProfileTrajectories", ts.getCANID()))
			{
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile trajectories");
			}
			else
			{
				ROS_INFO_STREAM("Failed to clear joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile trajectories");
				tc.setClearMotionProfileTrajectories();
				break;
			}
		}

		if (tc.clearMotionProfileHasUnderrunChanged())
		{
			if (safeTalonConfigCall(victor->ClearMotionProfileHasUnderrun(configTimeoutMs),"ClearMotionProfileHasUnderrun", ts.getCANID()))
			{
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile underrun changed");
			}
			else
			{
				ROS_INFO_STREAM("Failed to Clear joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile underrun changed");
				tc.setClearMotionProfileHasUnderrun();
				break;
			}
		}

		// TODO : check that Talon motion buffer is not full
		// before writing, communicate how many have been written
		// - and thus should be cleared - from the talon_command
		// list of requests.

		// TODO : rewrite this using BufferedTrajectoryPointStream
		std::vector<hardware_interface::TrajectoryPoint> trajectory_points;
		if (tc.motionProfileTrajectoriesChanged(trajectory_points))
		{
			//int i = 0;
			for (auto it = trajectory_points.cbegin(); it != trajectory_points.cend(); ++it)
			{
				ctre::phoenix::motion::TrajectoryPoint pt;
				pt.position = it->position / radians_scale;
				pt.velocity = it->velocity / radians_per_second_scale;
				pt.headingDeg = it->headingRad * 180. / M_PI;
				pt.arbFeedFwd = it->arbFeedFwd;
				pt.auxiliaryPos = it->auxiliaryPos; // TODO : unit conversion?
				pt.auxiliaryVel = it->auxiliaryVel; // TODO : unit conversion?
				pt.auxiliaryArbFeedFwd = it->auxiliaryArbFeedFwd; // TODO : unit conversion?
				pt.profileSlotSelect0 = it->profileSlotSelect0;
				pt.profileSlotSelect1 = it->profileSlotSelect1;
				pt.isLastPoint = it->isLastPoint;
				pt.zeroPos = it->zeroPos;
				pt.timeDur = it->timeDur;
				pt.useAuxPID = it->useAuxPID;
				//ROS_INFO_STREAM("id: " << joint_id << " pos: " << pt.position << " i: " << i++);
			}
			ROS_INFO_STREAM("Added joint " << joint_id << "=" <<
					can_ctre_mc_names_[joint_id] << " motion profile trajectories");
		}

		// Set new motor setpoint if either the mode or the setpoint has been changed
		if (robot_enabled)
		{
			double command;
			hardware_interface::TalonMode in_mode;
			hardware_interface::DemandType demand1_type_internal;
			double demand1_value;

			const bool b1 = tc.modeChanged(in_mode);
			const bool b2 = tc.commandChanged(command);
			const bool b3 = tc.demand1Changed(demand1_type_internal, demand1_value);

			// ROS_INFO_STREAM("b1 = " << b1 << " b2 = " << b2 << " b3 = " << b3);
			if (b1 || b2 || b3)
			{
				ctre::phoenix::motorcontrol::ControlMode out_mode;
				ctre::phoenix::motorcontrol::DemandType demand1_type_phoenix;
				if (talon_convert_.controlMode(in_mode, out_mode) &&
					talon_convert_.demand1Type(demand1_type_internal, demand1_type_phoenix))
				{
					ts.setSetpoint(command); // set the state before converting it to native units
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
						default:
							break;
					}

#ifdef DEBUG_WRITE
					ROS_INFO_STREAM("called Set(4) on " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<
							" out_mode = " << static_cast<int>(out_mode) << " command = " << command <<
							" demand1_type_phoenix = " << static_cast<int>(demand1_type_phoenix) <<
							" demand1_value = " << demand1_value);
#endif
					ts.setTalonMode(in_mode);
					ts.setDemand1Type(demand1_type_internal);
					ts.setDemand1Value(demand1_value);

					victor->Set(out_mode, command, demand1_type_phoenix, demand1_value);
				}
				else
				{
					ROS_ERROR_STREAM("Couldn't convert to the talon enum type");
				}
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
			if (last_robot_enabled_)
			{
				// On the switch from robot enabled to robot disabled, set Talons to ControlMode::Disabled
				// call resetMode() to queue up a change back to the correct mode / setpoint
				// when the robot switches from disabled back to enabled
				tc.resetMode();    // also forces re-write of setpoint
				tc.resetDemand1(); // make sure demand1 type/value is also written on re-enable
				victor->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
				ts.setTalonMode(hardware_interface::TalonMode_Disabled);
				ROS_INFO_STREAM("Robot disabled - called Set(Disabled) on " << joint_id << "=" << can_ctre_mc_names_[joint_id]);
			}
		}

		if (tc.clearStickyFaultsChanged())
		{
			if (safeTalonConfigCall(victor->ClearStickyFaults(configTimeoutMs), "ClearStickyFaults", ts.getCANID()))
			{
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " sticky_faults");
			}
			else
			{
				ROS_INFO_STREAM("Failed to clear joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " sticky_faults");
				tc.setClearStickyFaults();
				break;
			}
		}
	}
	last_robot_enabled_ = robot_enabled;

	for (size_t joint_id = 0; joint_id < num_cancoders_; ++joint_id)
	{
		if (!cancoder_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &cancoder = cancoders_[joint_id];
		auto &cs = cancoder_state_[joint_id];
		auto &cc = cancoder_command_[joint_id];
		if (cancoder->HasResetOccurred())
		{
			cc.resetPosition();
			cc.resetVelocityMeasPeriod();
			cc.resetVelocityMeasWindow();
			cc.resetAbsoluteSensorRange();
			cc.resetMagnetOffset();
			cc.resetInitializationStrategy();
			cc.resetFeedbackCoefficient();
			cc.resetDirection();
			cc.resetSensorDataStatusFramePeriod();
			cc.resetVBatAndFaultsStatusFramePeriod();
		}
		cs.setConversionFactor(cc.getConversionFactor());
		double position;
		if (cc.positionChanged(position))
		{
			if (safeTalonCall(cancoder->SetPosition(position / cs.getConversionFactor()), "cancoder->SetPosition", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set position to " << position);
				// Don't set state - it will be updated in next read() loop
			}
			else
			{
				cc.resetPosition();
			}
		}
		if (cc.positionToAbsoluteChanged())
		{
			if (safeTalonCall(cancoder->SetPositionToAbsolute(), "cancoder->SetPositionToAbsolute", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set position to absolute");
				// Don't set state - it will be updated in next read() loop
			}
			else
			{
				cc.setPositionToAbsolute();
			}
		}
		hardware_interface::cancoder::SensorVelocityMeasPeriod velocity_meas_period;
		ctre::phoenix::sensors::SensorVelocityMeasPeriod ctre_velocity_meas_period;
		if (cc.velocityMeasPeriodChanged(velocity_meas_period) &&
			cancoder_convert_.velocityMeasPeriod(velocity_meas_period, ctre_velocity_meas_period))
		{
			if (safeTalonCall(cancoder->ConfigVelocityMeasurementPeriod(ctre_velocity_meas_period), "cancoder->ConfigVelocityMeasurementPeriod", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set velocity measurement period to " << static_cast<int>(ctre_velocity_meas_period));
				cs.setVelocityMeasPeriod(velocity_meas_period);
			}
			else
			{
				cc.resetVelocityMeasPeriod();
			}
		}

		int velocity_meas_window;
		if (cc.velocityMeasWindowChanged(velocity_meas_window))
		{
			if (safeTalonCall(cancoder->ConfigVelocityMeasurementWindow(velocity_meas_window), "cancoder->ConfigVelocityMeasurementWindow", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set velocity measurement window to " << velocity_meas_window);
				cs.setVelocityMeasWindow(velocity_meas_window);
			}
			else
			{
				cc.resetVelocityMeasWindow();
			}
		}
		hardware_interface::cancoder::AbsoluteSensorRange absolute_sensor_range;
		ctre::phoenix::sensors::AbsoluteSensorRange ctre_absolute_sensor_range;
		if (cc.absoluteSensorRangeChanged(absolute_sensor_range) &&
			cancoder_convert_.absoluteSensorRange(absolute_sensor_range, ctre_absolute_sensor_range))
		{
			if (safeTalonCall(cancoder->ConfigAbsoluteSensorRange(ctre_absolute_sensor_range), "cancoder->ConfigAbsoluteSensorRange", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set absolute sensor range to " << absolute_sensor_range);
				cs.setAbsoluteSensorRange(absolute_sensor_range);
			}
			else
			{
				cc.resetAbsoluteSensorRange();
			}
		}

		double magnet_offset;
		if (cc.magnetOffsetChanged(magnet_offset))
		{
			if (safeTalonCall(cancoder->ConfigMagnetOffset(magnet_offset), "cancoder->ConfigMagnetOffset", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set magnet offset to " << magnet_offset);
				cs.setMagnetOffset(magnet_offset);
			}
			else
			{
				cc.resetMagnetOffset();
			}
		}

		hardware_interface::cancoder::SensorInitializationStrategy initialization_strategy;
		ctre::phoenix::sensors::SensorInitializationStrategy ctre_initialization_strategy;
		if (cc.InitializationStrategyChanged(initialization_strategy) &&
			cancoder_convert_.initializationStrategy(initialization_strategy, ctre_initialization_strategy))
		{
			if (safeTalonCall(cancoder->ConfigSensorInitializationStrategy(ctre_initialization_strategy), "cancoder->ConfigSensorInitializationStrategy", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set sensor intitialization strategy to " << initialization_strategy);
				cs.setInitializationStrategy(initialization_strategy);
			}
			else
			{
				cc.resetInitializationStrategy();
			}
		}
		double feedback_coefficient;
		std::string unit_string;
		hardware_interface::cancoder::SensorTimeBase time_base;
		ctre::phoenix::sensors::SensorTimeBase ctre_time_base;
		if (cc.feedbackCoefficientChanged(feedback_coefficient, unit_string, time_base) &&
			cancoder_convert_.timeBase(time_base, ctre_time_base))
		{
			if (safeTalonCall(cancoder->ConfigFeedbackCoefficient(feedback_coefficient, unit_string, ctre_time_base), "cancoder->ConfigFeedbackCoefficient", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set feedback coefficient to  " << feedback_coefficient << " " << unit_string << " " << time_base);
				cs.setFeedbackCoefficient(feedback_coefficient);
				cs.setUnitString(unit_string);
				cs.setTimeBase(time_base);
			}
			else
			{
				cc.resetFeedbackCoefficient();
			}
		}

		bool direction;
		if (cc.directionChanged(direction))
		{
			if (safeTalonCall(cancoder->ConfigSensorDirection(direction), "cancoder->ConfigSensorDirection", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set direction to " << direction);
				cs.setDirection(direction);
			}
			else
			{
				cc.resetDirection();
			}
		}

		int sensor_data_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(sensor_data_status_frame_period))
		{
			if (safeTalonCall(cancoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, sensor_data_status_frame_period), "cancoder->SetStatusFramePeriod(SensorData)", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set sensor data status frame period to " << sensor_data_status_frame_period);
				cs.setSensorDataStatusFramePeriod(sensor_data_status_frame_period);
			}
			else
			{
				cc.resetSensorDataStatusFramePeriod();
			}
		}

		int vbat_and_faults_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(vbat_and_faults_status_frame_period))
		{
			if (safeTalonCall(cancoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, vbat_and_faults_status_frame_period), "cancoder->SetStatusFramePeriod(VbatAndFaults)", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set vbat and fault status frame period to " << vbat_and_faults_status_frame_period);
				cs.setVbatAndFaultsStatusFramePeriod(vbat_and_faults_status_frame_period);
			}
			else
			{
				cc.resetVBatAndFaultsStatusFramePeriod();
			}
		}

		if (cc.clearStickyFaultsChanged())
		{
			if (safeTalonCall(cancoder->ClearStickyFaults(), "cancoder->ClearStickyFaults", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id] << " : Sticky faults cleared");
			}
			else
			{
				cc.setClearStickyFaults();
			}
		}
	}

	this->write_tracer_.start_unique("candle");
	for (size_t candle_id = 0; candle_id < this->num_candles_; candle_id++) {
		if (!(this->candle_local_hardwares_[candle_id]))
			continue;
		
		// Get candle, state, and command interfaces
		auto &candle = this->candles_[candle_id];
		auto &candle_state = this->candle_state_[candle_id];
		auto &candle_command = this->candle_command_[candle_id];

		// For each Command property, check if it changed
		//  if it did change, update the actual CANdle, and update the state

		// Brightness
		double brightness;
		if (candle_command.brightnessChanged(brightness)) {
			if (safeTalonCall(
				candle->ConfigBrightnessScalar(brightness),
				"candle->ConfigBrightnessScalar",
				candle_state.getDeviceID()
			)) {
				ROS_INFO_STREAM("CANdle " << this->candle_names_[candle_id]
						<< " : Set brightness to " << brightness);
				candle_state.setBrightness(brightness);
			} else {
				candle_command.resetBrightnessChanged();
			}
		}

		// Show status LED when active
		bool status_led_when_active;
		if (candle_command.statusLEDWhenActiveChanged(status_led_when_active)) {
			if (safeTalonCall(
				candle->ConfigStatusLedState(!status_led_when_active),
				"candle->ConfigStatusLedState",
				candle_state.getDeviceID()
			)) {
				ROS_INFO_STREAM("CANdle " << this->candle_names_[candle_id]
						<< " : Set status led when active to " << status_led_when_active);
				candle_state.setStatusLEDWhenActive(status_led_when_active);
			} else {
				candle_command.resetStatusLEDWhenActiveChanged();
			}
		}

		// Enabled
		bool enabled;
		if (candle_command.enabledChanged(enabled)) {
			if (safeTalonCall(
				candle->configV5Enabled(enabled),
				"candle->configV5Enabled",
				candle_state.getDeviceID()
			)) {
				ROS_INFO_STREAM("CANdle " << this->candle_names_[candle_id]
						<< " : Set enabled to " << enabled);
				candle_state.setEnabled(enabled);
			} else {
				candle_command.resetEnabledChanged();
			}
		}

		// Stop animations
		bool stop_animations;
		if (candle_command.stopAnimationsChanged(stop_animations)) {
			size_t max_animations = (size_t)candle->GetMaxSimultaneousAnimationCount();
			for (size_t i = 0; i < max_animations; i++) {
				candle->ClearAnimation(i);
			}

			candle_command.drainAnimations();
			candle_state.setMaxAnimations(max_animations);
			candle_state.clearAnimations();
		}

		// Animation
		std::vector<hardware_interface::candle::Animation> candle_animations;
		if (candle_command.animationsChanged(candle_animations)) {
			for (hardware_interface::candle::Animation& candle_animation : candle_animations) {
				// Clear any old animations
				size_t group_max = (size_t)(candle_animation.start + candle_animation.count);
				for (size_t led_id = candle_animation.start; led_id < group_max; led_id++) {
					std::optional<hardware_interface::candle::LED> led = candle_state.getLED(led_id);
					if (led.has_value() && led->type == hardware_interface::candle::LEDType::Animated) {
						int animation_id = led->getAnimationID().value();
						candle->ClearAnimation(animation_id);

						hardware_interface::candle::Animation existing_animation = candle_state.getAnimation(animation_id).value();
						candle->SetLEDs(
							0,
							0,
							0,
							0,
							existing_animation.start,
							existing_animation.count
						);

						candle_state.clearAnimation(animation_id);
					}
				}

				// A pointer to the animation, after it gets converted
				std::shared_ptr<ctre::phoenix::led::Animation> animation;

				// Convert from Animation to the appropriate CTRE animation class
				if (candle_animation.class_type == hardware_interface::candle::AnimationClass::BaseStandard) {
					animation = candle_convert::convertBaseStandardAnimation(candle_animation);
				} else {
					animation = candle_convert::convertBaseTwoAnimation(candle_animation);
				}

				size_t slot = candle_state.getNextAnimationSlot();

				if (safeTalonCall(
					candle->Animate(*animation, slot),
					"candle->Animate",
					candle_state.getDeviceID()
				)) {
					ROS_INFO_STREAM("CANdle " << this->candle_names_[candle_id] << " : Changed its animation");
					candle_state.setAnimation(candle_animation);
				} else {
					// If we fail to animate, just re-add the animation to the queue
					candle_command.setAnimation(candle_animation);
				}
			}
			candle_command.drainAnimations();
		}

		// LEDs
		std::vector<hardware_interface::candle::LEDGroup> led_groups;
		if (candle_command.ledGroupsChanged(led_groups)) {
			candle_command.drainLEDGroups();
			for (hardware_interface::candle::LEDGroup group : led_groups) {
				size_t group_max = (size_t)(group.start + group.count);

				for (size_t led_id = group.start; led_id < group_max; led_id++) {
					std::optional<hardware_interface::candle::LED> led = candle_state.getLED(led_id);
					if (led.has_value() && led->type == hardware_interface::candle::LEDType::Animated) {
						int animation_id = led->getAnimationID().value();
						candle->ClearAnimation(animation_id);

						hardware_interface::candle::Animation animation = candle_state.getAnimation(animation_id).value();
						candle->SetLEDs(
							0,
							0,
							0,
							0,
							animation.start,
							animation.count
						);

						candle_state.clearAnimation(animation_id);
					}
				}

				if (safeTalonCall(
					candle->SetLEDs(
						group.colour.red,
						group.colour.green,
						group.colour.blue,
						group.colour.white,
						group.start,
						group.count
					),
					"candle->SetLEDs",
					candle_state.getDeviceID()
				)) {
					for (size_t led_id = group.start; led_id < group_max; led_id++) {
						candle_state.setLED(led_id, group.colour);
					}

					ROS_INFO_STREAM("CANdle " << this->candle_names_[candle_id]
							<< " : Changed colours");
				} else {
					candle_command.setLEDGroup(group);
				}
			}
		}
	}

	write_tracer_.start_unique("pigeon2");
	for (size_t joint_id = 0; joint_id < num_pigeon2s_; ++joint_id)
	{
		if (!pigeon2_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &pigeon2 = pigeon2s_[joint_id];
		auto &ps = pigeon2_state_[joint_id];
		auto &pc = pigeon2_command_[joint_id];
		if (pigeon2->HasResetOccurred())
		{
			pc.resetMountPoseAxis();
			pc.resetMountPoseRPY();
			pc.resetXAxisGyroError();
			pc.resetYAxisGyroError();
			pc.resetZAxisGyroError();
			pc.resetCompassEnable();
			pc.resetDisableTemperatureCompensation();
			pc.resetDisableNoMotionCalibration();
			pc.resetSetYaw();
			pc.resetAddYaw();
		}
		hardware_interface::pigeon2::AxisDirection forward;
		hardware_interface::pigeon2::AxisDirection up;
		ctre::phoenix::sensors::AxisDirection forward_phoenix;
		ctre::phoenix::sensors::AxisDirection up_phoenix;
		if (pc.mountPoseAxisChanged(forward, up) &&
			(forward != hardware_interface::pigeon2::AxisDirection::Undefined) &&
		    (up != hardware_interface::pigeon2::AxisDirection::Undefined) &&
			pigeon2_convert_.axisDirection(forward, forward_phoenix) &&
			pigeon2_convert_.axisDirection(up, up_phoenix) )
		{
			if (safeTalonCall(pigeon2->ConfigMountPose(forward_phoenix, up_phoenix, configTimeoutMs), "pigeon2->ConfigMountPose(axis)", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " : ConfigMountPose(axis) set");
				ps.setMountPoseForward(forward);
				ps.setMountPoseUp(up);
				ps.setMountPoseRoll(0);
				ps.setMountPosePitch(0);
				ps.setMountPoseYaw(0);
			}
			else
			{
				pc.resetMountPoseAxis();
			}
		}

		double roll;
		double pitch;
		double yaw;
		if (pc.mountPoseRPYChanged(roll, pitch, yaw))
		{
			if (safeTalonCall(pigeon2->ConfigMountPose(roll, pitch, yaw, configTimeoutMs), "pigeon2->ConfigMountPose(rpy)", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " : ConfigMountPose(rpy) set");
				ps.setMountPoseForward(hardware_interface::pigeon2::AxisDirection::Undefined);
				ps.setMountPoseUp(hardware_interface::pigeon2::AxisDirection::Undefined);
				ps.setMountPoseRoll(angles::to_degrees(roll));
				ps.setMountPosePitch(angles::to_degrees(pitch));
				ps.setMountPoseYaw(angles::to_degrees(yaw));
			}
			else
			{
				pc.resetMountPoseRPY();
			}
		}

		double x_axis_gyro_error;
		if (pc.xAxisGyroErrorChanged(x_axis_gyro_error))
		{
			if (safeTalonCall(pigeon2->ConfigXAxisGyroError(x_axis_gyro_error), "pigeon2->ConfigXAxisGyroError()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pogeon2 " << pigeon2_names_[joint_id] << " ConfigXAxisGyroError = " << x_axis_gyro_error);
				ps.setXAxisGyroError(angles::to_degrees(x_axis_gyro_error));
			}
			else
			{
				pc.resetXAxisGyroError();
			}
		}
		double y_axis_gyro_error;
		if (pc.yAxisGyroErrorChanged(y_axis_gyro_error))
		{
			if (safeTalonCall(pigeon2->ConfigYAxisGyroError(y_axis_gyro_error), "pigeon2->ConfigYAxisGyroError()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pogeon2 " << pigeon2_names_[joint_id] << " ConfigYAxisGyroError = " << y_axis_gyro_error);
				ps.setYAxisGyroError(angles::to_degrees(y_axis_gyro_error));
			}
			else
			{
				pc.resetYAxisGyroError();
			}
		}
		double z_axis_gyro_error;
		if (pc.zAxisGyroErrorChanged(z_axis_gyro_error))
		{
			if (safeTalonCall(pigeon2->ConfigXAxisGyroError(z_axis_gyro_error), "pigeon2->ConfigZAxisGyroError()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pogeon2 " << pigeon2_names_[joint_id] << " ConfigZAxisGyroError = " << z_axis_gyro_error);
				ps.setZAxisGyroError(angles::to_degrees(z_axis_gyro_error));
			}
			else
			{
				pc.resetZAxisGyroError();
			}
		}

		bool compass_enable;
		if (pc.compassEnableChanged(compass_enable))
		{
			if (safeTalonCall(pigeon2->ConfigEnableCompass(compass_enable, configTimeoutMs), "pigeon2->ConfigEnableCommpass", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " ConfigEnableCompass = " << compass_enable);
				ps.setCompassEnable(compass_enable);
			}
			else
			{
				pc.resetCompassEnable();
			}
		}

		bool disable_temperature_compensation;
		if (pc.disableTemperatureCompensationChanged(disable_temperature_compensation))
		{
			if (safeTalonCall(pigeon2->ConfigDisableTemperatureCompensation(disable_temperature_compensation, configTimeoutMs), "pigeon2->ConfigDisableTemperatureCompensation", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " ConfigDisableTemperatureCompensation = " << disable_temperature_compensation);
				ps.setDisableTemperatureCompensation(disable_temperature_compensation);
			}
			else
			{
				pc.resetDisableTemperatureCompensation();
			}
		}
		bool disable_no_motion_compensation;
		if (pc.disableNoMotionCalibrationChanged(disable_no_motion_compensation))
		{
			if (safeTalonCall(pigeon2->ConfigDisableNoMotionCalibration(disable_no_motion_compensation, configTimeoutMs), "pigeon2->ConfigDisableNoMotionCalibration", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " ConfigDisableNoMotionCalibration = " << disable_no_motion_compensation);
				ps.setDisableNoMotionCalibration(disable_no_motion_compensation);
			}
			else
			{
				pc.resetDisableNoMotionCalibration();
			}
		}
		
		if (pc.zeroGyroBiasNowChanged())
		{
			if (safeTalonCall(pigeon2->ZeroGyroBiasNow(), "pigeon2->ZeroGyroBiasNow()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " ZeroGyroBiasNow");
			}
			else
			{
				pc.setZeroGyroBiasNow();
			}
		}
		if (pc.clearStickyFaultsChanged())
		{
			if (safeTalonCall(static_cast<ctre::phoenix::ErrorCode>(pigeon2->ClearStickyFaults(timeoutMs)), "pigeon2->SetYawToCompass()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " ClearStickyFaults");
			}
			else
			{
				pc.setClearStickyFaults();
			}
		}

		double set_yaw;
		if (pc.setYawChanged(set_yaw))
		{
			if (safeTalonCall(static_cast<ctre::phoenix::ErrorCode>(pigeon2->SetYaw(angles::to_degrees(set_yaw), timeoutMs)), "pigeon2->SetYaw()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " SetYaw to " << set_yaw);
			}
			else
			{
				pc.resetSetYaw();
			}
		}

		double add_yaw;
		if (pc.addYawChanged(add_yaw))
		{
			if (safeTalonCall(static_cast<ctre::phoenix::ErrorCode>(pigeon2->AddYaw(angles::to_degrees(add_yaw), timeoutMs)), "pigeon2->AddYaw()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " AddYaw to " << add_yaw);
			}
			else
			{
				pc.resetAddYaw();
			}
		}

		if (pc.setYawToCompassChanged())
		{
			if (safeTalonCall(static_cast<ctre::phoenix::ErrorCode>(pigeon2->SetYawToCompass(timeoutMs)), "pigeon2->SetYawToCompass()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " SetYawToCompass");
			}
			else
			{
				pc.setSetYawToCompass();
			}
		}

		if (pc.setAccumZAngleChanged())
		{
			if (safeTalonCall(static_cast<ctre::phoenix::ErrorCode>(pigeon2->SetAccumZAngle(timeoutMs)), "pigeon2->SetAccumZAngle()", ps.getDeviceNumber()))
			{
				ROS_INFO_STREAM("Pigeon2 " << pigeon2_names_[joint_id] << " SetAccumZAngle");
			}
			else
			{
				pc.setSetAccumZAngle();
			}
		}
	}

	write_tracer_.start_unique("nidec");
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		if (nidec_brushless_local_hardwares_[i])
		{
			nidec_brushlesses_[i]->Set(brushless_command_[i]);
			ROS_INFO_STREAM("Write NIDEC brushless " << nidec_brushless_names_[i] <<
					" at PWM channel " << nidec_brushless_pwm_channels_[i] <<
					" / DIO channel " << nidec_brushless_dio_channels_[i] <<
					" = " << brushless_command_[i]);
		}
	}

	write_tracer_.start_unique("digital outputs");
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		// Only invert the desired output once, on the controller
		// where the update originated
		const bool converted_command = (digital_output_command_[i] > 0) ^ (digital_output_inverts_[i] && digital_output_local_updates_[i]);
		if (converted_command != digital_output_state_[i])
		{
			const double double_command = converted_command ? 1.0 : 0.0;
			if (digital_output_local_hardwares_[i])
				digital_outputs_[i]->Set(double_command);
			digital_output_state_[i] = double_command ? 1.0 : 0.0;
			ROS_INFO_STREAM("Wrote digital output " << digital_output_names_[i] <<
					" index " << i << "=" << converted_command);
		}
	}

	write_tracer_.start_unique("pwms");
	for (size_t i = 0; i < num_pwms_; i++)
	{
		const int setpoint = pwm_command_[i] * ((pwm_inverts_[i] & pwm_local_updates_[i]) ? -1 : 1);
		if (pwm_state_[i] != setpoint)
		{
			if (pwm_local_hardwares_[i])
				PWMs_[i]->SetSpeed(setpoint);
			pwm_state_[i] = setpoint;
			ROS_INFO_STREAM("PWM " << pwm_names_[i] <<
					" at channel" <<  pwm_pwm_channels_[i] <<
					" set to " << pwm_state_[i]);
		}
	}

	write_tracer_.start_unique("solenoids");
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		// MODE_POSITION is standard on/off setting
		if (solenoid_mode_[i] == hardware_interface::JointCommandModes::MODE_POSITION)
		{
			const bool on = solenoid_command_[i] > 0;
			if ((solenoid_mode_[i] != prev_solenoid_mode_[i]) || (solenoid_state_[i] != on))
			{
				if (solenoid_local_hardwares_[i])
				{
					solenoids_[i]->Set(on);
				}
				solenoid_state_[i] = on;
				ROS_INFO_STREAM_NAMED(name_, "Write solenoid " << solenoid_names_[i] <<
						" = " << static_cast<int>(on));
			}
		}
		// MODE_EFFORT is PWM via one-shot duration pulses
		else if (solenoid_mode_[i] == hardware_interface::JointCommandModes::MODE_EFFORT)
		{
			if (solenoid_command_[i] > 0)
			{
				if (solenoid_local_hardwares_[i])
				{
					// TODO - do we need to wait for previous one-shot to expire before sending another one?
					solenoids_[i]->SetPulseDuration(static_cast<units::second_t>(solenoid_command_[i]));
					solenoids_[i]->StartPulse();
				}
				ROS_INFO_STREAM_NAMED(name_, "Wrote solenoid one shot " << solenoid_names_[i] <<
						" = " << solenoid_command_[i]);
				// TODO - should we re-load this saved pwm command after the previous
				// one expires, or require a controller to monitor and reload?
				solenoid_pwm_state_[i] = solenoid_command_[i];
				// TODO - this will be polled in the read() function on interface local
				// to the hardware. How will the remote interface get that state update?
				solenoid_state_[i] = 1;
				solenoid_command_[i] = 0;
			}
		}
		else
		{
			ROS_ERROR_STREAM("Invalid solenoid_mode_[i] = " << static_cast<int>(solenoid_mode_[i]));
		}
		prev_solenoid_mode_[i] = solenoid_mode_[i];
	}

	write_tracer_.start_unique("double solenoids");
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		// Not sure if it makes sense to store command values
		// in state or wpilib enum values
		if (double_solenoid_state_[i] != double_solenoid_command_[i])
		{
			if (double_solenoid_local_hardwares_[i])
			{
				frc::DoubleSolenoid::Value value;
				if (double_solenoid_command_[i] >= 1.0)
					value = frc::DoubleSolenoid::Value::kForward;
				else if (double_solenoid_command_[i] <= -1.0)
					value = frc::DoubleSolenoid::Value::kReverse;
				else
					value = frc::DoubleSolenoid::Value::kOff;

				double_solenoids_[i]->Set(value);
			}
			double_solenoid_state_[i] = double_solenoid_command_[i];
			ROS_INFO_STREAM_NAMED(name_, "Wrote double solenoid " << double_solenoid_names_[i] <<
					" = " << double_solenoid_command_[i]);
		}
	}

	write_tracer_.start_unique("rumbles");
	for (size_t i = 0; i < num_rumbles_; i++)
	{
		if (rumble_state_[i] != rumble_command_[i])
		{
			const unsigned int rumbles = *((unsigned int*)(&rumble_command_[i]));
			const unsigned int left_rumble  = (rumbles >> 16) & 0xFFFF;
			const unsigned int right_rumble = (rumbles      ) & 0xFFFF;
			if (rumble_local_hardwares_[i])
				HAL_SetJoystickOutputs(rumble_ports_[i], 0, left_rumble, right_rumble);
			rumble_state_[i] = rumble_command_[i];
			ROS_INFO_STREAM("Wrote rumble " << rumble_names_[i] << " index " << i <<
					"=" << rumble_command_[i] <<
					" left=" << left_rumble <<
					" right=" << right_rumble);
		}
	}

	write_tracer_.start_unique("pcms");
	for (size_t i = 0; i < num_pcms_; i++)
	{
		if (pcm_compressor_closed_loop_enable_command_[i] != pcm_compressor_closed_loop_enable_state_[i])
		{
			if (pcm_local_hardwares_[i])
			{
				const bool setpoint = pcm_compressor_closed_loop_enable_command_[i] > 0;
				if (setpoint)
				{
					pcms_[i]->EnableCompressorDigital();
				}
				else
				{
					pcms_[i]->DisableCompressor();
				}
			}
			// TODO - should we retry if status != 0?
			pcm_compressor_closed_loop_enable_state_[i] = pcm_compressor_closed_loop_enable_command_[i];
			ROS_INFO_STREAM("Wrote pcm " << i << " closedloop enable = "
					<< pcm_compressor_closed_loop_enable_command_[i]);
		}
	}

	write_tracer_.start_unique("phs");
	for (size_t i = 0; i < num_phs_; i++)
	{
		double compressor_min_analog_voltage;
		double compressor_max_analog_voltage;
		bool compressor_force_disable;
		bool compressor_use_digital;
		if (ph_command_[i].closedLoopControlChanged(compressor_min_analog_voltage, compressor_max_analog_voltage,
									 compressor_force_disable, compressor_use_digital))
		{
			if (ph_local_hardwares_[i])
			{
				if (compressor_force_disable)
				{
					phs_[i]->DisableCompressor();
				}
				else
				{
					if (compressor_use_digital)
					{
						phs_[i]->EnableCompressorDigital();
					}
					else
					{
						phs_[i]->EnableCompressorAnalog(static_cast<units::pounds_per_square_inch_t>(compressor_min_analog_voltage),
														static_cast<units::pounds_per_square_inch_t>(compressor_max_analog_voltage));
					}
					// TODO - hybrid?
				}
			}
			ROS_INFO_STREAM("Wrote ph " << i
					<< " min_analog_voltage=" << compressor_min_analog_voltage
					<< " max_analog_voltage=" << compressor_max_analog_voltage
					<< " compressor_force_disable=" << static_cast<int>(compressor_force_disable)
					<< " compressor_use_digital=" << static_cast<int>(compressor_use_digital));
		}
	}

	write_tracer_.start_unique("pdhs");
	for (size_t i = 0; i < num_pdhs_; i++)
	{
		auto &pc = pdh_command_[i];

		if (bool enable; pc.switchableChannelEnableChanged(enable))
		{
			if (pdh_local_hardwares_[i])
			{
				int32_t status = 0;
				HAL_SetREVPDHSwitchableChannel(pdhs_[i], enable, &status);
				if (status == 0)
				{
					ROS_INFO_STREAM("Set PDH " << pdh_names_[i]
							<< " enable = " << static_cast<int>(enable));
				}
				else
				{
					ROS_ERROR_STREAM("Error setting PDH " << pdh_names_[i]
							<< " enable = " << static_cast<int>(enable)
							<< " : status = " << status
							<< " : " << HAL_GetErrorMessage(status));
					pc.resetSwitchableChannelEnable();
				}
			}
		}

		if (pc.clearStickyFaultsChanged())
		{
			if (pdh_local_hardwares_[i])
			{
				int32_t status = 0;
				HAL_ClearREVPDHStickyFaults(pdhs_[i], &status);
				if (status == 0)
				{
					ROS_INFO_STREAM("Cleared sticky faults on PDH " << pdh_names_[i]);
				}
				else
				{
					ROS_ERROR_STREAM("Error clearing sticky faults on " << pdh_names_[i]
							<< " : status = " << status
							<< " : " << HAL_GetErrorMessage(status));
					pc.setClearStickyFaults();
				}
			}
		}

#if 0 // Gone from wpilib code
		if (pc.identifyPDHChanged())
		{
			if (pdh_local_hardwares_[i])
			{
				int32_t status = 0;
				HAL_REV_IdentifyPDH(pdhs_[i], &status);
				if (status == 0)
				{
					ROS_INFO_STREAM("Identified PDH " << pdh_names_[i]);
				}
				else
				{
					ROS_ERROR_STREAM("Error identifying PDH " << pdh_names_[i]
							<< " : status = " << status
							<< " : " << HAL_GetErrorMessage(status));
					pc.setIdentifyPDH();
				}
			}
		}
#endif
	}

	// TODO : what to do about this?
	write_tracer_.start_unique("dummy joints");
	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		if (dummy_joint_locals_[i])
		{
			// Use dummy joints to communicate info between
			// various controllers and driver station smartdash vars
			{
				dummy_joint_effort_[i] = 0;
				//if (dummy_joint_names_[i].substr(2, std::string::npos) == "_angle")
				{
					// position mode
					dummy_joint_velocity_[i] = (dummy_joint_command_[i] - dummy_joint_position_[i]) / period.toSec();
					dummy_joint_position_[i] = dummy_joint_command_[i];
				}
#if 0
				else if (dummy_joint_names_[i].substr(2, std::string::npos) == "_drive")
				{
					// velocity mode
					dummy_joint_position_[i] += dummy_joint_command_[i] * period.toSec();
					dummy_joint_velocity_[i] = dummy_joint_command_[i];
				}
#endif
			}
		}
	}
	write_tracer_.report(60);
}

double FRCRobotInterface::getConversionFactor(int encoder_ticks_per_rotation,
											  hardware_interface::FeedbackDevice encoder_feedback,
											  hardware_interface::TalonMode talon_mode) const
{
	if((talon_mode == hardware_interface::TalonMode_Position) ||
	   (talon_mode == hardware_interface::TalonMode_MotionMagic)) // TODO - maybe motion profile as well?
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2. * M_PI / encoder_ticks_per_rotation;
			case hardware_interface::FeedbackDevice_Analog:
				return 2. * M_PI / 1024.;
			case hardware_interface::FeedbackDevice_IntegratedSensor:
				return 2. * M_PI / 2048.;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				return 2. * M_PI / encoder_ticks_per_rotation;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
				return 1.;
		}
	}
	else if(talon_mode == hardware_interface::TalonMode_Velocity)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return .1;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2. * M_PI / encoder_ticks_per_rotation / .1;
			case hardware_interface::FeedbackDevice_Analog:
				return 2. * M_PI / 1024. / .1;
			case hardware_interface::FeedbackDevice_IntegratedSensor:
				return 2. * M_PI / 2048. / .1;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				return 2. * M_PI / encoder_ticks_per_rotation / .1;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
				return .1;
		}
	}
	else
	{
		//ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
		return 1.;
	}
}

bool FRCRobotInterface::safeTalonConfigCall(ctre::phoenix::ErrorCode error_code, const std::string &talon_method_name, const int talon_id)
{
	talon_config_count_ += 1;
	if (talon_config_count_ > talon_config_count_limit_)
	{
		ROS_INFO_STREAM("Talon config call count for this iteration reached on function " << talon_method_name);
		return false;
	}
	if (!safeTalonCall(error_code, talon_method_name, talon_id, true))
	{
		return false;
	}
	return true;
}
bool FRCRobotInterface::safeTalonCall(ctre::phoenix::ErrorCode error_code, const std::string &talon_method_name, const int talon_id, bool config_call)
{
	//ROS_INFO_STREAM("safeTalonCall(" << talon_method_name << ") = " << error_code);
	std::string error_name;
	static bool error_sent = false;
	switch (error_code)
	{
		case ctre::phoenix::OK :
			can_error_count_ = 0;
			error_sent = false;

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
			error_name = "BufferFull";
			break;
		case ctre::phoenix::CAN_OVERFLOW:
			error_name = "CAN_OVERFLOW";
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
		case ctre::phoenix::BufferFailure :
			error_name = "BufferFailure";
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
		case ctre::phoenix::WrongRemoteLimitSwitchSource :
			error_name = "WrongRemoteLimitSwitchSource";
			break;
		case ctre::phoenix::DoubleVoltageCompensatingWPI :
			error_name = "DoubleVoltageCompensatingWPI";
			break;

		case ctre::phoenix::IncompatibleMode :
			error_name = "IncompatibleMode";
			break;
		case ctre::phoenix::InvalidHandle :
			error_name = "InvalidHandle";
			break;

		case ctre::phoenix::FeatureRequiresHigherFirm:
			error_name = "FeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::TalonFeatureRequiresHigherFirm:
			error_name = "TalonFeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::ConfigFactoryDefaultRequiresHigherFirm:
			error_name = "ConfigFactoryDefaultRequiresHigherFirm";
			break;
		case ctre::phoenix::ConfigMotionSCurveRequiresHigherFirm:
			error_name = "ConfigMotionSCurveRequiresHigherFirm";
			break;
		case ctre::phoenix::TalonFXFirmwarePreVBatDetect:
			error_name = "TalonFXFirmwarePreVBatDetect";
			break;
		case ctre::phoenix::LibraryCouldNotBeLoaded :
			error_name = "LibraryCouldNotBeLoaded";
			break;
		case ctre::phoenix::MissingRoutineInLibrary :
			error_name = "MissingRoutineInLibrary";
			break;
		case ctre::phoenix::ResourceNotAvailable :
			error_name = "ResourceNotAvailable";
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
			error_name = "ConrolModeNotSupportedYet";
			break;
		case ctre::phoenix::CascadedPIDNotSupporteYet:
			error_name = "CascadedPIDNotSupporteYet/AuxiliaryPIDNotSupportedYet";
			break;
		case ctre::phoenix::RemoteSensorsNotSupportedYet:
			error_name = "RemoteSensorsNotSupportedYet";
			break;
		case ctre::phoenix::MotProfFirmThreshold:
			error_name = "MotProfFirmThreshold";
			break;
		case ctre::phoenix::MotProfFirmThreshold2:
			error_name = "MotProfFirmThreshold2";
			break;

		case ctre::phoenix::MusicFileNotFound:
			error_name = "MusicFileNotFound";
			break;
		case ctre::phoenix::MusicFileWrongSize:
			error_name = "MusicFileWrongSize";
			break;
		case ctre::phoenix::MusicFileTooNew:
			error_name = "MusicFileTooNew";
			break;
		case ctre::phoenix::MusicFileInvalid:
			error_name = "MusicFileInvalid";
			break;
		case ctre::phoenix::InvalidOrchestraAction:
			error_name = "InvalidOrchestraAction";
			break;
		case ctre::phoenix::MusicFileTooOld:
			error_name = "MusicFileTooOld";
			break;
		case ctre::phoenix::MusicInterrupted:
			error_name = "MusicInterrupted";
			break;
		case ctre::phoenix::MusicNotSupported:
			error_name = "MusicNotSupported";
			break;


		case kInvalidGuid:
			error_name = "kInvalidGuid";
			break;
		case kInvalidClass:
			error_name = "kInvalidClass";
			break;
		case kInvalidProtocol:
			error_name = "kInvalidProtocol";
			break;
		case kInvalidPath:
			error_name = "kInvalidPath";
			break;
		case kGeneralWinUsbError:
			error_name = "kGeneralWinUsbError";
			break;
		case kFailedSetup:
			error_name = "kFailedSetup";
			break;
		case kListenFailed:
			error_name = "kListenFailed";
			break;
		case kSendFailed:
			error_name = "kSendFailed";
			break;
		case kReceiveFailed:
			error_name = "kReceiveFailed";
			break;
		case kInvalidRespFormat:
			error_name = "kInvalidRespFormat";
			break;
		case kWinUsbInitFailed:
			error_name = "kWinUsbInitFailed";
			break;
		case kWinUsbQueryFailed:
			error_name = "kWinUsbQueryFailed";
			break;
		case kWinUsbGeneralError:
			error_name = "kWinUsbGeneralError";
			break;
		case kAccessDenied:
			error_name = "kAccessDenied";
			break;
		case kFirmwareInvalidResponse:
			error_name = "kFirmwareInvalidResponse";
			break;

		default:
			{
				std::stringstream s;
				s << "Unknown Talon error " << error_code;
				error_name = s.str();
				break;
			}

	}
	ROS_ERROR_STREAM("Error : CANid = " << talon_id << " calling " << talon_method_name << " : " << error_name);
	can_error_count_++;
	if ((can_error_count_> 1000) && !error_sent)
	{
		HAL_SendError(true, -1, false, "safeTalonCall - too many CAN bus errors!", "", "", true);
		error_sent = true;
	}
	return false;
}


void FRCRobotInterface::reset()
{
}

bool FRCRobotInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &/*start_list*/,
									  const std::list<hardware_interface::ControllerInfo> &/*stop_list*/)
{
	return true;
}

void FRCRobotInterface::loadURDF(ros::NodeHandle &/*nh*/, std::string /*param_name*/)
{
	return;
#if 0
	std::string urdf_string;
	urdf_model_ = new urdf::Model();

	// search and wait for robot_description on param server
	while (urdf_string.empty() && ros::ok())
	{
		std::string search_param_name;
		if (nh.searchParam(param_name, search_param_name))
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << search_param_name);
			nh.getParam(search_param_name, urdf_string);
		}
		else
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << param_name);
			nh.getParam(param_name, urdf_string);
		}

		usleep(100000);
	}

	if (!urdf_model_->initString(urdf_string))
		ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
	else
		ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
#endif
}

}  // namespace
