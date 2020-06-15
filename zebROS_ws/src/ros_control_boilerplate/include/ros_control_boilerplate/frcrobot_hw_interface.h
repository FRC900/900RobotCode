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
           For a more detailed simulation example, see sim_hw_interface.h
*/

#pragma once

#include <thread>

#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/CANifier.h>
#include "WPILibVersion.h"
#include <frc/AnalogInput.h>
#include <frc/DriverStation.h>
#include <frc/NidecBrushless.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/PWMSpeedController.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/Joystick.h>
#include <hal/HALBase.h>
#include <hal/DriverStation.h>
#include <hal/FRCUsageReporting.h>

#include <AHRS.h>

#include "frc_interfaces/robot_controller_interface.h"
#include "frc_msgs/MatchSpecificData.h"

#include "ros_control_boilerplate/AS726x.h"
#include "ros_control_boilerplate/as726x_convert.h"
#include "ros_control_boilerplate/cancoder_convert.h"
#include "ros_control_boilerplate/canifier_convert.h"
#include "ros_control_boilerplate/DSError.h"
#include "ros_control_boilerplate/frc_robot_interface.h"
#include "ros_control_boilerplate/talon_convert.h"
#include "ros_control_boilerplate/tracer.h"

namespace frcrobot_control
{
// Very simple code to communicate with the HAL. This recieves
// packets from the driver station and lets the field management
// know our robot is alive.
class ROSIterativeRobot
{
	public:
		ROSIterativeRobot(void) : m_ds(frc::DriverStation::GetInstance())
		{
			if (!HAL_Initialize(500, 0))
			{
				ROS_ERROR("FATAL ERROR: HAL could not be initialized");
				std::terminate();
			}
			std::FILE* file = std::fopen("/tmp/frc_versions/FRC_Lib_Version.ini", "w");

			if (file != nullptr) {
				std::fputs("C++ ", file);
				std::fputs(GetWPILibVersion(), file);
				std::fclose(file);
			}

			HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_ROS);
			HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 900, 0, "field centric swerve");
			//HAL_Report(HALUsageReporting::kResourceType_kKinematics, HALUsageReporting::kKinematics_SwerveDrive);
#if 0
			for (int i = 0; i < 900; i++)
				HAL_Report(HALUsageReporting::kResourceType_NidecBrushless, 900);
#endif
			HAL_Report(HALUsageReporting::kResourceType_Language, 900, 0, "C++/CMake/Javascript/Python/Shell/PERL");
		}

		void StartCompetition(void) const
		{
			HAL_ObserveUserProgramStarting();
		}

		void OneIteration(void) const
		{
			// Call the appropriate function depending upon the current robot mode
			if (m_ds.IsDisabled()) {
				HAL_ObserveUserProgramDisabled();
			} else if (m_ds.IsAutonomous()) {
				HAL_ObserveUserProgramAutonomous();
			} else if (m_ds.IsOperatorControl()) {
				HAL_ObserveUserProgramTeleop();
			} else {
				HAL_ObserveUserProgramTest();
			}
		}
	private:
		frc::DriverStation& m_ds;
};

class DoubleSolenoidHandle
{
	public:
		DoubleSolenoidHandle(HAL_SolenoidHandle forward, HAL_SolenoidHandle reverse)
			: forward_(forward)
		    , reverse_(reverse)
	{
	}
		HAL_SolenoidHandle forward_;
		HAL_SolenoidHandle reverse_;
};

/// \brief Hardware interface for a robot
class FRCRobotHWInterface : public ros_control_boilerplate::FRCRobotInterface
{
	public:
		/**
		 * \brief Constructor
		 * \param nh - Node handle for topics.
		 */
		FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		~FRCRobotHWInterface();

		/** \brief Initialize the hardware interface */
		virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

		/** \brief Read the state from the robot hardware. */
		virtual void read(const ros::Time& time, const ros::Duration& period) override;

		/** \brief Write the command to the robot hardware. */
		virtual void write(const ros::Time& time, const ros::Duration& period) override;

	private:
		/* Get conversion factor for position, velocity, and closed-loop stuff */
		double getConversionFactor(int encoder_ticks_per_rotation, hardware_interface::FeedbackDevice encoder_feedback, hardware_interface::TalonMode talon_mode) const;

		bool safeTalonCall(ctre::phoenix::ErrorCode error_code,
				const std::string &talon_method_name);

		//certain data will be read at a slower rate than the main loop, for computational efficiency
		//robot iteration calls - sending stuff to driver station
		double t_prev_robot_iteration_;
		double robot_iteration_hz_;

		double t_prev_joystick_read_;
		double joystick_read_hz_;

		double t_prev_match_data_read_;
		double match_data_read_hz_;

		double t_prev_robot_controller_read_;
		double robot_controller_read_hz_;

		// Count sequential CAN errors
		size_t can_error_count_;

		std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController>> ctre_mcs_;

		// Maintain a separate read thread for each talon SRX
		std::vector<std::shared_ptr<std::mutex>> ctre_mc_read_state_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::TalonHWState>> ctre_mc_read_thread_states_;
		std::vector<std::thread> ctre_mc_read_threads_;
		void ctre_mc_read_thread(std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController> ctre_mc, std::shared_ptr<hardware_interface::TalonHWState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);

		std::vector<std::shared_ptr<ctre::phoenix::CANifier>> canifiers_;
		std::vector<std::shared_ptr<std::mutex>> canifier_read_state_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::canifier::CANifierHWState>> canifier_read_thread_states_;
		std::vector<std::thread> canifier_read_threads_;
		void canifier_read_thread(std::shared_ptr<ctre::phoenix::CANifier> canifier, std::shared_ptr<hardware_interface::canifier::CANifierHWState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);

		std::vector<std::shared_ptr<ctre::phoenix::sensors::CANCoder>> cancoders_;
		std::vector<std::shared_ptr<std::mutex>> cancoder_read_state_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::cancoder::CANCoderHWState>> cancoder_read_thread_states_;
		std::vector<std::thread> cancoder_read_threads_;
		void cancoder_read_thread(std::shared_ptr<ctre::phoenix::sensors::CANCoder> cancoder, std::shared_ptr<hardware_interface::cancoder::CANCoderHWState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);

		std::vector<std::shared_ptr<frc::NidecBrushless>> nidec_brushlesses_;
		std::vector<std::shared_ptr<frc::DigitalInput>> digital_inputs_;
		std::vector<std::shared_ptr<frc::DigitalOutput>> digital_outputs_;
		std::vector<std::shared_ptr<frc::PWM>> PWMs_;
		std::vector<HAL_SolenoidHandle> solenoids_;
		std::vector<DoubleSolenoidHandle> double_solenoids_;
		std::vector<std::shared_ptr<AHRS>> navXs_;
		std::vector<std::shared_ptr<frc::AnalogInput>> analog_inputs_;

		std::vector<std::shared_ptr<std::mutex>> pcm_read_thread_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::PCMState>> pcm_read_thread_state_;
		void pcm_read_thread(HAL_CompressorHandle compressor_handle, int32_t pcm_id, std::shared_ptr<hardware_interface::PCMState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);
		std::vector<std::thread> pcm_thread_;
		std::vector<HAL_CompressorHandle> compressors_;

		std::vector<std::shared_ptr<std::mutex>> pdp_read_thread_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::PDPHWState>> pdp_read_thread_state_;
		void pdp_read_thread(int32_t pdp, std::shared_ptr<hardware_interface::PDPHWState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);
		std::vector<std::thread> pdp_thread_;
		std::vector<int32_t> pdps_;

		std::vector<std::shared_ptr<frc::Joystick>> joysticks_;
		std::vector<std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Joy>>> realtime_pub_joysticks_;

		std::vector<std::shared_ptr<as726x::roboRIO_AS726x>> as726xs_;
		std::vector<std::shared_ptr<std::mutex>> as726x_read_thread_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::as726x::AS726xState>> as726x_read_thread_state_;
		void as726x_read_thread(std::shared_ptr<as726x::roboRIO_AS726x> as726x, std::shared_ptr<hardware_interface::as726x::AS726xState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);
		std::vector<std::thread> as726x_thread_;

		std::unique_ptr<ROSIterativeRobot> robot_;

		Tracer read_tracer_;

		as726x_convert::AS726xConvert as726x_convert_;
		cancoder_convert::CANCoderConvert cancoder_convert_;
		canifier_convert::CANifierConvert canifier_convert_;
		talon_convert::TalonConvert talon_convert_;

		bool DSErrorCallback(ros_control_boilerplate::DSError::Request &req, ros_control_boilerplate::DSError::Response &res);
		ros::ServiceServer ds_error_server_;
};  // class

}  // namespace

