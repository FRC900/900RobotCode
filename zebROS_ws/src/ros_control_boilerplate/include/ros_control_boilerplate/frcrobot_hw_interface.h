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

#include <atomic>
#include <thread>

#include <ros_control_boilerplate/frc_robot_interface.h>
#include <ros_control_boilerplate/tracer.h>
#include <realtime_tools/realtime_publisher.h>

#include <frc_interfaces/robot_controller_interface.h>
#include "ros_control_boilerplate/AutoMode.h"
#include "frc_msgs/MatchSpecificData.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/IterativeRobotBase.h>
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
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/HALBase.h>
#include <hal/DriverStation.h>
#include <hal/FRCUsageReporting.h>

#include <AHRS.h>

#include <frc_interfaces/robot_controller_interface.h>

namespace frcrobot_control
{
// Very simple code to communicate with the HAL. This recieves
// packets from the driver station and lets the field management
// know our robot is alive.
class ROSIterativeRobot : public frc::IterativeRobotBase
{
	public:
		ROSIterativeRobot(void) : IterativeRobotBase(0.02)
		{
			HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_ROS);
			HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 900, 0, "field centric swerve");
#if 0
			for (int i = 0; i < 900; i++)
				HAL_Report(HALUsageReporting::kResourceType_NidecBrushless, 900);
#endif
			HAL_Report(HALUsageReporting::kResourceType_NidecBrushless, 900);
			HAL_Report(HALUsageReporting::kResourceType_Language, 900, 0, "C++/CMake/Javascript/Python/Shell/PERL");
		}

		void StartCompetition(void) override
		{
			RobotInit();
			HAL_ObserveUserProgramStarting();
			LiveWindow::GetInstance()->SetEnabled(false);
			LiveWindow::GetInstance()->DisableAllTelemetry();
		}

		void OneIteration(void)
		{
			// wait for driver station data so the loop doesn't hog the CPU
			//DriverStation::GetInstance().WaitForData(.01);
			LoopFunc();
		}

	private:
		void LoopFunc(bool use_livewindow = false)
		{
			// Call the appropriate function depending upon the current robot mode
			if (IsDisabled()) {
				// Call DisabledInit() if we are now just entering disabled mode from
				// either a different mode or from power-on.
				if (m_lastMode != Mode::kDisabled) {
					if (use_livewindow)
						LiveWindow::GetInstance()->SetEnabled(false);
					DisabledInit();
					m_lastMode = Mode::kDisabled;
				}
				HAL_ObserveUserProgramDisabled();
				DisabledPeriodic();
			} else if (IsAutonomous()) {
				// Call AutonomousInit() if we are now just entering autonomous mode from
				// either a different mode or from power-on.
				if (m_lastMode != Mode::kAutonomous) {
					if (use_livewindow)
						LiveWindow::GetInstance()->SetEnabled(false);
					AutonomousInit();
					m_lastMode = Mode::kAutonomous;
				}
				HAL_ObserveUserProgramAutonomous();
				AutonomousPeriodic();
			} else if (IsOperatorControl()) {
				// Call TeleopInit() if we are now just entering teleop mode from
				// either a different mode or from power-on.
				if (m_lastMode != Mode::kTeleop) {
					if (use_livewindow)
						LiveWindow::GetInstance()->SetEnabled(false);
					TeleopInit();
					m_lastMode = Mode::kTeleop;
					//Scheduler::GetInstance()->SetEnabled(true);
				}
				HAL_ObserveUserProgramTeleop();
				TeleopPeriodic();
			} else {
				// Call TestInit() if we are now just entering test mode from
				// either a different mode or from power-on.
				if (m_lastMode != Mode::kTest) {
					if (use_livewindow)
						LiveWindow::GetInstance()->SetEnabled(true);
					TestInit();
					m_lastMode = Mode::kTest;
				}
				HAL_ObserveUserProgramTest();
				TestPeriodic();
			}
			RobotPeriodic();
			//SmartDashboard::UpdateValues();
			if (use_livewindow)
				LiveWindow::GetInstance()->UpdateValues();
		}

		enum class Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

		Mode m_lastMode = Mode::kNone;
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
		virtual void init(void) override;

		/** \brief Read the state from the robot hardware. */
		virtual void read(ros::Duration &elapsed_time) override;

		/** \brief Write the command to the robot hardware. */
		virtual void write(ros::Duration &elapsed_time) override;

	protected:
		virtual std::vector<ros_control_boilerplate::DummyJoint> getDummyJoints(void) override;

	private:
		void process_motion_profile_buffer_thread(double hz);

		/* Get conversion factor for position, velocity, and closed-loop stuff */

		double getConversionFactor(int encoder_ticks_per_rotation, hardware_interface::FeedbackDevice encoder_feedback, hardware_interface::TalonMode talon_mode);

		bool convertControlMode(const hardware_interface::TalonMode input_mode,
								ctre::phoenix::motorcontrol::ControlMode &output_mode);
		bool convertDemand1Type( const hardware_interface::DemandType input,
				ctre::phoenix::motorcontrol::DemandType &output);
		bool convertNeutralMode(const hardware_interface::NeutralMode input_mode,
								ctre::phoenix::motorcontrol::NeutralMode &output_mode);
		bool convertFeedbackDevice(
			const hardware_interface::FeedbackDevice input_fd,
			ctre::phoenix::motorcontrol::FeedbackDevice &output_fd);
		bool convertRemoteFeedbackDevice(
			const hardware_interface::RemoteFeedbackDevice input_fd,
			ctre::phoenix::motorcontrol::RemoteFeedbackDevice &output_fd);
		bool convertRemoteSensorSource(
				const hardware_interface::RemoteSensorSource input_rss,
				ctre::phoenix::motorcontrol::RemoteSensorSource &output_rss);
		bool convertLimitSwitchSource(
			const hardware_interface::LimitSwitchSource input_ls,
			ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls);
		bool convertRemoteLimitSwitchSource(
			const hardware_interface::RemoteLimitSwitchSource input_ls,
			ctre::phoenix::motorcontrol::RemoteLimitSwitchSource &output_ls);
		bool convertLimitSwitchNormal(
			const hardware_interface::LimitSwitchNormal input_ls,
			ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls);
		bool convertVelocityMeasurementPeriod(
			const hardware_interface::VelocityMeasurementPeriod input_v_m_p,
			ctre::phoenix::motorcontrol::VelocityMeasPeriod &output_v_m_period);
		bool convertStatusFrame(const hardware_interface::StatusFrame input,
			ctre::phoenix::motorcontrol::StatusFrameEnhanced &output);
		bool convertControlFrame(const hardware_interface::ControlFrame input,
			ctre::phoenix::motorcontrol::ControlFrame &output);

		bool safeTalonCall(ctre::phoenix::ErrorCode error_code,
				const std::string &talon_method_name);

		double navX_zero_;

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

		std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController>> ctre_mcs_;

		// Maintain a separate read thread for each talon SRX
		std::vector<std::shared_ptr<std::mutex>> ctre_mc_read_state_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::TalonHWState>> ctre_mc_read_thread_states_;
		std::vector<std::thread> ctre_mc_read_threads_;
		void ctre_mc_read_thread(std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController> ctre_mc, std::shared_ptr<hardware_interface::TalonHWState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);

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

		std::thread motion_profile_thread_;
		std::vector<std::shared_ptr<std::mutex>> motion_profile_mutexes_;

		std::vector<std::shared_ptr<std::mutex>> pdp_read_thread_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::PDPHWState>> pdp_read_thread_state_;
		void pdp_read_thread(int32_t pdp, std::shared_ptr<hardware_interface::PDPHWState> state, std::shared_ptr<std::mutex> mutex, std::unique_ptr<Tracer> tracer);
		std::vector<std::thread> pdp_thread_;
		std::vector<int32_t> pdps_;

		std::vector<std::shared_ptr<Joystick>> joysticks_;
		std::vector<std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Joy>>> realtime_pub_joysticks_;

		std::unique_ptr<ROSIterativeRobot> robot_;

		Tracer read_tracer_;
};  // class

}  // namespace

