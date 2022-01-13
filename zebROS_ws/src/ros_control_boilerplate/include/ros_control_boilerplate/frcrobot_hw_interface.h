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

#include <ctre/phoenix/CANifier.h>
#include <ctre/phoenix/music/Orchestra.h>
#include "WPILibVersion.h"
#include <frc/AnalogInput.h>
#include <frc/Compressor.h>
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DriverStation.h>
#include <frc/Solenoid.h>
#include <frc/motorcontrol/NidecBrushless.h>
#include <frc/motorcontrol/PWMMotorController.h>
#include <hal/HALBase.h>
#include <hal/DriverStation.h>
#include <hal/FRCUsageReporting.h>
#include <rev/CANSparkMax.h>

#include <AHRS.h>

#include "frc_interfaces/robot_controller_interface.h"
#include "frc_msgs/MatchSpecificData.h"

#include "ros_control_boilerplate/AS726x.h"
#include "ros_control_boilerplate/as726x_convert.h"
#include "ros_control_boilerplate/canifier_convert.h"
#include "ros_control_boilerplate/DSError.h"
#include "ros_control_boilerplate/frc_robot_interface.h"
#include "ros_control_boilerplate/rev_convert.h"
#include "ros_control_boilerplate/tracer.h"

namespace ros_control_boilerplate
{
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
		bool safeSparkMaxCall(rev::REVLibError can_error,
				const std::string &spark_max_method_name,
				int id);

		std::vector<std::shared_ptr<ctre::phoenix::CANifier>> canifiers_;
		std::vector<std::shared_ptr<std::mutex>> canifier_read_state_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::canifier::CANifierHWState>> canifier_read_thread_states_;
		std::vector<std::thread> canifier_read_threads_;
		void canifier_read_thread(std::shared_ptr<ctre::phoenix::CANifier> canifier,
				std::shared_ptr<hardware_interface::canifier::CANifierHWState> state,
				std::shared_ptr<std::mutex> mutex,
				std::unique_ptr<Tracer> tracer,
				double poll_frequency);

		std::vector<std::shared_ptr<rev::CANSparkMax>>           can_spark_maxs_;
		std::vector<std::shared_ptr<rev::SparkMaxPIDController>> can_spark_max_pid_controllers_;

		// Maintain a separate read thread for each spark_max SRX
		std::vector<std::shared_ptr<std::mutex>> spark_max_read_state_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::SparkMaxHWState>> spark_max_read_thread_states_;
		std::vector<std::thread> spark_max_read_threads_;
		void spark_max_read_thread(std::shared_ptr<rev::CANSparkMax> spark_max,
				std::shared_ptr<hardware_interface::SparkMaxHWState> state,
				std::shared_ptr<std::mutex> mutex,
				std::unique_ptr<Tracer> tracer,
				double poll_frequency);

		std::vector<std::shared_ptr<as726x::roboRIO_AS726x>> as726xs_;
		std::vector<std::shared_ptr<std::mutex>> as726x_read_thread_mutexes_;
		std::vector<std::shared_ptr<hardware_interface::as726x::AS726xState>> as726x_read_thread_state_;
		std::vector<std::thread> as726x_thread_;
		void as726x_read_thread(std::shared_ptr<as726x::roboRIO_AS726x> as726x,
				std::shared_ptr<hardware_interface::as726x::AS726xState> state,
				std::shared_ptr<std::mutex> mutex,
				std::unique_ptr<Tracer> tracer,
				double poll_frequency);

		std::vector<std::shared_ptr<ctre::phoenix::music::Orchestra>> talon_orchestras_;

		as726x_convert::AS726xConvert as726x_convert_;
		canifier_convert::CANifierConvert canifier_convert_;
		rev_convert::RevConvert rev_convert_;

		bool DSErrorCallback(ros_control_boilerplate::DSError::Request &req, ros_control_boilerplate::DSError::Response &res);
		ros::ServiceServer ds_error_server_;
};  // class

}  // namespace

