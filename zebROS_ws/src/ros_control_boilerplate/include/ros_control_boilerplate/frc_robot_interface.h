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
   Desc:   Example ros_control hardware interface that performs a perfect control loop for
   simulation
*/

#ifndef FRC_ROBOT_INTERFACE_INC_
#define FRC_ROBOT_INTERFACE_INC_

#include <thread>

#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include "ros_control_boilerplate/tracer.h"

class Devices;

namespace ros_control_boilerplate
{

/// \brief Hardware interface for a robot
class FRCRobotInterface : public hardware_interface::RobotHW
{
	public:
		//******Stuff from frcrobot_hw_interface
		/**
		 * \brief Constructor
		 * \param nh - Node handle for topics.
		 * \param urdf - optional pointer to a parsed robot model
		 */
		FRCRobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		FRCRobotInterface(const FRCRobotInterface &) = delete;
		FRCRobotInterface(const FRCRobotInterface &&) noexcept = delete;
		~FRCRobotInterface() override;
		FRCRobotInterface& operator=(const FRCRobotInterface &) = delete;
		FRCRobotInterface& operator=(const FRCRobotInterface &&) noexcept = delete;

		/** \brief Initialize the hardware interface */
		bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

		/** \brief Read the state from the robot hardware. */
		void read(const ros::Time& time, const ros::Duration& period) override;

		/** \brief Write the command to the robot hardware. */
		void write(const ros::Time& time, const ros::Duration& period) override;

		//******
		/**
		 * \brief Check (in non-realtime) if given controllers could be started and stopped from the
		 * current state of the RobotHW
		 * with regard to necessary hardware interface switches. Start and stop list are disjoint.
		 * This is just a check, the actual switch is done in doSwitch()
		 */
		bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &/*start_list*/,
							       const std::list<hardware_interface::ControllerInfo> &/*stop_list*/) override;

		/**
		 * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
		 * and stop the given controllers.
		 * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
		 */
		void doSwitch(const std::list<hardware_interface::ControllerInfo> &/*start_list*/,
							  const std::list<hardware_interface::ControllerInfo> &/*stop_list*/) override
		{
		}

	protected:
		/** \brief Get the URDF XML from the parameter server */
		virtual void loadURDF(ros::NodeHandle &nh, std::string param_name);

		// Short name of this class
		std::string name_;

		bool run_hal_robot_{true};
		std::string can_interface_{"can0"};

		urdf::Model *urdf_model_{nullptr};

		bool robot_code_ready_{false};
		bool last_robot_enabled_{false};

		Tracer read_tracer_;
		Tracer write_tracer_;

		std::vector<std::unique_ptr<Devices>> devices_;
};  // class

}  // namespace
#endif
