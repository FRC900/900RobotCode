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

#include <sensor_msgs/Joy.h>
#include "frc_msgs/MatchSpecificData.h"

#include "ros_control_boilerplate/frc_robot_interface.h"
#include "ros_control_boilerplate/LineBreakSensors.h"
#include "ros_control_boilerplate/set_limit_switch.h"

namespace frcrobot_control
{
/// \brief Hardware interface for a robot
class FRCRobotSimInterface : public ros_control_boilerplate::FRCRobotInterface
{
	public:
		/**
		 * \brief Constructor
		 * \param nh - Node handle for topics.
		 */
		FRCRobotSimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);
		~FRCRobotSimInterface();

		virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) override;

		/** \brief Read the state from the robot hardware. */
		virtual void read(const ros::Time& time, const ros::Duration& period) override;

		/** \brief Write the command to the robot hardware. */
		virtual void write(const ros::Time& time, const ros::Duration& period) override;

		virtual bool setlimit(ros_control_boilerplate::set_limit_switch::Request &req,ros_control_boilerplate::set_limit_switch::Response &res);

	private:
        ros::Subscriber match_data_sub_;
		std::vector<ros::Subscriber> joystick_subs_;
        void match_data_callback(const frc_msgs::MatchSpecificData &match_data);
		void joystickCallback(const sensor_msgs::JoyConstPtr &msg, int32_t joystick_num);
		bool evaluateDigitalInput(ros_control_boilerplate::LineBreakSensors::Request &req, ros_control_boilerplate::LineBreakSensors::Response &res);

		ros::ServiceServer linebreak_sensor_srv_;
		ros::ServiceServer limit_switch_srv_;
};  // class

}  // namespace
