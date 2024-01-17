#ifndef SWERVEBOT_H_INC__
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials
#include <array>
#include <memory>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>

#include "ctre_interfaces/talonfxpro_command_interface.h"
#include "ctre_interfaces/latency_compensation_state_interface.h"

// NaN
#include <limits>

// ostringstream
#include <sstream>

template <size_t NUM_JOINTS = 4>
class SwerveBot : public hardware_interface::RobotHW
{
	public:
		SwerveBot()
		{
			// Intialize raw data
			for (size_t i = 0; i < NUM_JOINTS; ++i)
			{
				angle_talon_states_.emplace_back(std::make_unique<hardware_interface::talonfxpro::TalonFXProHWState>(10+i));
				drive_talon_states_.emplace_back(std::make_unique<hardware_interface::talonfxpro::TalonFXProHWState>(20+i));
				angle_talon_commands_.emplace_back(std::make_unique<hardware_interface::talonfxpro::TalonFXProHWCommand>());
				drive_talon_commands_.emplace_back(std::make_unique<hardware_interface::talonfxpro::TalonFXProHWCommand>());
			}
			auto make_talon = [&](const char *suffix, const auto i, auto &talon_states, auto &talon_commands)
			{
				std::ostringstream angle_name;
				angle_name << motor_prefixes_[i] << suffix;
				latency_compensation_state_.addEntry(angle_name.str());

				hardware_interface::talonfxpro::TalonFXProStateHandle state_handle(angle_name.str(), talon_states[i].get());
				talon_state_interface_.registerHandle(state_handle);

				hardware_interface::talonfxpro::TalonFXProCommandHandle cmd_handle(talon_state_interface_.getHandle(angle_name.str()), talon_commands[i].get());
				talon_command_interface_.registerHandle(cmd_handle);

			};
			// Connect and register the joint state and velocity interface
			for (size_t i = 0; i < NUM_JOINTS; ++i)
			{
				make_talon("angle", i, angle_talon_states_, angle_talon_commands_);
				make_talon("drive", i, drive_talon_states_, drive_talon_commands_);
			}
			registerInterface(&talon_state_interface_);
			registerInterface(&talon_command_interface_);

			latency_compensation_state_.addEntry("pigeon2");
			hardware_interface::latency_compensation::CTRELatencyCompensationStateHandle latency_compensation_state_handle("latency_compensation", &latency_compensation_state_);
			latency_compensation_state_interface_.registerHandle(latency_compensation_state_handle);
			registerInterface(&latency_compensation_state_interface_);
		}

		void set_motor_speeds(const std::array<double, NUM_JOINTS> &speeds)
		{
			for (size_t i = 0; i < NUM_JOINTS; i++)
			{
				drive_talon_states_[i]->setVelocity(speeds[i]);
			}
		}
		void set_motor_angles(const std::array<double, NUM_JOINTS> &angles)
		{
			for (size_t i = 0; i < NUM_JOINTS; i++)
			{
				angle_talon_states_[i]->setPosition(angles[i]);
			}
		}

		hardware_interface::talonfxpro::TalonFXProStateInterface talon_state_interface_;
		hardware_interface::talonfxpro::TalonFXProCommandInterface talon_command_interface_;
		hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface latency_compensation_state_interface_;
		std::vector<std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWState>> angle_talon_states_;
		std::vector<std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWCommand>> angle_talon_commands_;
		std::vector<std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWState>> drive_talon_states_;
		std::vector<std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWCommand>> drive_talon_commands_;
		hardware_interface::latency_compensation::CTRELatencyCompensationState latency_compensation_state_{"latency_compensation"};
		const std::array<std::string, NUM_JOINTS> motor_prefixes_{"fl_",
																  "fr_",
																  "bl_",
																  "br_"};
		ros::NodeHandle nh_;
};
#endif