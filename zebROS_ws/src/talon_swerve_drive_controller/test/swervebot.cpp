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

// ROS
#include <ros/ros.h>


#include "swervebot.h"
#include "../src/swerve_drive_controller.cpp"

class SimTime
{
	public:
		SimTime operator+=(const ros::Duration &duration)
		{
			sim_time_ += duration;
			most_recent_duration_ = duration;
			return *this;
		}
		ros::Time sim_time_{ros::Time::now()};
		ros::Duration most_recent_duration_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swervebot");
	ros::NodeHandle nh;

	SimTime sim_time;
	SwerveBot<4> robot;
	talon_swerve_drive_controller::TalonSwerveDriveController<4, hardware_interface::talonfxpro::TalonFXProCommandInterface> swerve_controller;

	// Initialize sensor state before starting controller
	robot.set_motor_angles({0, 0, 0, 0});
	robot.set_motor_speeds({1, 1, 1, 1});
	// The last two values are value, slope.  For testing, having slope be 0 might 
	// be most useful
	robot.latency_compensation_state_.setEntry("bl_angle", sim_time.sim_time_, 0, 0);
	robot.latency_compensation_state_.setEntry("bl_drive", sim_time.sim_time_, 1, 0);
	robot.latency_compensation_state_.setEntry("br_angle", sim_time.sim_time_, 0, 0);
	robot.latency_compensation_state_.setEntry("br_drive", sim_time.sim_time_, 1, 0);
	robot.latency_compensation_state_.setEntry("fl_angle", sim_time.sim_time_, 0, 0);
	robot.latency_compensation_state_.setEntry("fl_drive", sim_time.sim_time_, 1, 0);
	robot.latency_compensation_state_.setEntry("fr_angle", sim_time.sim_time_, 0, 0);
	robot.latency_compensation_state_.setEntry("fr_drive", sim_time.sim_time_, 1, 0);
	robot.latency_compensation_state_.setEntry("pigeon2", sim_time.sim_time_, 10, 0);

	ros::NodeHandle controller_nh("swerve_drive_controller");
	swerve_controller.init(&robot, nh, controller_nh);
	swerve_controller.starting(sim_time.sim_time_);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	swerve_controller.update(sim_time.sim_time_, ros::Duration{});
	sim_time += ros::Duration{0.2};
	robot.latency_compensation_state_.setEntry("pigeon2", sim_time.sim_time_, 10 + M_PI / 2., 0);
	swerve_controller.update(sim_time.sim_time_, sim_time.most_recent_duration_);
	spinner.stop();

	return 0;
}
