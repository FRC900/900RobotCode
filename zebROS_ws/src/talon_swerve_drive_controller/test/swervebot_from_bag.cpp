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
#include <talon_state_msgs/LatencyCompensationState.h>

class SimTime
{
	public:
		SimTime operator+=(const ros::Duration &duration)
		{
			sim_time_ += duration;
			most_recent_duration_ = duration;
			return *this;
		}
		ros::Time sim_time_ = ros::Time(0);
		ros::Duration most_recent_duration_;
};

class SwerveBotFromBag {

public:
	SwerveBotFromBag(): robot(), swerve_controller(), nh(), sim_time() {

		ros::NodeHandle controller_nh("swerve_drive_controller");
		swerve_controller.init(&robot, nh, controller_nh);
		swerve_controller.starting(sim_time.sim_time_);
		swerve_controller.state_ = controller_interface::ControllerBase::ControllerState::INITIALIZED;

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

		latency_compensation_state_sub = nh.subscribe("/frcrobot_jetson/latency_compensation_states", 1, &SwerveBotFromBag::latencyCompensationStateCallback, this);

	}

private:
	void latencyCompensationStateCallback(const talon_state_msgs::LatencyCompensationState::ConstPtr &msg)
	{
		swerve_controller.state_ = controller_interface::ControllerBase::ControllerState::RUNNING;
		if (sim_time.sim_time_ == ros::Time(0))
		{
			sim_time.sim_time_ = msg->header.stamp;
		}
		if (last_latency_compensation_state.header.stamp == ros::Time(0))
		{
			last_latency_compensation_state = *msg;
			return;
		}
		for (const talon_state_msgs::LatencyCompensationGroup &group : msg->latency_compensation_groups)
		{
			for (size_t i = 0; i < group.name.size(); i++)
			{
				robot.latency_compensation_state_.setEntry(group.name[i], group.stamp[i], group.value[i], group.slope[i]);
			}
		}
		sim_time += (msg->header.stamp - last_latency_compensation_state.header.stamp);
		swerve_controller.update(sim_time.sim_time_, sim_time.most_recent_duration_);
		last_latency_compensation_state = *msg;
	}

	ros::NodeHandle nh;
	SimTime sim_time;
	SwerveBot<4> robot;
	talon_swerve_drive_controller::TalonSwerveDriveController<4, hardware_interface::talonfxpro::TalonFXProCommandInterface> swerve_controller;
	talon_state_msgs::LatencyCompensationState last_latency_compensation_state;
	ros::Subscriber latency_compensation_state_sub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swervebot");
	SwerveBotFromBag s = SwerveBotFromBag();
	ros::spin(); // this line is important :)
	return 0;
}