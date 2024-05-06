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
#include <ros/ros.h>

#include "hal/HALBase.h"
#include "../sim/HALInitializer.h"

#include "ros_control_boilerplate/devices.h"
#include "ros_control_boilerplate/frcrobot_sim_interface.h"
#include "ros_control_boilerplate/match_data_devices.h"

namespace ros_control_boilerplate
{

FRCRobotSimInterface::FRCRobotSimInterface()
	: ros_control_boilerplate::FRCRobotInterface<true>()
{
}
FRCRobotSimInterface::~FRCRobotSimInterface() = default;

bool FRCRobotSimInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	ROS_INFO_STREAM("root_nh.getNamespace() = " << root_nh.getNamespace());
	FRCRobotInterface::readParams(root_nh, robot_hw_nh);
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " line: " << __LINE__);

	if (!FRCRobotInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " base class init() failed");
		return false;
	}

	for (const auto &d : devices_)
	{
		d->simInit(root_nh);
	}
	hal::init::InitializeDriverStationData();

	ROS_INFO_STREAM_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready on " << root_nh.getNamespace());
	return true;
}

void FRCRobotSimInterface::read(const ros::Time &time, const ros::Duration &period)
{
	// Run WPIlib physics sim for each mechanism.
	// Right now those mechanisms are hard-coded, but in the future make
	// them configurable via config file?
	// This could happen at the end of write() or beginning of read(),
	// shouldn't matter which just so long as the sim mechanisms are
	// updated once per control loop using the appropriate timestep

	read_tracer_->start_unique("HAL_SimPeriodicBefore");
	HAL_SimPeriodicBefore();

	for (const auto &d : devices_)
	{
		d->simPreRead(time, period, *read_tracer_);
	}

	read_tracer_->start_unique("HAL_SimPeriodicAfter");
	HAL_SimPeriodicAfter();

	FRCRobotInterface::read(time, period);
	for (const auto &d : devices_)
	{
		d->simPostRead(time, period, *read_tracer_);
	}

}

template <class T>
static bool read_device_enabled(const std::vector<std::unique_ptr<Devices>> &devices, bool &val)
{
	if (const auto d = getDevicesOfType<T>(devices))
	{
		const auto isEnabled = d->isEnabled();
		if (isEnabled)
		{
			val = *isEnabled;
			return true;
		}
	}
	return false;
}

void FRCRobotSimInterface::write(const ros::Time& time, const ros::Duration& period)
{
	// Was the robot enabled last time write was run?
	write_tracer_->start_unique("read robot enabled");
	if (bool robot_enabled = false; read_device_enabled<MatchDataDevices<true>>(devices_, robot_enabled))
	{
		Devices::setEnabled(robot_enabled);
	}

	for (const auto &d: devices_)
	{
		d->simWrite(time, period, *write_tracer_);
	}
	FRCRobotInterface::write(time, period);
}

bool FRCRobotSimInterface::gazeboSimInit(const ros::NodeHandle &model_nh, boost::shared_ptr<gazebo::physics::Model> parent_model)
{
	for (auto const &d : devices_)
	{
		d->gazeboSimInit(model_nh, parent_model);
	}
	return true;
}

}  // namespace
