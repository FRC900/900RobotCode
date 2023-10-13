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
#include <memory> // for make_unique()
#include <ros/ros.h>

#include "hal/HALBase.h"
#include "../sim/HALInitializer.h"

#include "ros_control_boilerplate/frcrobot_sim_interface.h"

#include "ros_control_boilerplate/as726x_devices.h"
#include "ros_control_boilerplate/canifier_devices.h"
#include "ros_control_boilerplate/joystick_devices.h"
#include "ros_control_boilerplate/match_data_devices.h"
#include "ros_control_boilerplate/sparkmax_devices.h"
#include "ros_control_boilerplate/talon_orchestra_devices.h"
#include "ros_control_boilerplate/talonfxpro_devices.h"

namespace ros_control_boilerplate
{

FRCRobotSimInterface::FRCRobotSimInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
}
FRCRobotSimInterface::~FRCRobotSimInterface() = default;

bool FRCRobotSimInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " line: " << __LINE__);

	if (!FRCRobotInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " base class init() failed");
		return false;
	}

	// Create devices which have different code for HW vs. Sim
	// (hw interface has a similar block, but using HW vs Sim devices)
	devices_.emplace_back(std::make_shared<SimAS726xDevices>(root_nh));
	devices_.emplace_back(std::make_shared<SimCANifierDevices>(root_nh));
    devices_.emplace_back(std::make_shared<SimJoystickDevices>(root_nh));
	devices_.emplace_back(std::make_shared<SimMatchDataDevices>(root_nh));
	devices_.emplace_back(std::make_shared<SimSparkMaxDevices>(root_nh));
	devices_.emplace_back(std::make_shared<SimTalonOrchestraDevices>(root_nh));

	// Orchestra needs a set of previously created TalonFXs to use as instruments
	const auto orchestra_devices = getDevicesOfType<SimTalonOrchestraDevices>(devices_);
    const auto talonfxpro_devices = getDevicesOfType<TalonFXProDevices>(devices_);
	if (talonfxpro_devices && orchestra_devices)
	{
		std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> talonfxs;
		talonfxpro_devices->appendDeviceMap(talonfxs);
		orchestra_devices->setTalonFXData(talonfxs);
	}

	for (auto &d : devices_)
	{
		d->simInit(root_nh);
	}
	// Need to do this here rather than in the base class so
	// registerInterface isn't called twice for devices
	// created in the base init().
	for (auto &d : devices_)
	{
		registerInterfaceManager(d->registerInterface());
	}
	hal::init::InitializeDriverStationData();

	ROS_INFO_STREAM_NAMED("frcrobot_sim_interface", "FRCRobotSimInterface Ready on " << root_nh.getNamespace());
	return true;
}

void FRCRobotSimInterface::read(const ros::Time& time, const ros::Duration& period)
{
	// Run WPIlib physics sim for each mechanism.
	// Right now those mechanisms are hard-coded, but in the future make
	// them configurable via config file?
	// This could happen at the end of write() or beginning of read(),
	// shouldn't matter which just so long as the sim mechanisms are
	// updated once per control loop using the appropriate timestep

	read_tracer_.start_unique("HAL_SimPeriodicBefore");
	HAL_SimPeriodicBefore();

	for (auto &d : devices_)
	{
		d->simRead(time, period, read_tracer_);
	}

	read_tracer_.start_unique("HAL_SimPeriodicAfter");
	HAL_SimPeriodicAfter();

	FRCRobotInterface::read(time, period);
}

template <class T>
static bool read_device_enabled(const std::vector<std::shared_ptr<Devices>> &devices_, bool &val)
{
	const auto devices = getDevicesOfType<T>(devices_);
	if (devices)
	{
		const auto isEnabled = devices->isEnabled();
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
	write_tracer_.start_unique("read robot enabled");
	bool robot_enabled = false;
	if (read_device_enabled<SimMatchDataDevices>(devices_, robot_enabled))
	{
		Devices::setEnabled(robot_enabled);
	}

	for (auto &d: devices_)
	{
		d->simWrite(time, period, write_tracer_);
	}
	FRCRobotInterface::write(time, period);
}

}  // namespace
