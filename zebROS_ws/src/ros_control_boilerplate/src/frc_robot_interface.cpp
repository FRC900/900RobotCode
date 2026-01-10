/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
   Desc:   Helper ros_control hardware interface that loads configurations
*/
#include <ros/ros.h>
#include <ros_control_boilerplate/frc_robot_interface.h>
#ifdef __linux__
#include <pthread.h>                                  // for pthread_self
//#include <sched.h>                                    // for sched_get_prior...
//#include <cerrno>                                     // for errno
//#include <cstring>                                    // for size_t, strerror
#endif

#include <ctre/phoenix/unmanaged/Unmanaged.h>

#include "ros_control_boilerplate/ros_math_shared.hpp"

#include "ros_control_boilerplate/analog_input_devices.h"
#include "ros_control_boilerplate/as726x_devices.h"
#include "ros_control_boilerplate/can_bus_status_devices.h"
#include "ros_control_boilerplate/cancoder_devices.h"
#include "ros_control_boilerplate/candle_devices.h"
#include "ros_control_boilerplate/canifier_devices.h"
#include "ros_control_boilerplate/ctre_v5_motor_controllers.h"
#include "ros_control_boilerplate/digital_input_devices.h"
#include "ros_control_boilerplate/digital_output_devices.h"
#include "ros_control_boilerplate/double_solenoid_devices.h"
#include "ros_control_boilerplate/hoot_logger_devices.h"
#include "ros_control_boilerplate/joystick_devices.h"
#include "ros_control_boilerplate/latency_compensation_groups.h"
#include "ros_control_boilerplate/match_data_devices.h"
#include "ros_control_boilerplate/pcm_devices.h"
#include "ros_control_boilerplate/pdh_devices.h"
#include "ros_control_boilerplate/pdp_devices.h"
#include "ros_control_boilerplate/ph_devices.h"
#include "ros_control_boilerplate/pigeon2_devices.h"
#include "ros_control_boilerplate/pwm_devices.h"
#include "ros_control_boilerplate/ready_devices.h"
#include "ros_control_boilerplate/robot_controller_devices.h"
#include "ros_control_boilerplate/ros_iterative_robot_devices.h"
#include "ros_control_boilerplate/rumble_devices.h"
#include "ros_control_boilerplate/servo_devices.h"
#include "ros_control_boilerplate/solenoid_devices.h"
// #include "ros_control_boilerplate/sparkmax_devices.h"
#include "ros_control_boilerplate/talonfxpro_devices.h"
#include "ros_control_boilerplate/talon_orchestra_devices.h"

//PURPOSE: Stuff used by to run both hw and sim interfaces
namespace ros_control_boilerplate
{

template <bool SIM>
FRCRobotInterface<SIM>::FRCRobotInterface(void)
	: name_("generic_hw_interface")
{
}

template <bool SIM>
FRCRobotInterface<SIM>::~FRCRobotInterface() = default;


template <bool SIM>
void FRCRobotInterface<SIM>::readParams(const ros::NodeHandle& root_nh, const ros::NodeHandle &/*robot_hw_nh*/)
{
	ros::NodeHandle rpnh(root_nh, "hardware_interface"); // TODO(davetcoleman): change the namespace to "frc_robot_interface" aka name_
	run_hal_robot_ = rpnh.param<bool>("run_hal_robot", run_hal_robot_);
	can_interface_ = rpnh.param<std::string>("can_interface", can_interface_);
}

template<bool SIM>
std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> FRCRobotInterface<SIM>::get_ctrev6_devices() {
	// Grab a collection of all the ctre V6 device types
	std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> ctrev6_devices;

	auto append_device_map = [&ctrev6_devices, this]<typename T>(void)
	{
		const auto device_ptr = getDevicesOfType<T>(devices_);
		if (device_ptr)
		{
			device_ptr->appendDeviceMap(ctrev6_devices);
		}
	};
	append_device_map.template operator()<CANCoderDevices<SIM>>(); // C++ 20 templated lamba call syntax is dumb if there's no function parameter to deduce the types from
	append_device_map.template operator()<Pigeon2Devices>();  // and apparently even dumber if they're in a templated member function
	append_device_map.template operator()<TalonFXProDevices<SIM>>();
	return ctrev6_devices;
}

template <bool SIM>
bool FRCRobotInterface<SIM>::init(ros::NodeHandle& root_nh, ros::NodeHandle &/*robot_hw_nh*/)
{
	read_tracer_ = std::make_unique<Tracer>("FRCRobotInterface " + root_nh.getNamespace() + "::read()");
	write_tracer_ = std::make_unique<Tracer>("FRCRobotInterface " + root_nh.getNamespace() + "::write()");

#ifdef __linux__
#if 0
	struct sched_param schedParam{};

	schedParam.sched_priority = sched_get_priority_min(SCHED_RR);
	const auto rc = pthread_setschedparam(pthread_self(), SCHED_RR, &schedParam);
	if (rc)
	{
		ROS_WARN_STREAM("pthread_setschedparam() returned " << rc
				<< " priority = " << schedParam.sched_priority
				<< " errno = " << errno << " (" << strerror(errno) << ") : Run me as root?");
	}
	else
	{
		ROS_INFO_STREAM("pthread_setschedparam() succeeded");
	}
#endif
	if (pthread_setname_np(pthread_self(), "hwi_main_loop"))
	{
		ROS_ERROR_STREAM("Error setting thread name hwi_main_loop " << errno);
	}
#endif

	wpi::math::MathSharedStore::SetMathShared(std::make_unique<ROSMathShared>());

	if (run_hal_robot_)
	{
		// Make sure to initialize WPIlib code before creating
		// a CAN Talon object to avoid NIFPGA: Resource not initialized
		// errors? See https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
		devices_.emplace_back(std::make_unique<ROSIterativeRobotDevices>(root_nh));
		/**
		 * Calling this function will load and start
		 * the Phoenix background tasks.
		 *
		 * This can be useful if you need the
		 * Enable/Disable functionality for CAN devices
		 * but aren't using any of the CAN device classes.
		 **/
		ctre::phoenix::unmanaged::Unmanaged::LoadPhoenix();
		if constexpr (SIM)
		{
			// Only run Phoenix tuner server on the Jetson in sim, disable it here for the Rio
			ctre::phoenix::unmanaged::Unmanaged::SetPhoenixDiagnosticsStartTime(-1);
		}

	}
	else
	{
		if constexpr (!SIM)
		{
			// Only run Phoenix tuner server on the Rio, disable it here for the Jetson
			ctre::phoenix::unmanaged::Unmanaged::SetPhoenixDiagnosticsStartTime(-1);
		}
	}
	ROS_INFO_STREAM("Phoenix Version String : " << ctre::phoenix::unmanaged::Unmanaged::GetPhoenixVersion());
	Devices::setHALRobot(run_hal_robot_);
	Devices::setRobotHW(this);

	// Create all the devices specified in the yaml joint list, one type at a time
	// Those that need different code for sim vs real hardware are templated using
	// the SIM template param for this class
	devices_.emplace_back(std::make_unique<AnalogInputDevices>(root_nh));
	devices_.emplace_back(std::make_unique<AS726xDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<CANBusStatusDevices>(root_nh));
	devices_.emplace_back(std::make_unique<CANCoderDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<CANdleDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<CANifierDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<CTREV5MotorControllers<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<DigitalInputDevices>(root_nh));
	devices_.emplace_back(std::make_unique<DigitalOutputDevices>(root_nh));
	devices_.emplace_back(std::make_unique<DoubleSolenoidDevices>(root_nh));
	devices_.emplace_back(std::make_unique<HootLoggerDevices>(root_nh));
	devices_.emplace_back(std::make_unique<JoystickDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<MatchDataDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<PCMDevices>(root_nh));
	devices_.emplace_back(std::make_unique<PDHDevices>(root_nh));
	devices_.emplace_back(std::make_unique<PDPDevices>(root_nh));
	devices_.emplace_back(std::make_unique<PHDevices>(root_nh));
	devices_.emplace_back(std::make_unique<Pigeon2Devices>(root_nh));
	devices_.emplace_back(std::make_unique<PWMDevices>(root_nh));
	devices_.emplace_back(std::make_unique<ReadyDevices>(root_nh));
	if (run_hal_robot_)
	{
		devices_.emplace_back(std::make_unique<RobotControllerDevices>(root_nh));
	}
	devices_.emplace_back(std::make_unique<RumbleDevices>(root_nh));
	devices_.emplace_back(std::make_unique<ServoDevices>(root_nh));
	devices_.emplace_back(std::make_unique<SolenoidDevices>(root_nh));
	// devices_.emplace_back(std::make_unique<SparkMaxDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<TalonFXProDevices<SIM>>(root_nh));
	devices_.emplace_back(std::make_unique<TalonOrchestraDevices<SIM>>(root_nh));

	std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> ctrev6_devices = get_ctrev6_devices();
	devices_.emplace_back(std::make_unique<LatencyCompensationGroups>(root_nh, ctrev6_devices));

	// Create controller interfaces for all the types created above
	for (const auto &d : devices_)
	{
		auto i = d->registerInterface();
		if (i)
		{
			registerInterfaceManager(i);
		}
	}

	// Orchestra needs a set of previously created TalonFXs to use as instruments
	const auto orchestra_devices = getDevicesOfType<TalonOrchestraDevices<SIM>>(devices_);
	const auto talonfxpro_devices = getDevicesOfType<TalonFXProDevices<SIM>>(devices_);
	if (talonfxpro_devices && orchestra_devices)
	{
		std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> talonfxs;
		talonfxpro_devices->appendDeviceMap(talonfxs);
		orchestra_devices->setTalonFXData(talonfxs);
	}

	ROS_INFO_STREAM_NAMED("frc_robot_interface", "FRCRobotInterface Ready on " << root_nh.getNamespace());

	return true;
}

template <bool SIM>
void FRCRobotInterface<SIM>::read(const ros::Time &time, const ros::Duration &period)
{
	read_tracer_->start_unique("Check for ready");
	if (!robot_code_ready_)
	{
		const auto ready_device = getDevicesOfType<ReadyDevices>(devices_);
		// This will be written by the last controller to be
		// spawned - waiting here prevents the robot from
		// reporting robot code ready to the field until
		// all other controllers are started
		if (ready_device && ready_device->areReady())
		{
			ROS_INFO_STREAM("Robot is ready");
			robot_code_ready_ = true;

			Devices::signalReady();
		}
	}

	for (const auto &d: devices_)
	{
		d->read(time, period, *read_tracer_);
	}
	read_tracer_->report(60);
}

template <bool SIM>
void FRCRobotInterface<SIM>::write(const ros::Time& time, const ros::Duration& period)
{
	for (const auto &d: devices_)
	{
		d->write(time, period, *write_tracer_);
	}

	write_tracer_->report(60);
}

template <bool SIM>
bool FRCRobotInterface<SIM>::prepareSwitch(const std::list<hardware_interface::ControllerInfo> & /*start_list*/,
										   const std::list<hardware_interface::ControllerInfo> & /*stop_list*/)
{
	return true;
}

}  // namespace