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

	The hardware interface code reads and writes directly from/to hardware
	connected to the RoboRIO. This include DIO, Analog In, pneumatics,
	and CAN Talons, among other things.

	The two main methods are read() and write().

	read() is responsible for reading hardware state and filling in
	a buffered copy of it. This buffered copy of the hardware state
	can be accessed by various controllers to figure out what to do next.

	write() does the opposite. It takes commands that have been buffered
	by various controllers and sends them to the hardware.  The design goal
	here is to minimize redundant writes to the HW.  Previous values written
	are cached, and subsequent writes of the same value are skipped.

	The main read loop actually reads from all hardware except CAN Talons.
	The CAN talon status reads are double buffered. A thread is kicked
	off for each CAN talon.  That thread updates a buffer which is shared
	by the main read loop. The only thing the main read loop does is
	consolidate the data from each thread into a separate state buffer,
	this one externally visible to controllers.  Since reads are the slowest
	part of the process, this decouples hardware read speed from the
	control loop update rate.

	The PDP data also works in a similar way.  There is a thread running
	at a constant rate polling PDP data, and read() picks up the latest
	copy of that data each time through the read/update/write loop
*/

//PURPOSE: File that reads and writes to hardware

#include "ros_control_boilerplate/frcrobot_hw_interface.h"
#include "ros_control_boilerplate/error_queue.h"

//HAL / wpilib includes
#include <HALInitializer.h>
#include <hal/DriverStation.h>

#include "ros_control_boilerplate/devices.h"
#include "ros_control_boilerplate/match_data_devices.h"

extern "C" { void HALSIM_SetControlWord(HAL_ControlWord); }
void HAL_SetCANBusString(const std::string &bus);
//
// digital output, PWM, Pneumatics, compressor, nidec, talons
//    controller on jetson  (local update = true, local hardware = false
//        don't do anything in read
//        random controller updates command in controller
//        set output state var from command in write() on jetson - this will be reflected in joint_states
//          but do not call Set since hardware doesn't exist (local write)
//
//      on rio (local update = false, local hardware = true
//         don't do anything in read
//         update loop needs to read joint_states using joint state listener
//             this writes values from the jetson to each local joint command on the Rio
//         write() sets hardware from those joint commands, and also sets state
//            write needs to set value as - is, don't apply invert,
//            since it was already applied on the remote side
//
//	local_update = true, local hardware = true -> no listener
//
//		This would be for hardware on the Rio which is also modified by controllers running on the Rio
//
//	local_update = false, local hardware = true -> listener to transfer cmd from remote to local
//
//		E.g. config on the Rio if a controller on the Jetson wanted to update hardware on the Rio
//
//	local_update = true, local_hardware = false -> no listener, update local state but don't write to hw
//
//		e.g. config on the Jetson if a controller on the Jetson wanted to update hardware on the rio
//
//	local_update = false, local_hardware = false -> listener to mirror updated state from local?
//
//		nothing is happening on the controller wrt the hardware other than wanting to keep current on status
//		not sure how useful this might be, except in cases like digital in where update==hardware
//		by definition
//
//	So !local_update implies add to remote Interface to run a listener
//
// For analog & digital input and state like PDP, match, joystick, etc, there's only 1 local flag.
// The only cases which make sense are local_update = local_hardware, since the value can only be
// updated by reading the hardware itself.  There, just use a "local" flag.
//
namespace ros_control_boilerplate
{
// Constructor. Pass appropriate params to base class constructor,
// initialze robot_ pointer to NULL
FRCRobotHWInterface::FRCRobotHWInterface()
	: ros_control_boilerplate::FRCRobotInterface<false>() // false == not sim, meaning real hardware
{
}

bool FRCRobotHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
	FRCRobotInterface::readParams(root_nh, robot_hw_nh);
	if (!run_hal_robot_)
	{
		errorQueue = std::make_unique<ErrorQueue>();
		// This is for non Rio-based robots.  Call init for the wpilib HAL code we've "borrowed" before using them
		hal::init::InitializeCANAPI();
		hal::init::InitializeCTREPCM();
		hal::init::InitializeCTREPDP();
		hal::init::InitializeREVPDH();
		hal::init::InitializeREVPH();
	}
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	if (!FRCRobotInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": FRCRobotInterface::init() returnd false");
		return false;
	}

	if (run_hal_robot_)
	{
		ds_error_server_ = robot_hw_nh.advertiseService("/frcrobot_rio/ds_error_service", &FRCRobotHWInterface::DSErrorCallback, this);
	}
	else
	{
		// Use to pass the can_interface_ string into various CAN shim calls
		// so we end up talking to the correct can bus for e.g. PCM and PDH access
		HAL_SetCANBusString(can_interface_);
	}

	// Handle any init specifically needed for hardware
	for (const auto &d : devices_)
	{
		d->hwInit(root_nh);
	}

	ROS_INFO_STREAM(robot_hw_nh.getNamespace() << " : FRCRobotHWInterface Ready on " << root_nh.getNamespace());
	HAL_SendError(true, 0, false, std::string("(Not an error) " + robot_hw_nh.getNamespace() + " : FRCRobotHWInterface Ready on " + root_nh.getNamespace()).c_str(), "", "", true);
	return true;
}

void FRCRobotHWInterface::read(const ros::Time& time, const ros::Duration& period)
{
	for (const auto &d : devices_)
	{
		d->hwRead(time, period, *read_tracer_);
	}
	FRCRobotInterface::read(time, period);
}

template <class T>
static bool read_control_word(const std::vector<std::unique_ptr<Devices>> &devices, HAL_ControlWord &cw)
{
	const auto d = getDevicesOfType<T>(devices);
	return d && d->getControlWord(cw);
}

//#define DEBUG_WRITE
void FRCRobotHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
	// This is used to set a variable which is queried by the Jetson/x86
	// frc::Driverstation stub functions.  Using data from the Rio &
	// real driverstation, it exports robot state to a variable that is then
	// used by calls to a faked frc::DriverStation::IsEnabled() ... set of
	// functions. These functions are used internally by other wpilib code
	// pulled into Jetson and x86 builds, so this lets those calls get the
	// correct value for current robot states
	// For the Rio, the HALSIM_SetControlWord() call does nothing, since in
	// that case the real frc::DriverStation code is used which is actually
	// hooked up directly to the real driver station.
	if (HAL_ControlWord cw; read_control_word<MatchDataDevices<false>>(devices_, cw))
	{
		HALSIM_SetControlWord(cw);

		// TODO : For spark max - move to frc_robot_interface if possible
		// Should be able to make robot_enabled_ a protected member var and reuse it here
		// after setting it in FRCRobotInterface::write();
		Devices::setEnabled(cw.enabled);
	}

	for (const auto &d : devices_)
	{
		d->hwWrite(time, period, *write_tracer_);
	}
	FRCRobotInterface::write(time, period);
}

bool FRCRobotHWInterface::DSErrorCallback(ros_control_boilerplate::DSError::Request &req, ros_control_boilerplate::DSError::Response &/*res*/)
{
	ROS_ERROR_STREAM("HWI received DSErrorCallback " << req.details.c_str());
	HAL_SendError(true, req.error_code, false, req.details.c_str(), "", "", true);
	return true;
}

} // namespace
