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

#include <iostream>

#include "ros_control_boilerplate/frcrobot_hw_interface.h"
#include "ros_control_boilerplate/error_queue.h"

//HAL / wpilib includes
#include <HALInitializer.h>
#include <hal/DriverStation.h>

#include <ctre/phoenix/platform/Platform.h>           // for SetCANInterface
//#include <ctre/phoenix/cci/Unmanaged_CCI.h>

#ifdef __linux__
#include <sched.h>
#endif

extern "C" { void HALSIM_SetControlWord(HAL_ControlWord); }
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
//	local_update = true, local_hardare = false -> no listener, update local state but don't write to hw
//
//		e.g. config on the Jetson if a controller on the Jetson wanted to update hardware on the rio
//
//	local_update = false, local_hardare = false -> listener to mirror updated state from local?
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
FRCRobotHWInterface::FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
{
}

// Clean up whatever we've created in init()
FRCRobotHWInterface::~FRCRobotHWInterface()
{
	for (size_t i = 0; i < num_as726xs_; i++)
		as726x_thread_[i].join();
	for (size_t i = 0; i < num_cancoders_; i++)
		cancoder_read_threads_[i].join();
	for (size_t i = 0; i < num_canifiers_; i++)
		canifier_read_threads_[i].join();
}

bool FRCRobotHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	if (run_hal_robot_)
	{
		ds_error_server_ = robot_hw_nh.advertiseService("/frcrobot_rio/ds_error_service", &FRCRobotHWInterface::DSErrorCallback, this);
	}
	else
	{
		// This is for non Rio-based robots.  Call init for the wpilib HAL code we've "borrowed" before using them
		hal::init::InitializeCANAPI();
		hal::init::InitializeCompressor();
		hal::init::InitializePCMInternal();
		hal::init::InitializePDP();
		hal::init::InitializeSolenoid();

		errorQueue = std::make_unique<ErrorQueue>();
		const auto rc = ctre::phoenix::platform::can::SetCANInterface(can_interface_.c_str());
		if (rc != 0)
		{
			HAL_SendError(true, -1, false, "SetCANInterface failed - likely CAN adapter failure", "", "", true);
		}
	}

	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	if (!FRCRobotInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": FRCRobotInterface::init() returnd false");
		return false;
	}

	for (size_t i = 0; i < num_canifiers_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << canifier_names_[i] <<
							  (canifier_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (canifier_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << canifier_can_ids_[i]);

		if (canifier_local_hardwares_[i])
		{
			canifiers_.emplace_back(std::make_shared<ctre::phoenix::CANifier>(canifier_can_ids_[i]));
			canifier_read_state_mutexes_.emplace_back(std::make_shared<std::mutex>());
			canifier_read_thread_states_.emplace_back(std::make_shared<hardware_interface::canifier::CANifierHWState>(canifier_can_ids_[i]));
			canifier_read_threads_.emplace_back(std::thread(&FRCRobotHWInterface::canifier_read_thread, this,
												canifiers_[i], canifier_read_thread_states_[i],
												canifier_read_state_mutexes_[i],
												std::make_unique<Tracer>("canifier_read_" + canifier_names_[i] + " " + root_nh.getNamespace())));
		}
		else
		{
			canifiers_.push_back(nullptr);
			canifier_read_state_mutexes_.push_back(nullptr);
			canifier_read_thread_states_.push_back(nullptr);
		}
	}
	for (size_t i = 0; i < num_cancoders_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << cancoder_names_[i] <<
							  (cancoder_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (cancoder_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << cancoder_can_ids_[i]);

		if (cancoder_local_hardwares_[i])
		{
			cancoders_.emplace_back(std::make_shared<ctre::phoenix::sensors::CANCoder>(cancoder_can_ids_[i]));
			cancoder_read_state_mutexes_.emplace_back(std::make_shared<std::mutex>());
			cancoder_read_thread_states_.emplace_back(std::make_shared<hardware_interface::cancoder::CANCoderHWState>(cancoder_can_ids_[i]));
			cancoder_read_threads_.emplace_back(std::thread(&FRCRobotHWInterface::cancoder_read_thread, this,
												cancoders_[i], cancoder_read_thread_states_[i],
												cancoder_read_state_mutexes_[i],
												std::make_unique<Tracer>("cancoder_read_" + cancoder_names_[i] + " " + root_nh.getNamespace())));
		}
		else
		{
			cancoders_.push_back(nullptr);
			cancoder_read_state_mutexes_.push_back(nullptr);
			cancoder_read_thread_states_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_as726xs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading as726x joint " << i << "=" << as726x_names_[i] <<
							  (as726x_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (as726x_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as as726x with port=" << as726x_ports_[i] <<
							  " address=" << as726x_addresses_[i]);
		if (as726x_local_hardwares_[i])
		{
			frc::I2C::Port port;
			if (as726x_ports_[i] == "onboard")
				port = frc::I2C::Port::kOnboard;
			else if (as726x_ports_[i] == "mxp")
				port = frc::I2C::Port::kMXP;
			else
			{
				// Allow arbitrary integer ports to open /dev/i2c-<number> devices
				// on the Jetson or other linux platforms
				try
				{
					port = static_cast<frc::I2C::Port>(std::stoi(as726x_ports_[i]));
				}
				catch(...)
				{
					ROS_ERROR_STREAM("Invalid port specified for as726x - " <<
							as726x_ports_[i] << "valid options are onboard, mxp, or a number");
					return false;
				}
			}

			as726xs_.emplace_back(std::make_shared<as726x::roboRIO_AS726x>(port, as726x_addresses_[i]));
			if (as726xs_.back()->begin())
			{
					as726x_read_thread_state_.emplace_back(std::make_shared<hardware_interface::as726x::AS726xState>(as726x_ports_[i], as726x_addresses_[i]));
					as726x_read_thread_mutexes_.emplace_back(std::make_shared<std::mutex>());
					as726x_thread_.emplace_back(std::thread(&FRCRobotHWInterface::as726x_read_thread, this,
												as726xs_[i],
												as726x_read_thread_state_[i],
												as726x_read_thread_mutexes_[i],
												std::make_unique<Tracer>("AS726x:" + as726x_names_[i] + " " + root_nh.getNamespace())));
			}
			else
			{
				ROS_ERROR_STREAM("Error intializing as726x");
				as726xs_.push_back(nullptr);
				as726x_read_thread_state_.push_back(nullptr);
				as726x_read_thread_mutexes_.push_back(nullptr);
			}
		}
		else
		{
			as726xs_.push_back(nullptr);
			as726x_read_thread_state_.push_back(nullptr);
			as726x_read_thread_mutexes_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_talon_orchestras_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << talon_orchestra_names_[i]);

                talon_orchestras_.push_back(std::make_shared<ctre::phoenix::music::Orchestra>());
	}

	ROS_INFO_STREAM(robot_hw_nh.getNamespace() << " : FRCRobotHWInterface Ready.");
	HAL_SendError(true, 0, false, std::string("(Not an error) " + robot_hw_nh.getNamespace() + " : FRCRobotHWInterface Ready").c_str(), "", "", true);
	return true;
}

// Each canifier gets their own read thread. The thread loops at a fixed rate
// reading all state from that canifier. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotHWInterface::canifier_read_thread(std::shared_ptr<ctre::phoenix::CANifier> canifier,
											std::shared_ptr<hardware_interface::canifier::CANifierHWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "canifier_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate rate(100); // TODO : configure me from a file or
						 // be smart enough to run at the rate of the fastest status update?

	while(ros::ok())
	{
		tracer->start("canifier read main_loop");

		int encoder_ticks_per_rotation;
		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the state here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*mutex);
			encoder_ticks_per_rotation = state->getEncoderTicksPerRotation();
			conversion_factor = state->getConversionFactor();
		}

		std::array<bool, hardware_interface::canifier::GeneralPin::GeneralPin_LAST> general_pins;
		for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
		{
			ctre::phoenix::CANifier::GeneralPin ctre_general_pin;
			if (canifier_convert_.generalPin(static_cast<hardware_interface::canifier::GeneralPin>(i), ctre_general_pin))
				general_pins[i] = canifier->GetGeneralInput(ctre_general_pin);
		}

		// Use FeedbackDevice_QuadEncoder to force getConversionFactor to use the encoder_ticks_per_rotation
		// variable to calculate these values
		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Velocity) * conversion_factor;
		const double quadrature_position = canifier->GetQuadraturePosition() * radians_scale;
		const double quadrature_velocity = canifier->GetQuadratureVelocity() * radians_per_second_scale;
		const double bus_voltage      = canifier->GetBusVoltage();

		std::array<std::array<double, 2>, hardware_interface::canifier::PWMChannel::PWMChannelLast> pwm_inputs;
		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
			if (canifier_convert_.PWMChannel(static_cast<hardware_interface::canifier::PWMChannel>(i), ctre_pwm_channel))
				canifier->GetPWMInput(ctre_pwm_channel, &pwm_inputs[i][0]);
		}

		ctre::phoenix::CANifierFaults ctre_faults;
		canifier->GetFaults(ctre_faults);
		const unsigned faults = ctre_faults.ToBitfield();
		ctre::phoenix::CANifierStickyFaults ctre_sticky_faults;
		canifier->GetStickyFaults(ctre_sticky_faults);
		const unsigned sticky_faults = ctre_sticky_faults.ToBitfield();

		// Actually update the CANifierHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);

			for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
			{
				state->setGeneralPinInput(static_cast<hardware_interface::canifier::GeneralPin>(i), general_pins[i]);
			}

			state->setQuadraturePosition(quadrature_position);
			state->setQuadratureVelocity(quadrature_velocity);
			state->setBusVoltage(bus_voltage);

			for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
			{
				state->setPWMInput(static_cast<hardware_interface::canifier::PWMChannel>(i), pwm_inputs[i]);
			}

			state->setFaults(faults);
			state->setStickyFaults(sticky_faults);
		}
		tracer->report(60);
		rate.sleep();
	}
}

// Each cancoder gets their own read thread. The thread loops at a fixed rate
// reading all state from that cancoder. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotHWInterface::cancoder_read_thread(std::shared_ptr<ctre::phoenix::sensors::CANCoder> cancoder,
											std::shared_ptr<hardware_interface::cancoder::CANCoderHWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "cancoder_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate rate(100); // TODO : configure me from a file or
						 // be smart enough to run at the rate of the fastest status update?

	while(ros::ok())
	{
		tracer->start("cancoder read main_loop");

		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the state here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*mutex);
			conversion_factor = state->getConversionFactor();
		}
		//TODO redo using feedback coefficent

		// Use FeedbackDevice_QuadEncoder to force getConversionFactor to use the encoder_ticks_per_rotation
		// variable to calculate these values
		const double position = cancoder->GetPosition() * conversion_factor;
		const double velocity = cancoder->GetVelocity() * conversion_factor;
		const double absolute_position = cancoder->GetAbsolutePosition() * conversion_factor;
		const double bus_voltage = cancoder->GetBusVoltage();
		const auto ctre_magnet_field_strength = cancoder->GetMagnetFieldStrength();
		hardware_interface::cancoder::MagnetFieldStrength magnet_field_strength;
		cancoder_convert_.magnetFieldStrength(ctre_magnet_field_strength, magnet_field_strength);
		const double last_timestamp = cancoder->GetLastTimestamp();
		const int firmware_version = cancoder->GetFirmwareVersion();

		ctre::phoenix::sensors::CANCoderFaults ctre_faults;
		cancoder->GetFaults(ctre_faults);
		const unsigned faults = ctre_faults.ToBitfield();
		ctre::phoenix::sensors::CANCoderStickyFaults ctre_sticky_faults;
		cancoder->GetStickyFaults(ctre_sticky_faults);
		const unsigned sticky_faults = ctre_sticky_faults.ToBitfield();

		// Actually update the CANCoderHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);
			state->setPosition(position);
			state->setVelocity(velocity);
			state->setAbsolutePosition(absolute_position);
			state->setBusVoltage(bus_voltage);
			state->setMagnetFieldStrength(magnet_field_strength);
			state->setLastTimestamp(last_timestamp);
			state->setFirmwareVersion(firmware_version);
			state->setFaults(faults);
			state->setStickyFaults(sticky_faults);
		}
		tracer->report(60);
		rate.sleep();
	}
}

// The AS726x state reads happen in their own thread. This thread
// loops at XHz to match the update rate of PCM CAN
// status messages.  Each iteration, data read from the
// AS726x color sensor is copied to a state buffer shared with the main read
// thread.
void FRCRobotHWInterface::as726x_read_thread(
		std::shared_ptr<as726x::roboRIO_AS726x> as726x,
		std::shared_ptr<hardware_interface::as726x::AS726xState> state,
		std::shared_ptr<std::mutex> mutex,
		std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "as726x_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let I2C start up
	ros::Rate r(7); // TODO : Tune me?

	uint16_t temperature;
	std::array<uint16_t, 6> raw_channel_data;
	std::array<float, 6> calibrated_channel_data;
	while (ros::ok())
	{
		tracer->start("main loop");
		// Create a scope for the lock protecting hardware accesses
		{
			std::lock_guard<std::mutex> l(*(as726x->getMutex()));
			temperature = as726x->readTemperature();
			as726x->startMeasurement();
			auto data_ready_start = ros::Time::now();
			bool timeout = false;
			while (!as726x->dataReady() && !timeout)
			{
				ros::Duration(0.01).sleep();
				if ((ros::Time::now() - data_ready_start).toSec() > 1.5)
				{
					timeout = true;
				}
			}
			if (timeout)
			{
				ROS_WARN("Timeout waiting for as726 data ready");
				tracer->stop();
				continue;
			}

			as726x->readRawValues(&raw_channel_data[0], 6);
			as726x->readCalibratedValues(&calibrated_channel_data[0], 6);
		}
		// Create a new scope for the lock protecting the shared state
		{
			std::lock_guard<std::mutex> l(*mutex);
			state->setTemperature(temperature);
			state->setRawChannelData(raw_channel_data);
			state->setCalibratedChannelData(calibrated_channel_data);
		}
		tracer->report(60);
		ros::Duration(0.1).sleep(); // allow time for write() to run
		r.sleep();
	}
}

void FRCRobotHWInterface::read(const ros::Time& time, const ros::Duration& period)
{
	FRCRobotInterface::read(time, period);

	read_tracer_.start_unique("canifier");
	for (size_t joint_id = 0; joint_id < num_canifiers_; ++joint_id)
	{
		if (canifier_local_hardwares_[joint_id])
		{
			std::lock_guard<std::mutex> l(*canifier_read_state_mutexes_[joint_id]);
			auto &cs   = canifier_state_[joint_id];
			auto &crts = canifier_read_thread_states_[joint_id];

			// These are used to convert position and velocity units - make sure the
			// read thread's local copy of state is kept up to date
			crts->setEncoderTicksPerRotation(cs.getEncoderTicksPerRotation());
			crts->setConversionFactor(cs.getConversionFactor());

			for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
			{
				const auto pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
				cs.setGeneralPinInput(pin, crts->getGeneralPinInput(pin));
			}
			cs.setQuadraturePosition(crts->getQuadraturePosition());
			cs.setQuadratureVelocity(crts->getQuadratureVelocity());
			cs.setBusVoltage(crts->getBusVoltage());

			for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
			{
				const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
				cs.setPWMInput(pwm_channel, crts->getPWMInput(pwm_channel));
			}
			cs.setFaults(crts->getFaults());
			cs.setStickyFaults(crts->getStickyFaults());
		}
	}

	read_tracer_.start_unique("cancoder");
	for (size_t joint_id = 0; joint_id < num_cancoders_; ++joint_id)
	{
		if (cancoder_local_hardwares_[joint_id])
		{
			std::lock_guard<std::mutex> l(*cancoder_read_state_mutexes_[joint_id]);
			auto &cs   = cancoder_state_[joint_id];
			auto &crts = cancoder_read_thread_states_[joint_id];

			// These are used to convert position and velocity units - make sure the
			// read thread's local copy of state is kept up to date
			crts->setConversionFactor(cs.getConversionFactor());

			cs.setPosition(crts->getPosition());
			cs.setVelocity(crts->getVelocity());
			cs.setAbsolutePosition(crts->getAbsolutePosition());
			cs.setBusVoltage(crts->getBusVoltage());
			cs.setMagnetFieldStrength(crts->getMagnetFieldStrength());
			cs.setLastTimestamp(crts->getLastTimestamp());
			cs.setFirmwareVersion(crts->getFirmwareVersion());
			cs.setFaults(crts->getFaults());
			cs.setStickyFaults(crts->getStickyFaults());
		}
	}

	read_tracer_.start_unique("as726xs");
	for (size_t i = 0; i < num_as726xs_; i++)
	{
		if (as726x_local_updates_[i])
		{
			std::lock_guard<std::mutex> l(*as726x_read_thread_mutexes_[i]);
			as726x_state_[i].setTemperature(as726x_read_thread_state_[i]->getTemperature());
			as726x_state_[i].setRawChannelData(as726x_read_thread_state_[i]->getRawChannelData());
			as726x_state_[i].setCalibratedChannelData(as726x_read_thread_state_[i]->getCalibratedChannelData());
		}
	}

	read_tracer_.start_unique("talon orchestras");
	for(size_t i = 0; i < num_talon_orchestras_; i++)
	{
		if(talon_orchestras_[i]->IsPlaying())
		{
			orchestra_state_[i].setPlaying();
		}
		else
		{
			orchestra_state_[i].setStopped();
		}
	}
	read_tracer_.stop();
}

//#define DEBUG_WRITE
void FRCRobotHWInterface::write(const ros::Time& time, const ros::Duration& period)
{
	FRCRobotInterface::write(time, period);

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
	{
		std::unique_lock<std::mutex> l(match_data_mutex_, std::try_to_lock);
		if (l.owns_lock())
		{
			HAL_ControlWord cw;
			cw.enabled = match_data_.isEnabled();
			cw.autonomous = match_data_.isAutonomous();
			cw.test = match_data_.isTest();
			cw.eStop = match_data_.isEStopped();
			cw.fmsAttached = match_data_.isFMSAttached();
			cw.dsAttached = match_data_.isDSAttached();
			HALSIM_SetControlWord(cw);
		}
	}

	for (size_t joint_id = 0; joint_id < num_canifiers_; ++joint_id)
	{
		if (!canifier_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &canifier = canifiers_[joint_id];
		auto &cs = canifier_state_[joint_id];
		auto &cc = canifier_command_[joint_id];

		if (canifier->HasResetOccurred())
		{
			for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
				cc.resetLEDOutput(static_cast<hardware_interface::canifier::LEDChannel>(i));
			for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
				cc.resetGeneralPinOutput(static_cast<hardware_interface::canifier::GeneralPin>(i));
			cc.resetQuadraturePosition();
			cc.resetVelocityMeasurementPeriod();
			cc.resetVelocityMeasurementWindow();
			cc.resetClearPositionOnLimitF();
			cc.resetClearPositionOnLimitR();
			cc.resetClearPositionOnQuadIdx();
			for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
			{
				const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
				cc.resetPWMOutput(pwm_channel);
				cc.resetPWMOutputEnable(pwm_channel);
			}
			for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
				cc.resetStatusFramePeriod(static_cast<hardware_interface::canifier::CANifierStatusFrame>(i));
			for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
				cc.resetControlFramePeriod(static_cast<hardware_interface::canifier::CANifierControlFrame>(i));
		}

		for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
		{
			const auto led_channel = static_cast<hardware_interface::canifier::LEDChannel>(i);
			double percent_output;
			ctre::phoenix::CANifier::LEDChannel ctre_led_channel;
			if (cc.ledOutputChanged(led_channel, percent_output) &&
				canifier_convert_.LEDChannel(led_channel, ctre_led_channel))
			{
				if (safeTalonCall(canifier->SetLEDOutput(percent_output, ctre_led_channel), "canifier->SetLEDOutput", cs.getCANId()))
				{
					cs.setLEDOutput(led_channel, percent_output);
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set LED channel " << i << " to " << percent_output);
				}
				else
				{
					cc.resetLEDOutput(led_channel);
				}
			}
		}
		for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
		{
			const auto general_pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
			bool value;
			bool output_enable;
			ctre::phoenix::CANifier::GeneralPin ctre_general_pin;
			if (cc.generalPinOutputChanged(general_pin, value, output_enable) &&
					canifier_convert_.generalPin(general_pin, ctre_general_pin))
			{
				if (safeTalonCall(canifier->SetGeneralOutput(ctre_general_pin, value, output_enable), "canifier->SetGeneralOutput", cs.getCANId()))
				{
					cs.setGeneralPinOutput(general_pin, value);
					cs.setGeneralPinOutputEnable(general_pin, output_enable);
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set General Pin " << i <<
							" to enable=" << output_enable << " value=" << value);
				}
				else
				{
					cc.resetGeneralPinOutput(general_pin);
				}
			}
		}

		// Don't bother with all the changed/reset code here, just copy
		// from command to state each time through the write call
		cs.setEncoderTicksPerRotation(cc.getEncoderTicksPerRotation());
		cs.setConversionFactor(cc.getConversionFactor());
		double position;
		if (cc.quadraturePositionChanged(position))
		{
			const double radians_scale = getConversionFactor(cs.getEncoderTicksPerRotation(), hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Position) * cs.getConversionFactor();
			if (safeTalonCall(canifier->SetQuadraturePosition(position / radians_scale), "canifier->SetQuadraturePosition", cs.getCANId()))
			{
				// Don't set state encoder position, let it be read at the next read() call
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set Quadrature Position to " << position);
			}
			else
			{
				cc.resetQuadraturePosition();
			}
		}

		hardware_interface::canifier::CANifierVelocityMeasPeriod period;
		ctre::phoenix::CANifierVelocityMeasPeriod                ctre_period;
		if (cc.velocityMeasurementPeriodChanged(period) &&
			canifier_convert_.velocityMeasurementPeriod(period, ctre_period))
		{
			if (safeTalonCall(canifier->ConfigVelocityMeasurementPeriod(ctre_period), "canifier->ConfigVelocityMeasurementPeriod", cs.getCANId()))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set velocity measurement Period to " << period);
				cs.setVelocityMeasurementPeriod(period);
			}
			else
			{
				cc.resetVelocityMeasurementPeriod();
			}
		}

		int window;
		if (cc.velocityMeasurementWindowChanged(window))
		{
			if (safeTalonCall(canifier->ConfigVelocityMeasurementWindow(window), "canifier->ConfigVelocityMeasurementWindow", cs.getCANId()))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set velocity measurement window to " << window);
				cs.setVelocityMeasurementWindow(window);
			}
			else
			{
				cc.resetVelocityMeasurementWindow();
			}
		}

		bool clear_position_on_limit_f;
		if (cc.clearPositionOnLimitFChanged(clear_position_on_limit_f))
		{
			if (safeTalonCall(canifier->ConfigClearPositionOnLimitF(clear_position_on_limit_f), "canifier->ConfigClearPositionOnLimitF", cs.getCANId()))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set clear position on limit F to " << clear_position_on_limit_f);
				cs.setClearPositionOnLimitF(clear_position_on_limit_f);
			}
			else
			{
				cc.resetClearPositionOnLimitF();
			}
		}

		bool clear_position_on_limit_r;
		if (cc.clearPositionOnLimitRChanged(clear_position_on_limit_r))
		{
			if (safeTalonCall(canifier->ConfigClearPositionOnLimitR(clear_position_on_limit_r), "canifier->ConfigClearPositionOnLimitR", cs.getCANId()))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set clear position on limit R to " << clear_position_on_limit_r);
				cs.setClearPositionOnLimitR(clear_position_on_limit_r);
			}
			else
			{
				cc.resetClearPositionOnLimitR();
			}
		}

		bool clear_position_on_quad_idx;
		if (cc.clearPositionOnQuadIdxChanged(clear_position_on_quad_idx))
		{
			if (safeTalonCall(canifier->ConfigClearPositionOnQuadIdx(clear_position_on_quad_idx), "canifier->ConfigClearPositionOnQuadIdx", cs.getCANId()))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set clear position on quad idx to " << clear_position_on_quad_idx);
				cs.setClearPositionOnQuadIdx(clear_position_on_quad_idx);
			}
			else
			{
				cc.resetClearPositionOnQuadIdx();
			}
		}

		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
			ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
			bool output_enable;
			if (cc.pwmOutputEnableChanged(pwm_channel, output_enable) &&
				canifier_convert_.PWMChannel(pwm_channel, ctre_pwm_channel))
			{
				if (safeTalonCall(canifier->EnablePWMOutput(ctre_pwm_channel, output_enable), "canifier->EnablePWMOutput", cs.getCANId()))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set pwm channel " << pwm_channel << " output enable to " << static_cast<int>(output_enable));
					cs.setPWMOutputEnable(pwm_channel, output_enable);
				}
				else
				{
					cc.resetPWMOutputEnable(pwm_channel);
				}
			}
		}

		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
			ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
			double output;
			if (cc.pwmOutputChanged(pwm_channel, output) &&
				canifier_convert_.PWMChannel(pwm_channel, ctre_pwm_channel))
			{
				if (safeTalonCall(canifier->SetPWMOutput(ctre_pwm_channel, output), "canifier->SetPWMOutput", cs.getCANId()))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set pwm channel " << pwm_channel << " output to " << output);
					cs.setPWMOutput(pwm_channel, output);
				}
				else
				{
					cc.resetPWMOutput(pwm_channel);
				}
			}
		}

		for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
		{
			const auto frame_id = static_cast<hardware_interface::canifier::CANifierStatusFrame>(i);
			ctre::phoenix::CANifierStatusFrame ctre_frame_id;
			int period;
			if (cc.statusFramePeriodChanged(frame_id, period) &&
				canifier_convert_.statusFrame(frame_id, ctre_frame_id))
			{
				if (safeTalonCall(canifier->SetStatusFramePeriod(ctre_frame_id, period), "canifier->SetStatusFramePeriod", cs.getCANId()))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set frame_id " << i << " status period to " << period);
					cs.setStatusFramePeriod(frame_id, period);
				}
				else
				{
					cc.resetStatusFramePeriod(frame_id);
				}
			}
		}

		for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
		{
			const auto frame_id = static_cast<hardware_interface::canifier::CANifierControlFrame>(i);
			ctre::phoenix::CANifierControlFrame ctre_frame_id;
			int period;
			if (cc.controlFramePeriodChanged(frame_id, period) &&
				canifier_convert_.controlFrame(frame_id, ctre_frame_id))
			{
				if (safeTalonCall(canifier->SetControlFramePeriod(ctre_frame_id, period), "canifier->SetControlFramePeriod", cs.getCANId()))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set frame_id " << i << " control period to " << period);
					cs.setControlFramePeriod(frame_id, period);
				}
				else
				{
					cc.resetControlFramePeriod(frame_id);
				}
			}
		}

		if (cc.clearStickyFaultsChanged())
		{
			if (safeTalonCall(canifier->ClearStickyFaults(), "canifier->ClearStickyFaults()", cs.getCANId()))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id] << " : cleared sticky faults");
				// No corresponding status field
			}
			else
			{
				cc.setClearStickyFaults();
			}
		}
	}
	for (size_t joint_id = 0; joint_id < num_cancoders_; ++joint_id)
	{
		if (!cancoder_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &cancoder = cancoders_[joint_id];
		auto &cs = cancoder_state_[joint_id];
		auto &cc = cancoder_command_[joint_id];
		if (cancoder->HasResetOccurred())
		{
			cc.resetPosition();
			cc.resetVelocityMeasPeriod();
			cc.resetVelocityMeasWindow();
			cc.resetAbsoluteSensorRange();
			cc.resetMagnetOffset();
			cc.resetInitializationStrategy();
			cc.resetFeedbackCoefficient();
			cc.resetDirection();
			cc.resetSensorDataStatusFramePeriod();
			cc.resetVBatAndFaultsStatusFramePeriod();
		}
		cs.setConversionFactor(cc.getConversionFactor());
		double position;
		if (cc.positionChanged(position))
		{
			if (safeTalonCall(cancoder->SetPosition(position / cs.getConversionFactor()), "cancoder->SetPosition", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set position to " << position);
				// Don't set state - it will be updated in next read() loop
			}
			else
			{
				cc.resetPosition();
			}
		}
		if (cc.positionToAbsoluteChanged())
		{
			if (safeTalonCall(cancoder->SetPositionToAbsolute(), "cancoder->SetPositionToAbsolute", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set position to absolute");
				// Don't set state - it will be updated in next read() loop
			}
			else
			{
				cc.setPositionToAbsolute();
			}
		}
		hardware_interface::cancoder::SensorVelocityMeasPeriod velocity_meas_period;
		ctre::phoenix::sensors::SensorVelocityMeasPeriod ctre_velocity_meas_period;
		if (cc.velocityMeasPeriodChanged(velocity_meas_period) &&
			cancoder_convert_.velocityMeasPeriod(velocity_meas_period, ctre_velocity_meas_period))
		{
			if (safeTalonCall(cancoder->ConfigVelocityMeasurementPeriod(ctre_velocity_meas_period), "cancoder->ConfigVelocityMeasurementPeriod", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set velocity measurement period to " << static_cast<int>(ctre_velocity_meas_period));
				cs.setVelocityMeasPeriod(velocity_meas_period);
			}
			else
			{
				cc.resetVelocityMeasPeriod();
			}
		}

		int velocity_meas_window;
		if (cc.velocityMeasWindowChanged(velocity_meas_window))
		{
			if (safeTalonCall(cancoder->ConfigVelocityMeasurementWindow(velocity_meas_window), "cancoder->ConfigVelocityMeasurementWindow", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set velocity measurement window to " << velocity_meas_window);
				cs.setVelocityMeasWindow(velocity_meas_window);
			}
			else
			{
				cc.resetVelocityMeasWindow();
			}
		}
		hardware_interface::cancoder::AbsoluteSensorRange absolute_sensor_range;
		ctre::phoenix::sensors::AbsoluteSensorRange ctre_absolute_sensor_range;
		if (cc.absoluteSensorRangeChanged(absolute_sensor_range) &&
			cancoder_convert_.absoluteSensorRange(absolute_sensor_range, ctre_absolute_sensor_range))
		{
			if (safeTalonCall(cancoder->ConfigAbsoluteSensorRange(ctre_absolute_sensor_range), "cancoder->ConfigAbsoluteSensorRange", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set absolute sensor range to " << absolute_sensor_range);
				cs.setAbsoluteSensorRange(absolute_sensor_range);
			}
			else
			{
				cc.resetAbsoluteSensorRange();
			}
		}

		double magnet_offset;
		if (cc.magnetOffsetChanged(magnet_offset))
		{
			if (safeTalonCall(cancoder->ConfigMagnetOffset(magnet_offset), "cancoder->ConfigMagnetOffset", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set magnet offset to " << magnet_offset);
				cs.setMagnetOffset(magnet_offset);
			}
			else
			{
				cc.resetMagnetOffset();
			}
		}

		hardware_interface::cancoder::SensorInitializationStrategy initialization_strategy;
		ctre::phoenix::sensors::SensorInitializationStrategy ctre_initialization_strategy;
		if (cc.InitializationStrategyChanged(initialization_strategy) &&
			cancoder_convert_.initializationStrategy(initialization_strategy, ctre_initialization_strategy))
		{
			if (safeTalonCall(cancoder->ConfigSensorInitializationStrategy(ctre_initialization_strategy), "cancoder->ConfigSensorInitializationStrategy", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set sensor intitialization strategy to " << initialization_strategy);
				cs.setInitializationStrategy(initialization_strategy);
			}
			else
			{
				cc.resetInitializationStrategy();
			}
		}
		double feedback_coefficient;
		std::string unit_string;
		hardware_interface::cancoder::SensorTimeBase time_base;
		ctre::phoenix::sensors::SensorTimeBase ctre_time_base;
		if (cc.feedbackCoefficientChanged(feedback_coefficient, unit_string, time_base) &&
			cancoder_convert_.timeBase(time_base, ctre_time_base))
		{
			if (safeTalonCall(cancoder->ConfigFeedbackCoefficient(feedback_coefficient, unit_string, ctre_time_base), "cancoder->ConfigFeedbackCoefficient", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set feedback coefficient to  " << feedback_coefficient << " " << unit_string << " " << time_base);
				cs.setFeedbackCoefficient(feedback_coefficient);
				cs.setUnitString(unit_string);
				cs.setTimeBase(time_base);
			}
			else
			{
				cc.resetFeedbackCoefficient();
			}
		}

		bool direction;
		if (cc.directionChanged(direction))
		{
			if (safeTalonCall(cancoder->ConfigSensorDirection(direction), "cancoder->ConfigSensorDirection", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set direction to " << direction);
				cs.setDirection(direction);
			}
			else
			{
				cc.resetDirection();
			}
		}

		int sensor_data_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(sensor_data_status_frame_period))
		{
			if (safeTalonCall(cancoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, sensor_data_status_frame_period), "cancoder->SetStatusFramePeriod(SensorData)", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set sensor data status frame period to " << sensor_data_status_frame_period);
				cs.setSensorDataStatusFramePeriod(sensor_data_status_frame_period);
			}
			else
			{
				cc.resetSensorDataStatusFramePeriod();
			}
		}

		int vbat_and_faults_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(vbat_and_faults_status_frame_period))
		{
			if (safeTalonCall(cancoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, vbat_and_faults_status_frame_period), "cancoder->SetStatusFramePeriod(VbatAndFaults)", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set vbat and fault status frame period to " << vbat_and_faults_status_frame_period);
				cs.setVbatAndFaultsStatusFramePeriod(vbat_and_faults_status_frame_period);
			}
			else
			{
				cc.resetVBatAndFaultsStatusFramePeriod();
			}
		}

		if (cc.clearStickyFaultsChanged())
		{
			if (safeTalonCall(cancoder->ClearStickyFaults(), "cancoder->ClearStickyFaults", cs.getDeviceNumber()))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id] << " : Sticky faults cleared");
			}
			else
			{
				cc.setClearStickyFaults();
			}
		}
	}

	for (size_t i = 0; i < num_as726xs_; i++)
	{
		hardware_interface::as726x::IndLedCurrentLimits ind_led_current_limit;
		bool ind_led_enable;
		hardware_interface::as726x::DrvLedCurrentLimits drv_led_current_limit;
		bool drv_led_enable;
		hardware_interface::as726x::ConversionTypes conversion_type;
		hardware_interface::as726x::ChannelGain gain;
		uint8_t integration_time;

		auto &ac = as726x_command_[i];
		const bool ind_led_current_limit_changed = ac.indLedCurrentLimitChanged(ind_led_current_limit);
		const bool ind_led_enable_changed        = ac.indLedEnableChanged(ind_led_enable);
		const bool drv_led_current_limit_changed = ac.drvLedCurrentLimitChanged(drv_led_current_limit);
		const bool drv_led_enable_changed        = ac.drvLedEnableChanged(drv_led_enable);
		const bool conversion_type_changed       = ac.conversionTypeChanged(conversion_type);
		const bool gain_changed                  = ac.gainChanged(gain);
		const bool integration_time_changed      = ac.integrationTimeChanged(integration_time);
		// Only attempt to lock access to the sensor if there's actually
		// any changed config values to write to it
		if (ind_led_current_limit_changed ||
				ind_led_enable_changed ||
				drv_led_current_limit_changed ||
				drv_led_enable_changed ||
				conversion_type_changed ||
				gain_changed ||
				integration_time_changed)
		{
			// Try to get a lock for this instance of AS726x.  If not available
			// due to the read thread accessing the sensor,
			// continue on to the next sensor in the loop one.  This way the code won't be waiting
			// for a mutex to be released - it'll move on and try again next
			// iteration of write()
			std::unique_lock<std::mutex> l(*(as726xs_[i]->getMutex()), std::defer_lock);
			if (!l.try_lock())
			{
				// If we can't get a lock this iteration, reset and try again next time through
				if (ind_led_current_limit_changed)
					ac.resetIndLedCurrentLimit();
				if (ind_led_enable_changed)
					ac.resetIndLedEnable();
				if (drv_led_current_limit_changed)
					ac.resetDrvLedCurrentLimit();
				if (drv_led_enable_changed)
					ac.resetDrvLedEnable();
				if (conversion_type_changed)
					ac.resetConversionType();
				if (gain_changed)
					ac.resetGain();
				if (integration_time_changed)
					ac.resetIntegrationTime();
				continue;
			}
			auto &as = as726x_state_[i];

			if (ind_led_current_limit_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::ind_led_current_limits as726x_ind_led_current_limit;
					if (as726x_convert_.indLedCurrentLimit(ind_led_current_limit, as726x_ind_led_current_limit))
						as726xs_[i]->setIndicateCurrent(as726x_ind_led_current_limit);
				}
				as.setIndLedCurrentLimit(ind_led_current_limit);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " ind_led_current_limit = " << ind_led_current_limit);
			}

			if (ind_led_enable_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726xs_[i]->indicateLED(ind_led_enable);
				}
				as.setIndLedEnable(ind_led_enable);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " ind_led_enable = " << ind_led_enable);
			}

			if (drv_led_current_limit_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::drv_led_current_limits as726x_drv_led_current_limit;
					if (as726x_convert_.drvLedCurrentLimit(drv_led_current_limit, as726x_drv_led_current_limit))
						as726xs_[i]->setDrvCurrent(as726x_drv_led_current_limit);
				}
				as.setDrvLedCurrentLimit(drv_led_current_limit);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " drv_led_current_limit = " << drv_led_current_limit);
			}

			if (drv_led_enable_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					if (drv_led_enable)
					{
						as726xs_[i]->drvOn();
					}
					else
					{
						as726xs_[i]->drvOff();
					}
				}
				as.setDrvLedEnable(drv_led_enable);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " drv_led_enable = " << drv_led_enable);
			}

			if (conversion_type_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::conversion_types as726x_conversion_type;
					if (as726x_convert_.conversionType(conversion_type, as726x_conversion_type))
						as726xs_[i]->setConversionType(as726x_conversion_type);
				}
				as.setConversionType(conversion_type);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " conversion_type = " << conversion_type);
			}

			if (gain_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::channel_gain as726x_gain;
					if (as726x_convert_.channelGain(gain, as726x_gain))
						as726xs_[i]->setGain(as726x_gain);
				}
				as.setGain(gain);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " channel_gain = " << gain);
			}

			if (integration_time_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726xs_[i]->setIntegrationTime(integration_time);
				}
				as.setIntegrationTime(integration_time);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] <<  " integration_time = "
						<< static_cast<int>(integration_time));
			}
		}
	}

        for(size_t i = 0; i < num_talon_orchestras_; i++)
        {
            auto &oc = orchestra_command_[i];
            auto &os = orchestra_state_[i];
            std::string music_file_path;
            std::vector<std::string> instruments;
            if(oc.clearInstrumentsChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->ClearInstruments(), "ClearInstruments", 0))
                {
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " cleared instruments.");
                }
                else{
                    ROS_ERROR_STREAM("Failed to clear instruments in orchestra.");
                }
            }
            if(oc.instrumentsChanged(instruments))
            {
                if(safeTalonCall(talon_orchestras_[i]->ClearInstruments(), "ClearInstruments", 0))
                {
                    for(size_t j = 0; j < instruments.size(); j++)
                    {
                        size_t can_index = std::numeric_limits<size_t>::max();
                        for(size_t k = 0; k < can_ctre_mc_names_.size(); k++)
                        {
                            if(can_ctre_mc_names_[k] == instruments[j])
                            {
                                can_index = k;
                                break;
                            }
                        }
                        if(can_index == std::numeric_limits<size_t>::max())
                        {
                            ROS_ERROR_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " failed to add " << instruments[j] << " because it does not exist");
                        }
                        else if(can_ctre_mc_is_talon_fx_[can_index])
                        {
                            if(safeTalonCall(talon_orchestras_[i]->AddInstrument(*(std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonFX>(ctre_mcs_[can_index]))), "AddInstrument", 0))
                                ROS_INFO_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " added Falcon " << "falcon_name");
                            else{
                                ROS_ERROR_STREAM("Failed to add instrument to orchestra");
                                oc.resetInstruments();
                            }
                        }
                        else
                            ROS_INFO_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " failed to add " << instruments[j] << " because it is not a TalonFX");
                    }
                    os.setInstruments(instruments);
                }
                else{
                    ROS_ERROR_STREAM("Failed to clear instruments in orchestra");
                    //oc.resetClearInstruments();
                }
            }
            if(oc.musicChanged(music_file_path))
            {
                if(safeTalonCall(talon_orchestras_[i]->LoadMusic(music_file_path), "LoadMusic", 0))
                {
                    os.setChirpFilePath(music_file_path);
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " loaded music at " << music_file_path);
                }
                else{
                    ROS_ERROR_STREAM("Failed to load music into orchestra");
                    //oc.resetMusic();
                }
            }
            if(oc.pauseChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->Pause(), "Pause", 0))
                {
                    //os.setPaused();
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " pausing");
                }
                else{
                    ROS_ERROR_STREAM("Failed to pause orchestra");
                    //oc.pause();
                }
            }
            if(oc.playChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->Play(), "Play", 0))
                {
                    //os.setPlaying();
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " playing");
                }
                else{
                    ROS_ERROR_STREAM("Failed to play orchestra");
                    //oc.play();
                }
            }
            if(oc.stopChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->Stop(), "Stop", 0))
                {
                    //os.setStopped();
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " stopping");
                }
                else{
                    ROS_ERROR_STREAM("Failed to stop orchestra");
                    //oc.stop();
                }
            }
        }

	// TODO : what to do about this?
	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		if (dummy_joint_locals_[i])
		{
			// Use dummy joints to communicate info between
			// various controllers and driver station smartdash vars
			{
				dummy_joint_effort_[i] = 0;
				//if (dummy_joint_names_[i].substr(2, std::string::npos) == "_angle")
				{
					// position mode
					dummy_joint_velocity_[i] = (dummy_joint_command_[i] - dummy_joint_position_[i]) / period.toSec();
					dummy_joint_position_[i] = dummy_joint_command_[i];
				}
#if 0
				else if (dummy_joint_names_[i].substr(2, std::string::npos) == "_drive")
				{
					// velocity mode
					dummy_joint_position_[i] += dummy_joint_command_[i] * period.toSec();
					dummy_joint_velocity_[i] = dummy_joint_command_[i];
				}
#endif
			}
		}
	}
}

bool FRCRobotHWInterface::DSErrorCallback(ros_control_boilerplate::DSError::Request &req, ros_control_boilerplate::DSError::Response &res)
{
	ROS_ERROR_STREAM("HWI received DSErrorCallback " << req.details.c_str());
	HAL_SendError(true, req.error_code, false, req.details.c_str(), "", "", true);
	return true;
}

} // namespace
