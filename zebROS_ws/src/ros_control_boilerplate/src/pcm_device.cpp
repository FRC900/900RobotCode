#include <thread>

#include "ros/node_handle.h"

#include <hardware_interface/joint_command_interface.h>
#include "frc/PneumaticsControlModule.h"

#include "frc_interfaces/pcm_state_interface.h"
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/pcm_device.h"
#include "ros_control_boilerplate/tracer.h"

PCMDevice::PCMDevice(const std::string &name_space,
                     const int joint_index,
                     const std::string &joint_name,
                     const int pcm_id,
                     const bool local_hardware,
                     const bool local_update,
                     const double read_hz_)
    : name_{joint_name}
    , pcm_id_{pcm_id}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , pcm_{local_hardware ? std::make_unique<frc::PneumaticsControlModule>(pcm_id) : nullptr}
    , state_{std::make_unique<hardware_interface::PCMState>(pcm_id)}
    , read_thread_state_{local_hardware ? std::make_unique<hardware_interface::PCMState>(pcm_id) : nullptr}
    , read_state_mutex_{local_hardware ? std::make_unique<std::mutex>() : nullptr}
    , read_thread_{local_hardware ? std::make_unique<std::jthread>(&PCMDevice::read_thread, this, std::make_unique<Tracer>("pcm_read_" + joint_name + " " + name_space), read_hz_) : nullptr}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << name_ <<
                        (local_update_ ? " local" : " remote") << " update, " <<
                        (local_hardware_ ? "local" : "remote") << " hardware" <<
                        " as PCM " << pcm_id_);
}

PCMDevice::~PCMDevice(void) = default;

void PCMDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                   hardware_interface::JointCommandInterface &command_interface,
                                   hardware_interface::PositionJointInterface &position_joint_interface,
                                   hardware_interface::RemoteJointInterface &remote_joint_interface,
                                   hardware_interface::PCMStateInterface &pcm_state_interface,
                                   hardware_interface::RemotePCMStateInterface &remote_pcm_state_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for PCM : " << name_ << " at pcm id " << pcm_id_);
    // There's only one command to send to the PCM. It's a bool which enables
    // the compressor.  Model that here using a JointState / JointCommand
    // with the name of the PCM.
    hardware_interface::JointStateHandle state_handle(name_, &enable_state_, &enable_state_, &enable_state_);
    state_interface.registerHandle(state_handle);

    // If local_update is true, commands are coming from a local controller.
    // Expose the command_ buffer to the command_interface so that a local
    // controller can grab it.
    // If local_update is false, commands are coming from another hardware interface.
    // The path is -> other HW interface writes. The write doesn't go to actual hardware
    // (since nothing is connected to the other HW interface). Instead, it just updates
    // the joint state on the other device.  That change in state is seen by the 
    // joint listener controller on this HW interface. The joint listener controller
    // writes that value as a command to the remote joint below. The command is seen
    // by the next write() iteration on this HW interface, which then writes it to
    // the actual hardware.
    hardware_interface::JointHandle command_handle(state_handle, &enable_command_);
    if (local_update_)
    {
        command_interface.registerHandle(command_handle);
        position_joint_interface.registerHandle(command_handle);
    }
    else
    {
        remote_joint_interface.registerHandle(command_handle);
    }

    // The remainder of the state is read-only, and contained in the state_ PCMState
    // struct. Register it here.
    hardware_interface::PCMStateHandle pcm_state_handle(name_, state_.get());
    pcm_state_interface.registerHandle(pcm_state_handle);

    // If the hardware is connected to the remote hardware interface, create
    // a path for a listener controller to read the state from that HWI's state
    // publisher and write it into the local hardware interface's state_ buffer.
    // Note - we should really split local_hardware_ into local_read and local_write,
    // but we can get away with just one flag here because the low-level CAN
    // code on the Jetson explicitly filters out CAN writes to the PCM.
    if (!local_hardware_)
    {
        hardware_interface::PCMWritableStateHandle remote_pcm_handle(name_, state_.get());
        remote_pcm_state_interface.registerHandle(remote_pcm_handle);
    }
}

void PCMDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_hardware_)
    {
        std::unique_lock l(*read_state_mutex_, std::try_to_lock);
        if (l.owns_lock())
        {
            *state_ = *read_thread_state_;
        }
    }
}

void PCMDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (enable_command_ != enable_state_)
    {
        if (local_hardware_)
        {
            const bool setpoint = enable_command_ > 0;
            if (setpoint)
            {
                pcm_->EnableCompressorDigital();
            }
            else
            {
                pcm_->DisableCompressor();
            }
        }
        // TODO - should we retry if status != 0?
        enable_state_ = enable_command_;
        ROS_INFO_STREAM("Wrote pcm " << name_ << " closedloop enable = " << enable_command_);
    }
}

void PCMDevice::read_thread(std::unique_ptr<Tracer> tracer,
                            double poll_frequency)
{
    const auto pcm_id = pcm_->GetModuleNumber();
#ifdef __linux__
	std::stringstream s;
	s << "pcm_read_" << pcm_->GetModuleNumber();
	if (pthread_setname_np(pthread_self(), s.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << s.str() << " " << errno);
	}
#endif
	ros::Duration(2.1 + pcm_id / 10.).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting pcm " << pcm_id << " read thread at " << ros::Time::now());
	//pcm_->ClearAllStickyFaults(); TODO broken in wpilib
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("main loop");

		hardware_interface::PCMState pcm_state(pcm_id);
		pcm_state.setCompressorEnabled(pcm_->GetCompressor());
		pcm_state.setPressureSwitch(pcm_->GetPressureSwitch());
		pcm_state.setCompressorCurrent(pcm_->GetCompressorCurrent().value());
		pcm_state.setClosedLoopControl(pcm_->GetCompressorConfigType() != frc::CompressorConfigType::Disabled);
		pcm_state.setCurrentTooHigh(pcm_->GetCompressorCurrentTooHighFault());
		pcm_state.setCurrentTooHighSticky(pcm_->GetCompressorCurrentTooHighStickyFault());

		pcm_state.setShorted(pcm_->GetCompressorShortedFault());
		pcm_state.setShortedSticky(pcm_->GetCompressorShortedStickyFault());
		pcm_state.setNotConntected(pcm_->GetCompressorNotConnectedFault());
		pcm_state.setNotConnecteSticky(pcm_->GetCompressorNotConnectedStickyFault());
		pcm_state.setVoltageFault(pcm_->GetSolenoidVoltageFault());
		pcm_state.setVoltageSticky(pcm_->GetSolenoidVoltageStickyFault());
		pcm_state.setSolenoidDisabledList(pcm_->GetSolenoidDisabledList());

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard l(*read_state_mutex_);
			*read_thread_state_ = pcm_state;
		}
		tracer->report(60);
	}
}