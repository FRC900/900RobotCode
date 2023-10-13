#include <thread>

#include "ros/node_handle.h"

#include "frc/PneumaticHub.h"

#include "frc_interfaces/ph_command_interface.h"
#include "ros_control_boilerplate/ph_device.h"
#include "ros_control_boilerplate/tracer.h"

PHDevice::PHDevice(const std::string &name_space,
                   const int joint_index,
                   const std::string &joint_name,
                   const int ph_id,
                   const bool local_read,
                   const bool local_write,
                   const double read_hz_)
    : name_{joint_name}
    , ph_id_{ph_id}
    , local_read_{local_read}
    , local_write_{local_write}
    , ph_{local_read ? std::make_unique<frc::PneumaticHub>(ph_id) : nullptr}
    , state_{std::make_unique<hardware_interface::PHHWState>(ph_id)}
    , command_{std::make_unique<hardware_interface::PHHWCommand>()}
    , read_thread_state_{local_read_ ? std::make_unique<hardware_interface::PHHWState>(ph_id) : nullptr}
    , read_state_mutex_{local_read_ ? std::make_unique<std::mutex>() : nullptr}
    , read_thread_{local_read_ || local_write_? std::make_unique<std::thread>(&PHDevice::read_thread, this, std::make_unique<Tracer>("ph_read_" + joint_name + " " + name_space), read_hz_) : nullptr}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << name_ <<
                        (local_read_ ? "local" : "remote") << " read" <<
                        (local_write_ ? " local" : " remote") << " write, " <<
                        " as PH " << ph_id_);
}

PHDevice::~PHDevice(void)
{
    if (read_thread_ && read_thread_->joinable())
    {
        read_thread_->join();
    }
}

void PHDevice::registerInterfaces(hardware_interface::PHStateInterface &state_interface,
                                  hardware_interface::PHCommandInterface &command_interface,
                                  hardware_interface::RemotePHStateInterface &remote_state_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for PH : " << name_ << " at ph id " << ph_id_);

    hardware_interface::PHStateHandle state_handle(name_, state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::PHCommandHandle command_handle(state_handle, &(*command_));
    command_interface.registerHandle(command_handle);

    if (!local_read_)
    {
        hardware_interface::PHWritableStateHandle remote_handle(name_, state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

void PHDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_read_)
    {
        std::unique_lock<std::mutex> l(*read_state_mutex_, std::try_to_lock);
        if (l.owns_lock())
        {
            *state_ = *read_thread_state_;
        }
    }
}

void PHDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    double compressor_min_analog_voltage;
    double compressor_max_analog_voltage;
    bool compressor_force_disable;
    bool compressor_use_digital;
    if (command_->closedLoopControlChanged(compressor_min_analog_voltage, compressor_max_analog_voltage,
                                           compressor_force_disable, compressor_use_digital))
    {
        if (local_write_)
        {
            if (compressor_force_disable)
            {
                ph_->DisableCompressor();
            }
            else
            {
                if (compressor_use_digital)
                {
                    ph_->EnableCompressorDigital();
                }
                else
                {
                    ph_->EnableCompressorAnalog(static_cast<units::pounds_per_square_inch_t>(compressor_min_analog_voltage),
                                                static_cast<units::pounds_per_square_inch_t>(compressor_max_analog_voltage));
                }
                // TODO - hybrid?
            }
        }
        ROS_INFO_STREAM("Wrote ph " << name_
                        << " min_analog_voltage=" << compressor_min_analog_voltage
                        << " max_analog_voltage=" << compressor_max_analog_voltage
                        << " compressor_force_disable=" << static_cast<int>(compressor_force_disable)
                        << " compressor_use_digital=" << static_cast<int>(compressor_use_digital));
    }
}

void PHDevice::read_thread(std::unique_ptr<Tracer> tracer,
                           double poll_frequency)
{
	const auto ph_id = ph_->GetModuleNumber();
#ifdef __linux__
	std::stringstream s;
	s << "ph_read_" << ph_id;
	if (pthread_setname_np(pthread_self(), s.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << s.str() << " " << errno);
	}
#endif
	ros::Duration(1.7 + ph_id /10.).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting ph " << ph_id << " read thread at " << ros::Time::now());
    hardware_interface::PHHWState state(ph_id);
    for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("main loop");

		state.setCompressorEnabled(ph_->GetCompressor());
		state.setPressureSwitch(ph_->GetPressureSwitch());
		state.setCompressorCurrent(static_cast<double>(ph_->GetCompressorCurrent().value()));
		for (size_t i = 0; i < 2; i++)
		{
			state.setAnalogVoltage(static_cast<double>(ph_->GetAnalogVoltage(i).value()), i);
			state.setPressure(static_cast<double>(ph_->GetPressure(i).value()), i);
		}

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard<std::mutex> l(*read_state_mutex_);
			*read_thread_state_ = state;
		}

		tracer->report(60);
	}
}
