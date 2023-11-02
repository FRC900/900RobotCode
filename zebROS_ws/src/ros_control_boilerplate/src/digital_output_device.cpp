#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"

#include "ros_control_boilerplate/digital_output_device.h"
#include "frc/DigitalOutput.h"

DigitalOutputDevice::DigitalOutputDevice(const int joint_index,
                                         const std::string &joint_name,
                                         const int dio_channel,
                                         const bool invert,
                                         const bool local_hardware,
                                         const bool local_update)
    : name_{joint_name}
    , dio_channel_{dio_channel}
    , invert_{invert}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , digital_output_{local_hardware ? std::make_unique<frc::DigitalOutput>(dio_channel) : nullptr}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM("Loading joint " << joint_index << "=" << name_ <<
                    (local_update_ ? " local" : " remote") << " update, " <<
                    (local_hardware_ ? "local" : "remote") << " hardware" <<
                    " as Digital Output " << dio_channel_ <<
                    " invert " << invert_);
}

DigitalOutputDevice::~DigitalOutputDevice() = default;

void DigitalOutputDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                             hardware_interface::JointCommandInterface &command_interface,
                                             hardware_interface::PositionJointInterface &position_joint_interface,
                                             hardware_interface::RemoteJointInterface &remote_joint_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Digital Output " << name_ << " at channel " << dio_channel_ << " / invert = " << invert_);
    hardware_interface::JointStateHandle state_handle(name_, &state_, &state_, &state_);
    state_interface.registerHandle(state_handle);
    hardware_interface::JointHandle command_handle(state_handle, &command_);
    if (local_update_)
    {
        command_interface.registerHandle(command_handle);
        position_joint_interface.registerHandle(command_handle);
    }
    else
    {
        remote_joint_interface.registerHandle(command_handle);
    }
}

void DigitalOutputDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    // Only invert the desired output once, on the controller where the update originated
    const bool converted_command = (command_ > 0) ^ (invert_ && local_update_);
    if (converted_command != state_)
    {
        const double double_command = converted_command ? 1.0 : 0.0;
        if (local_hardware_)
        {
            digital_output_->Set(double_command);
        }
        state_ = double_command ? 1.0 : 0.0;
        ROS_INFO_STREAM("Wrote digital output " << name_ << "=" << converted_command);
    }
}