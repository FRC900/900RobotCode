#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"

#include "ros_control_boilerplate/digital_input_device.h"
#include "frc/DigitalInput.h"
#include "hal/simulation/DIOData.h"

DigitalInputDevice::DigitalInputDevice(const int joint_index,
                                       const std::string &joint_name,
                                       const int dio_channel,
                                       const bool invert,
                                       const bool local)
    : name_{joint_name}
    , dio_channel_{dio_channel}
    , invert_{invert}
    , local_{local}
    , digital_input_{local_ ? std::make_unique<frc::DigitalInput>(dio_channel) : nullptr}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM("Loading joint " << joint_index <<
                    "=" << name_ <<
                    " local = " << local_ <<
                    " as Digital Input Channel " << dio_channel_ <<
                    " invert " << invert_);
    
}

DigitalInputDevice::~DigitalInputDevice() =default;

void DigitalInputDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                            hardware_interface::RemoteJointInterface &remote_joint_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Digital Input : " << name_ << " at channel " << dio_channel_ << " / invert = " << invert_);
    hardware_interface::JointStateHandle state_handle(name_, &state_, &state_, &state_);
    state_interface.registerHandle(state_handle);
    if (!local_)
    {
        hardware_interface::JointHandle remote_handle(state_handle, &state_);
        remote_joint_interface.registerHandle(remote_handle);
    }
}

void DigitalInputDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_)
    {
        state_ = (digital_input_->Get() ^ invert_) ? 1.0 : 0.0;
    }
}

const std::string &DigitalInputDevice::getName(void) const
{
    return name_;
}

void DigitalInputDevice::setSimValue(const bool value)
{
    HALSIM_SetDIOValue(dio_channel_, static_cast<int32_t>(value ^ invert_));
}
