#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"

#include "ros_control_boilerplate/analog_input_device.h"
#include "frc/AnalogInput.h"
#include "hal/simulation/AnalogInData.h"

AnalogInputDevice::AnalogInputDevice(const int joint_index,
                                     const std::string &joint_name,
                                     const int ain_channel,
                                     const double a,
                                     const double b,
                                     const bool local)
    : name_{joint_name}
    , ain_channel_{ain_channel}
    , a_{a}
    , b_{b}
    , local_{local}
    , analog_input_{local_ ? std::make_unique<frc::AnalogInput>(ain_channel) : nullptr}
{
    // TODO : old code had a check for duplicate use of channels? Needed?
    ROS_INFO_STREAM("Loading joint " << joint_index <<
                    "=" << name_ <<
                    " local = " << local_ <<
                    " as Analog Input Channel " << ain_channel_ <<
                    " a = " << a_ <<
                    " b = " << b_);
}

AnalogInputDevice::~AnalogInputDevice() = default;

void AnalogInputDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                            hardware_interface::RemoteJointInterface &remote_joint_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Analog Input : " << name_ << " at channel " << ain_channel_);
    hardware_interface::JointStateHandle state_handle(name_, &state_, &state_, &state_);
    state_interface.registerHandle(state_handle);
    if (!local_)
    {
        hardware_interface::JointHandle remote_handle(state_handle, &state_);
        remote_joint_interface.registerHandle(remote_handle);
    }
}

void AnalogInputDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_)
    {
        state_ = analog_input_->GetValue() * a_ + b_;
    }
}

const std::string &AnalogInputDevice::getName(void) const
{
    return name_;
}

void AnalogInputDevice::setSimValue(const double value)
{
    HALSIM_SetAnalogInVoltage(ain_channel_, static_cast<int32_t>((value - b_) / a_));
}
