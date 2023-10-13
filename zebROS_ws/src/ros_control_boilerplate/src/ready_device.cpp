#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/ready_device.h"

ReadyDevice::ReadyDevice(const int joint_index,
                         const std::string &joint_name,
                         const bool local)
    : name_{joint_name}
    , local_{local}
    , state_{0.0}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM("Loading ready joint " << joint_index << "=" << name_ << " local = " << local_);
}

ReadyDevice::~ReadyDevice() = default;

void ReadyDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                             hardware_interface::JointCommandInterface &command_interface,
                                             hardware_interface::RemoteJointInterface &remote_joint_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for ready signal : " << name_);
    hardware_interface::JointStateHandle state_handle(name_, &state_, &state_, &state_);
    state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle command_handle(state_handle, &state_);
    command_interface.registerHandle(command_handle);

    if (!local_)
    {
        hardware_interface::JointHandle remote_handle(state_handle, &state_);
        remote_joint_interface.registerHandle(remote_handle);
    }
}

bool ReadyDevice::isReady(void) const
{
    return state_ != 0.0;
}