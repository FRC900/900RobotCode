#include <hardware_interface/joint_command_interface.h>

#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/pwm_device.h"
#include "frc/PWM.h"

PWMDevice::PWMDevice(const int joint_index,
                     const std::string &joint_name,
                     const int pwm_channel,
                     const bool invert,
                     const bool local_hardware,
                     const bool local_update)
    : name_{joint_name}
    , pwm_channel_{pwm_channel}
    , invert_{invert}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , pwm_{local_hardware ? std::make_unique<frc::PWM>(pwm_channel) : nullptr}
{
    // TODO : old code had a check for duplicate use of PWM channels? Needed?
    ROS_INFO_STREAM("Loading joint " << joint_index << "=" << name_ <<
                    (local_update_ ? " local" : " remote") << " update, " <<
                    (local_hardware_ ? "local" : "remote") << " hardware" <<
                    " as PWM " << pwm_channel_ <<
                    " invert " << invert_);
}

PWMDevice::~PWMDevice() = default;

void PWMDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                   hardware_interface::JointCommandInterface &command_interface,
                                   hardware_interface::PositionJointInterface &position_joint_interface,
                                   hardware_interface::RemoteJointInterface &remote_joint_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for PWM " << name_ << " at channel " << pwm_channel_ << " / invert = " << invert_);
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

void PWMDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    const double setpoint = command_* ((invert_ & local_update_) ? -1 : 1);
    if (state_ != setpoint)
    {
        if (local_hardware_)
        {
            pwm_->SetSpeed(setpoint);
        }
        state_ = setpoint;
        ROS_INFO_STREAM("PWM " << name_ << " at channel" << pwm_channel_ << " set to " << state_);
    }
}