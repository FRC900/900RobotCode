#include "hal/DriverStation.h"

#include "frc_interfaces/rumble_command_interface.h"
#include "ros_control_boilerplate/rumble_device.h"

RumbleDevice::RumbleDevice(const int joint_index,
                           const std::string &joint_name,
                           const int port,
                           const bool local_hardware,
                           const bool local_update)
    : name_{joint_name}
    , port_{port}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , state_{std::make_unique<hardware_interface::RumbleHWState>()}
    , command_{std::make_unique<hardware_interface::RumbleHWCommand>()}
{
    // No real init needed here, just report the config loaded for them
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                         "Loading joint " << joint_index << "=" << name_ <<
                         (local_update_ ? " local" : " remote") << " update, " <<
                         (local_hardware_ ? "local" : "remote") << " hardware" <<
                         " as Rumble with port " << port_);
}

RumbleDevice::~RumbleDevice(void) = default;

void RumbleDevice::registerInterfaces(hardware_interface::RumbleStateInterface &state_interface,
                                      hardware_interface::RumbleCommandInterface &command_interface,
                                      hardware_interface::RemoteRumbleStateInterface &remote_state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Rumble : " << name_ << " at port " << port_);

    hardware_interface::RumbleStateHandle state_handle(name_, state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::RumbleCommandHandle command_handle(state_handle, &(*command_));
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::RumbleWritableStateHandle remote_handle(name_, state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

void RumbleDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    unsigned int left;
    unsigned int right;
    if (command_->changed(left, right))
    {
        if (local_hardware_)
        {
            HAL_SetJoystickOutputs(port_, 0, left, right);
        }
        state_->setLeft(left);
        state_->setRight(right);
        ROS_INFO_STREAM("Wrote rumble " << name_ << " left=" << left << " right=" << right);
    }
}