#include <hardware_interface/joint_command_interface.h>

#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/pwm_device.h"
#include "frc/PWM.h"

PWMDevice::PWMDevice(const int joint_index,
                     const std::string &joint_name,
                     const int pwm_channel,
                     const double output_max,
                     const double deadband_max,
                     const double center,
                     const double output_min,
                     const double deadband_min,
                     const int period_multiplier,
                     const bool invert,
                     const bool local_hardware,
                     const bool local_update,
                     const bool print)
    : pwm_{local_hardware ? std::make_unique<frc::PWM>(pwm_channel) : nullptr}
    , name_{joint_name}
    , pwm_channel_{pwm_channel}
    , invert_{invert}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
{
    ROS_INFO_STREAM("pwm setbounds : " << output_max << " " << deadband_max << " " << center << " " << deadband_min << " " << output_min);
    pwm_->SetBounds(units::microsecond_t{output_max},
                    units::microsecond_t{deadband_max},
                    units::microsecond_t{center},
                    units::microsecond_t{deadband_min},
                    units::microsecond_t{output_min});
    frc::PWM::PeriodMultiplier frc_period_multiplier;
    switch(period_multiplier)
    {
        case 1:
            frc_period_multiplier = frc::PWM::kPeriodMultiplier_1X;
            break;
        case 2:
            frc_period_multiplier = frc::PWM::kPeriodMultiplier_2X;
            break;
        case 4:
            frc_period_multiplier = frc::PWM::kPeriodMultiplier_4X;
            break;
        default:
            throw std::runtime_error("Invalid PWM period multiplier " + std::to_string(period_multiplier) + " for joint " + joint_name);
    }
    pwm_->SetPeriodMultiplier(frc_period_multiplier);
    if (print)
    {
        ROS_INFO_STREAM("Loading joint " << joint_index << "=" << name_ <<
                        (local_update_ ? " local" : " remote") << " update, " <<
                        (local_hardware_ ? "local" : "remote") << " hardware" <<
                        " as PWM " << pwm_channel_ <<
                        " invert " << invert_);
    }
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
    const double setpoint = command_* ((invert_ && local_update_) ? -1 : 1);
    if (state_ != setpoint)
    {
        if (local_hardware_)
        {
            writeImpl(setpoint);
        }
        state_ = setpoint;
        ROS_INFO_STREAM("PWM " << name_ << " at channel" << pwm_channel_ << " set to " << state_);
    }
}

void PWMDevice::writeImpl(const double value)
{
    pwm_->SetSpeed(value);
}