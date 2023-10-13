#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"

#include "ros_control_boilerplate/solenoid_device.h"
#include "frc/Solenoid.h"

SolenoidDevice::SolenoidDevice(const int joint_index,
                               const std::string &joint_name,
                               const int channel,
                               const frc::PneumaticsModuleType module_type,
                               const int module_id,
                               const bool local_hardware,
                               const bool local_update)
    : name_{joint_name}
    , channel_{channel}
    , module_type_{module_type}
    , module_id_{module_id}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
{
    std::stringstream s;
    s << "Loading joint " << joint_index << "=" << name_ <<
        (local_update_ ? " local" : " remote") << " update, " <<
        (local_hardware_ ? "local" : "remote") << " hardware";
    if (local_hardware_)
    {
        s << " as solenoid at channel " << channel_ <<
             " at " << (module_type_ == frc::PneumaticsModuleType::CTREPCM ? "ctrepcm" : "revph") <<
             " id " << module_id_;
    }
    else
    {
        s << " on remote hardware";
    }

    ROS_INFO_STREAM(s.str());

    // Need to have 1 solenoid instantiated on the Rio to get
    // support for compressor and so on loaded?
    if (local_hardware_)
    {
        solenoid_ = std::make_unique<frc::Solenoid>(module_id_, module_type_, channel_);
    }
}

SolenoidDevice::~SolenoidDevice() = default;

void SolenoidDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                        hardware_interface::JointCommandInterface &command_interface,
                                        hardware_interface::PositionJointInterface &position_joint_interface,
                                        hardware_interface::RemoteJointInterface &remote_joint_interface,
                                        hardware_interface::JointModeInterface &joint_mode_interface,
                                        hardware_interface::RemoteJointModeInterface &remote_joint_mode_interface)
{
    std::stringstream s;
    s <<  "FRCRobotInterface: Registering interface for " << name_;
    if (local_hardware_)
    {
        s << " at channel " << channel_ <<
            " at " << (module_type_ == frc::PneumaticsModuleType::CTREPCM ? "ctrepcm" : "revph") <<
            " id " << module_id_;
    }
    else
    {
        s << " on remote hardware";
    }
    ROS_INFO_STREAM(s.str());
    hardware_interface::JointStateHandle state_handle(name_, &state_, &state_, &pwm_state_);
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

    hardware_interface::JointModeHandle mode_handle(name_, &mode_);
    if (local_update_)
    {
        joint_mode_interface.registerHandle(mode_handle);
    }
    else
    {
        remote_joint_mode_interface.registerHandle(mode_handle);
    }
}

#if 0
void SolenoidDevice::read(const ros::Time &time, const ros::Duration &period)
{
    if (local_update_)
    {
        // ROS_INFO_STREAM("read : solenoid = " << solenoids_[i]->Get() <<  " " << solenoids_[i]->GetChannel());
        //  TODO - only works if robot is enabled, so not sure how best to handle this
        state_ = solenoid_->Get();
        // if (pcms_.size() && pcms_[0])
        // ROS_INFO_STREAM("pcms_[0]->GetSolenoids() = " << pcms_[0]->GetSolenoids());
    }
}
#endif

void SolenoidDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    // MODE_POSITION is standard on/off setting
    if (mode_ == hardware_interface::JointCommandModes::MODE_POSITION)
    {
        const bool on = command_ > 0;
        if ((mode_ != prev_mode_) || (state_ != on))
        {
            if (local_hardware_)
            {
                solenoid_->Set(on);
            }
            state_ = on;
            ROS_INFO_STREAM_NAMED(name_, "Write solenoid " << name_ << " = " << static_cast<int>(on));
        }
    }
    // MODE_EFFORT is PWM via one-shot duration pulses
    else if (mode_ == hardware_interface::JointCommandModes::MODE_EFFORT)
    {
        if (command_ > 0)
        {
            if (local_hardware_)
            {
                // TODO - do we need to wait for previous one-shot to expire before sending another one?
                solenoid_->SetPulseDuration(static_cast<units::second_t>(command_));
                solenoid_->StartPulse();
            }
            ROS_INFO_STREAM_NAMED(name_, "Wrote solenoid one shot " << name_ << " = " << command_);
            // TODO - should we re-load this saved pwm command after the previous
            // one expires, or require a controller to monitor and reload?
            pwm_state_ = command_;
            // TODO - this will be polled in the read() function on interface local
            // to the hardware. How will the remote interface get that state update?
            state_ = 1;
            command_ = 0;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Invalid mode_ = " << static_cast<int>(mode_));
    }
    prev_mode_ = mode_;
}
