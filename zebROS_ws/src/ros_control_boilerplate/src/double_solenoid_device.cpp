#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"

#include "ros_control_boilerplate/double_solenoid_device.h"
#include "frc/DoubleSolenoid.h"

DoubleSolenoidDevice::DoubleSolenoidDevice(const int joint_index,
                                           const std::string &joint_name,
                                           const int forward_channel,
                                           const int reverse_channel,
                                           const frc::PneumaticsModuleType module_type,
                                           const int module_id,
                                           const bool local_hardware,
                                           const bool local_update)
    : name_{joint_name}
    , forward_channel_{forward_channel}
    , reverse_channel_{reverse_channel}
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
        s << " as double solenoid with forward channel " << forward_channel_ <<
             " and reverse channel " << reverse_channel_ <<
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
        double_solenoid_ = std::make_unique<frc::DoubleSolenoid>(module_id_, module_type_, forward_channel_, reverse_channel_);
    }
}

DoubleSolenoidDevice::~DoubleSolenoidDevice() = default;

void DoubleSolenoidDevice::registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                                              hardware_interface::JointCommandInterface &command_interface,
                                              hardware_interface::PositionJointInterface &position_joint_interface,
                                              hardware_interface::RemoteJointInterface &remote_joint_interface)
{
    std::stringstream s;
    s <<  "FRCRobotInterface: Registering interface for " << name_;
    if (local_hardware_)
    {
        s << " with forward channel " << forward_channel_ <<
            " & reverse channel " << reverse_channel_ <<
            " at " << (module_type_ == frc::PneumaticsModuleType::CTREPCM ? "ctrepcm" : "revph") <<
            " id " << module_id_;
    }
    else
    {
        s << " on remote hardware";
    }
    ROS_INFO_STREAM(s.str());
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

#if 0
void DoubleSolenoidDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_update_)
    {
        // ROS_INFO_STREAM("read : solenoid = " << solenoids_[i]->Get() <<  " " << solenoids_[i]->GetChannel());
        //  TODO - only works if robot is enabled, so not sure how best to handle this
        state_ = solenoid_->Get();
        // if (pcms_.size() && pcms_[0])
        // ROS_INFO_STREAM("pcms_[0]->GetDoubleSolenoids() = " << pcms_[0]->GetDoubleSolenoids());
    }
}
#endif

void DoubleSolenoidDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    // Not sure if it makes sense to store command values
    // in state or wpilib enum values
    if (state_ != command_)
    {
        if (local_hardware_)
        {
            frc::DoubleSolenoid::Value value;
            if (command_ >= 1.0)
                value = frc::DoubleSolenoid::Value::kForward;
            else if (command_ <= -1.0)
                value = frc::DoubleSolenoid::Value::kReverse;
            else
                value = frc::DoubleSolenoid::Value::kOff;

            double_solenoid_->Set(value);
        }
        state_ = command_;
        ROS_INFO_STREAM("Wrote double solenoid " << name_ << " = " << command_);
    }
}