#ifndef SOLENOID_DEVICE_INC__
#define SOLENOID_DEVICE_INC__

#include <memory>
#include <string>

#include "frc/PneumaticsModuleType.h"

namespace frc { class Solenoid; }
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class JointModeInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class SolenoidDevice
{
public:
    SolenoidDevice(const int joint_index,
                   const std::string &joint_name,
                   const int channel,
                   const frc::PneumaticsModuleType type,
                   const int module_id,
                   const bool local_hardware,
                   const bool local_update);
    SolenoidDevice(const SolenoidDevice &) = delete;
    SolenoidDevice(SolenoidDevice &&other) noexcept = delete;
    virtual ~SolenoidDevice();

    SolenoidDevice &operator=(const SolenoidDevice &) = delete;
    SolenoidDevice &operator=(SolenoidDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::JointCommandInterface &command_interface,
                            hardware_interface::PositionJointInterface &position_joint_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface,
                            hardware_interface::JointModeInterface &joint_mode_interface,
                            hardware_interface::RemoteJointModeInterface &remote_joint_mode_interface);
    void write(const ros::Time& time, const ros::Duration& period);

private:
    const std::string name_;
    const int channel_;
    const frc::PneumaticsModuleType module_type_;
    const int module_id_;
    const bool local_hardware_;
    const bool local_update_;
    std::unique_ptr<frc::Solenoid> solenoid_;

    double state_{std::numeric_limits<double>::max()};
    double pwm_state_{std::numeric_limits<double>::max()};
    hardware_interface::JointCommandModes mode_{hardware_interface::JointCommandModes::MODE_POSITION};
    hardware_interface::JointCommandModes prev_mode_{hardware_interface::JointCommandModes::BEGIN};
    double command_{0};
};

#endif