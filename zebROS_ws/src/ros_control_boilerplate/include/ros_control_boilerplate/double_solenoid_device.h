#ifndef DOUBLE_SOLENOID_DEVICE_INC__
#define DOUBLE_SOLENOID_DEVICE_INC__

#include <memory>
#include <string>

#include "frc/PneumaticsModuleType.h"

namespace frc { class DoubleSolenoid; }
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
};

class DoubleSolenoidDevice
{
public:
    DoubleSolenoidDevice(const int joint_index,
                         const std::string &joint_name,
                         const int forward_channel,
                         const int reverse_channel,
                         const frc::PneumaticsModuleType type,
                         const int module_id,
                         const bool local_hardware,
                         const bool local_update);
    DoubleSolenoidDevice(const DoubleSolenoidDevice &) = delete;
    DoubleSolenoidDevice(DoubleSolenoidDevice &&other) noexcept = delete;
    virtual ~DoubleSolenoidDevice();

    DoubleSolenoidDevice &operator=(const DoubleSolenoidDevice &) = delete;
    DoubleSolenoidDevice &operator=(DoubleSolenoidDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::JointCommandInterface &command_interface,
                            hardware_interface::PositionJointInterface &position_joint_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface);
    void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

private:
    const std::string name_;
    const int forward_channel_;
    const int reverse_channel_;
    const frc::PneumaticsModuleType module_type_;
    const int module_id_;
    const bool local_hardware_;
    const bool local_update_;
    std::unique_ptr<frc::DoubleSolenoid> double_solenoid_;

    double state_{std::numeric_limits<double>::max()};
    double command_{0};
};

#endif