#ifndef SOLENOID_DEVICES_H_INC__
#define SOLENOID_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class SolenoidDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
    class JointModeInterface;
    class RemoteJointModeInterface;
}

class SolenoidDevices : public Devices
{

public:
    SolenoidDevices(ros::NodeHandle &root_nh);
    SolenoidDevices(const SolenoidDevices &) = delete;
    SolenoidDevices(SolenoidDevices &&) noexcept = delete;
    virtual ~SolenoidDevices();

    SolenoidDevices &operator=(const SolenoidDevices &) = delete;
    SolenoidDevices &operator=(SolenoidDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<SolenoidDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::JointCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::PositionJointInterface> position_joint_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    std::unique_ptr<hardware_interface::JointModeInterface> mode_interface_;
    std::unique_ptr<hardware_interface::RemoteJointModeInterface> remote_mode_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif