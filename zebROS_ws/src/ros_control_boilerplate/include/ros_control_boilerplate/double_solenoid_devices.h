#ifndef DOUBLE_SOLENOID_DEVICES_H_INC__
#define DOUBLE_SOLENOID_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class DoubleSolenoidDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class DoubleSolenoidDevices : public Devices
{

public:
    explicit DoubleSolenoidDevices(ros::NodeHandle &root_nh);
    DoubleSolenoidDevices(const DoubleSolenoidDevices &) = delete;
    DoubleSolenoidDevices(DoubleSolenoidDevices &&) noexcept = delete;
    ~DoubleSolenoidDevices() override;

    DoubleSolenoidDevices &operator=(const DoubleSolenoidDevices &) = delete;
    DoubleSolenoidDevices &operator=(DoubleSolenoidDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<DoubleSolenoidDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::JointCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::PositionJointInterface> position_joint_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    hardware_interface::InterfaceManager interface_manager_;

};

#endif