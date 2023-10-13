#ifndef PWM_DEVICES_H_INC__
#define PWM_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class PWMDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class PWMDevices : public Devices
{

public:
    PWMDevices(ros::NodeHandle &root_nh);
    PWMDevices(const PWMDevices &) = delete;
    PWMDevices(PWMDevices &&) noexcept = delete;
    virtual ~PWMDevices();

    PWMDevices &operator=(const PWMDevices &) = delete;
    PWMDevices &operator=(PWMDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<PWMDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::JointCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::PositionJointInterface> position_joint_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif