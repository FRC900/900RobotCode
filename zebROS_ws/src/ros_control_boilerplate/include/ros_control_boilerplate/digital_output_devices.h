#ifndef DIGITAL_OUTPUT_DEVICES_H_INC__
#define DIGITAL_OUTPUT_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class DigitalOutputDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class DigitalOutputDevices : public Devices
{

public:
    explicit DigitalOutputDevices(ros::NodeHandle &root_nh);
    DigitalOutputDevices(const DigitalOutputDevices &) = delete;
    DigitalOutputDevices(DigitalOutputDevices &&) = delete;
    ~DigitalOutputDevices() override;

    DigitalOutputDevices &operator=(const DigitalOutputDevices &) = delete;
    DigitalOutputDevices &operator=(DigitalOutputDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<DigitalOutputDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::JointCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::PositionJointInterface> position_joint_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif