// This is similar to PWM devices, but the default bounds are different.  
#ifndef SERVO_DEVICES_H_INC__
#define SERVO_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class ServoDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class ServoDevices : public Devices
{
public:
    explicit ServoDevices(ros::NodeHandle &root_nh);
    ServoDevices(const ServoDevices &) = delete;
    ServoDevices(ServoDevices &&) noexcept = delete;
    ~ServoDevices() override;

    ServoDevices &operator=(const ServoDevices &) = delete;
    ServoDevices &operator=(ServoDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<ServoDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::JointCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::PositionJointInterface> position_joint_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif