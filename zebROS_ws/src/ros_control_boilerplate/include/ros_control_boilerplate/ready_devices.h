#ifndef READY_DEVICES_H_INC__
#define READY_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class ReadyDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class RemoteJointInterface;
}

class ReadyDevices : public Devices
{

public:
    ReadyDevices(ros::NodeHandle &root_nh);
    ReadyDevices(const ReadyDevices &) = delete;
    ReadyDevices(ReadyDevices &&) noexcept = delete;
    virtual ~ReadyDevices();

    ReadyDevices &operator=(const ReadyDevices &) = delete;
    ReadyDevices &operator=(ReadyDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    bool areReady(void) const;

private:
    std::vector<std::unique_ptr<ReadyDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::JointCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif