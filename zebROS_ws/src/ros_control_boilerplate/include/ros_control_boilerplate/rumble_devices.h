#ifndef RUMBLE_DEVICES_H_INC__
#define RUMBLE_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class RumbleDevice;
namespace hardware_interface
{
    class RumbleStateInterface;
    class RumbleCommandInterface;
    class RemoteRumbleStateInterface;
}

class RumbleDevices : public Devices
{
public:
    explicit RumbleDevices(ros::NodeHandle &root_nh);
    RumbleDevices(const RumbleDevices &) = delete;
    RumbleDevices(RumbleDevices &&) noexcept = delete;
    ~RumbleDevices() override;

    RumbleDevices &operator=(const RumbleDevices &) = delete;
    RumbleDevices &operator=(RumbleDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<RumbleDevice>> devices_;
    std::unique_ptr<hardware_interface::RumbleStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::RumbleCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::RemoteRumbleStateInterface> remote_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif