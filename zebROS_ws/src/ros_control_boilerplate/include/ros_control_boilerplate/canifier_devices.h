#ifndef CANIFIER_DEVICES_H_INC__
#define CANIFIER_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

template <bool SIMFLAG> class CANifierDevice;
namespace hardware_interface::canifier
{
    class CANifierStateInterface;
    class CANifierCommandInterface;
    class RemoteCANifierStateInterface;
}

template <bool SIMFLAG>
class CANifierDevices : public Devices
{

public:
    CANifierDevices(ros::NodeHandle &root_nh);
    CANifierDevices(const CANifierDevices &) = delete;
    CANifierDevices(CANifierDevices &&) noexcept = delete;
    ~CANifierDevices();

    CANifierDevices &operator=(const CANifierDevices &) = delete;
    CANifierDevices &operator=(CANifierDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    double read_hz_{100};
    std::vector<std::unique_ptr<CANifierDevice<SIMFLAG>>> devices_;
    std::unique_ptr<hardware_interface::canifier::CANifierStateInterface> canifier_state_interface_;
    std::unique_ptr<hardware_interface::canifier::CANifierCommandInterface> canifier_command_interface_;
    std::unique_ptr<hardware_interface::canifier::RemoteCANifierStateInterface> remote_canifier_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

using HWCANifierDevices = CANifierDevices<false>;
using SimCANifierDevices = CANifierDevices<true>;

#endif