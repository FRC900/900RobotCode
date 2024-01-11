#ifndef CAN_BUS_STATUS_DEVICES_INC__
#define CAN_BUS_STATUS_DEVICES__

#include "ros_control_boilerplate/devices.h"

class CANBusStatusDevice;
namespace hardware_interface::can_bus_status
{
    class CANBusStatusStateInterface;
}

class CANBusStatusDevices : public Devices
{

public:
    explicit CANBusStatusDevices(ros::NodeHandle &root_nh);
    CANBusStatusDevices(const CANBusStatusDevices &) = delete;
    CANBusStatusDevices(CANBusStatusDevices &&) noexcept = delete;
    ~CANBusStatusDevices() override;

    CANBusStatusDevices &operator=(const CANBusStatusDevices &) = delete;
    CANBusStatusDevices &operator=(CANBusStatusDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<CANBusStatusDevice>> devices_;
    std::unique_ptr<hardware_interface::can_bus_status::CANBusStatusStateInterface> state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif