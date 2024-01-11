#ifndef CAN_BUS_STATUS_DEVICE_INC__
#define CAN_BUS_STATUS_DEVICE_INC__

#include <string>

#include <ros/node_handle.h>

namespace hardware_interface::can_bus_status
{
    class CANBusStatusHWState;
    class CANBusStatusStateInterface;
}
class PeriodicIntervalCounter;

class CANBusStatusDevice
{
public:
    explicit CANBusStatusDevice(const std::string &name,
                                const double read_hz);
    CANBusStatusDevice(const CANBusStatusDevice &) = delete;
    CANBusStatusDevice(CANBusStatusDevice &&other) noexcept = delete;
    virtual ~CANBusStatusDevice();

    CANBusStatusDevice &operator=(const CANBusStatusDevice &) = delete;
    CANBusStatusDevice &operator=(CANBusStatusDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::can_bus_status::CANBusStatusStateInterface &state_interface) const;
    void read(const ros::Time& time, const ros::Duration& period);

private:
    std::unique_ptr<hardware_interface::can_bus_status::CANBusStatusHWState> state_;
    std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
};

#endif