#ifndef CANDLE_DEVICE_INC__
#define CANDLE_DEVICE_INC__

#include <string>

#include <ros/node_handle.h>

namespace ctre::phoenix::led
{
    class CANdle;
}
namespace hardware_interface::candle
{
    class CANdleHWState;
    class CANdleHWCommand;
    class CANdleStateInterface;
    class CANdleCommandInterface;
    class RemoteCANdleStateInterface;
}
#include "ros_control_boilerplate/ctre_v5_device.h"

template <bool SIMFLAG>
class CANdleDevice : public CTREV5Device
{
public:
    CANdleDevice(const std::string &name_space,
                 const int joint_index,
                 const std::string &joint_name,
                 const int candle_id,
                 const std::string &can_bus,
                 const bool local_hardware,
                 const bool local_update);
    CANdleDevice(CANdleDevice &&other) noexcept = delete;
    CANdleDevice(const CANdleDevice &) = delete;
    ~CANdleDevice() override;
    CANdleDevice &operator=(const CANdleDevice &) = delete;
    CANdleDevice &operator=(CANdleDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::candle::CANdleStateInterface &state_interface,
                            hardware_interface::candle::CANdleCommandInterface &command_interface,
                            hardware_interface::candle::RemoteCANdleStateInterface &remote_state_interface) const;
    void write(const ros::Time &/*time*/, const ros::Duration &/*period*/);

private:
    void clearLEDs(size_t start, size_t count);
    const bool local_hardware_;
    const bool local_update_;

    std::unique_ptr<ctre::phoenix::led::CANdle> candle_;

    std::unique_ptr<hardware_interface::candle::CANdleHWState> state_;
    std::unique_ptr<hardware_interface::candle::CANdleHWCommand> command_;
};

#endif