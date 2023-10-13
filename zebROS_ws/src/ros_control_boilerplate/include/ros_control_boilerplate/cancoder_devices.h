#ifndef CANCODER_DEVICES_H_INC__
#define CANCODER_DEVICES_H_INC__

#include <map>
#include "ros_control_boilerplate/devices.h"

class CANCoderDevice;
namespace hardware_interface::cancoder
{
    class CANCoderStateInterface;
    class CANCoderCommandInterface;
    class RemoteCANCoderStateInterface;
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

class CANCoderDevices : public Devices
{
public:
    CANCoderDevices(ros::NodeHandle &root_nh);
    CANCoderDevices(const CANCoderDevices &) = delete;
    CANCoderDevices(CANCoderDevices &&) noexcept = delete;
    virtual ~CANCoderDevices();

    CANCoderDevices &operator=(const CANCoderDevices &) = delete;
    CANCoderDevices &operator=(CANCoderDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const;

private:
    double read_hz_{100};
    std::vector<std::unique_ptr<CANCoderDevice>> devices_;
    std::unique_ptr<hardware_interface::cancoder::CANCoderStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::cancoder::CANCoderCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::cancoder::RemoteCANCoderStateInterface> remote_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif