#ifndef CANCODER_DEVICES_H_INC__
#define CANCODER_DEVICES_H_INC__

#include <map>
#include "ros_control_boilerplate/devices.h"

template <bool SIM> class CANCoderDevice;
namespace hardware_interface::cancoder
{
    class CANCoderStateInterface;
    class CANCoderCommandInterface;
    class RemoteCANcoderStateInterface;
    class CANCoderSimCommandInterface;
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

template <bool SIM>
class CANCoderDevices : public Devices
{
public:
    explicit CANCoderDevices(ros::NodeHandle &root_nh);
    CANCoderDevices(const CANCoderDevices &) = delete;
    CANCoderDevices(CANCoderDevices &&) noexcept = delete;
    ~CANCoderDevices() override;

    CANCoderDevices &operator=(const CANCoderDevices &) = delete;
    CANCoderDevices &operator=(CANCoderDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const;

    // Sim-only functions below
    void simPostRead(const ros::Time &time, const ros::Duration & /*period*/, Tracer &tracer) override;

private:
    double read_hz_{100};
    std::vector<std::unique_ptr<CANCoderDevice<SIM>>> devices_;
    std::unique_ptr<hardware_interface::cancoder::CANCoderStateInterface> state_interface_{std::make_unique<hardware_interface::cancoder::CANCoderStateInterface>()};
    std::unique_ptr<hardware_interface::cancoder::CANCoderCommandInterface> command_interface_{std::make_unique<hardware_interface::cancoder::CANCoderCommandInterface>()};
    std::unique_ptr<hardware_interface::cancoder::RemoteCANcoderStateInterface> remote_state_interface_{std::make_unique<hardware_interface::cancoder::RemoteCANcoderStateInterface>()};
    std::unique_ptr<hardware_interface::cancoder::CANCoderSimCommandInterface> sim_command_interface_{std::make_unique<hardware_interface::cancoder::CANCoderSimCommandInterface>()};
    hardware_interface::InterfaceManager interface_manager_;
};

#endif