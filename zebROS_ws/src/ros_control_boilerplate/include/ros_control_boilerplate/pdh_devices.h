#ifndef PDH_DEVICES_H_INC__
#define PDH_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class PDHDevice;
namespace hardware_interface
{
    class PDHStateInterface;
    class PDHCommandInterface;
    class RemotePDHStateInterface;
}

class PDHDevices : public Devices
{

public:
    PDHDevices(ros::NodeHandle &root_nh);
    PDHDevices(const PDHDevices &) = delete;
    PDHDevices(PDHDevices &&) noexcept = delete;
    virtual ~PDHDevices();

    PDHDevices &operator=(const PDHDevices &) = delete;
    PDHDevices &operator=(PDHDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    double read_hz_{20};
    std::vector<std::unique_ptr<PDHDevice>> devices_;
    std::unique_ptr<hardware_interface::PDHStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::PDHCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::RemotePDHStateInterface> remote_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif