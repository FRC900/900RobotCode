#ifndef PDP_DEVICES_H_INC__
#define PDP_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class PDPDevice;
namespace hardware_interface
{
    class PDPStateInterface;
    class RemotePDPStateInterface;
}

class PDPDevices : public Devices
{

public:
    explicit PDPDevices(ros::NodeHandle &root_nh);
    PDPDevices(const PDPDevices &) = delete;
    PDPDevices(PDPDevices &&) noexcept = delete;
    ~PDPDevices() override;

    PDPDevices &operator=(const PDPDevices &) = delete;
    PDPDevices &operator=(PDPDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    double read_hz_{20};
    std::vector<std::unique_ptr<PDPDevice>> devices_;
    std::shared_ptr<hardware_interface::PDPStateInterface> state_interface_;
    std::shared_ptr<hardware_interface::RemotePDPStateInterface> remote_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif