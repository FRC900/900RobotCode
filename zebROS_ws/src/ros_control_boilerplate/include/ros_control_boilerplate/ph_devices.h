#ifndef PH_DEVICES_H_INC__
#define PH_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class PHDevice;
namespace hardware_interface
{
    class PHStateInterface;
    class PHCommandInterface;
    class RemotePHStateInterface;
}

class PHDevices : public Devices
{

public:
    PHDevices(ros::NodeHandle &root_nh);
    PHDevices(const PHDevices &) = delete;
    PHDevices(PHDevices &&) noexcept = delete;
    virtual ~PHDevices();

    PHDevices &operator=(const PHDevices &) = delete;
    PHDevices &operator=(PHDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    double read_hz_{20};
    std::vector<std::unique_ptr<PHDevice>> devices_;
    std::unique_ptr<hardware_interface::PHStateInterface> ph_state_interface_;
    std::unique_ptr<hardware_interface::PHCommandInterface> ph_command_interface_;
    std::unique_ptr<hardware_interface::RemotePHStateInterface> remote_ph_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif