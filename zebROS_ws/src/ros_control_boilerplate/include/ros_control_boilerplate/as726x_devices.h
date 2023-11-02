#ifndef AS726X_DEVICES_H_INC__
#define AS726X_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

template <bool SIMFLAG> class AS726xDevice;
namespace hardware_interface::as726x
{
    class AS726xStateInterface;
    class AS726xCommandInterface;
    class RemoteAS726xStateInterface;
}

template <bool SIMFLAG>
class AS726xDevices : public Devices
{
public:
    explicit AS726xDevices(ros::NodeHandle &root_nh);
    AS726xDevices(const AS726xDevices &) = delete;
    AS726xDevices(AS726xDevices &&) = delete;
    virtual ~AS726xDevices();

    AS726xDevices &operator=(const AS726xDevices &) = delete;
    AS726xDevices &operator=(AS726xDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    double read_hz_{7};
    std::vector<std::unique_ptr<AS726xDevice<SIMFLAG>>> devices_;
    std::unique_ptr<hardware_interface::as726x::AS726xStateInterface> as726x_state_interface_;
    std::unique_ptr<hardware_interface::as726x::AS726xCommandInterface> as726x_command_interface_;
    std::unique_ptr<hardware_interface::as726x::RemoteAS726xStateInterface> remote_as726x_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

using HWAS726xDevices = AS726xDevices<false>;
using SimAS726xDevices = AS726xDevices<true>;

#endif