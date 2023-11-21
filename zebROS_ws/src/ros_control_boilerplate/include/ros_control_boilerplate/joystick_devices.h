#ifndef JOYSTICK_DEVICES_H_INC__
#define JOYSTICK_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

namespace hardware_interface
{
    class JoystickStateInterface;
}

class JoystickDevice;
class SimJoystickDevice;

template <bool SIM>
class JoystickDevices : public Devices
{
public:
    explicit JoystickDevices(ros::NodeHandle &root_nh);
    JoystickDevices(const JoystickDevices &) = delete;
    JoystickDevices(JoystickDevices &&) noexcept = delete;
    virtual ~JoystickDevices();

    JoystickDevices &operator=(const JoystickDevices &) = delete;
    JoystickDevices &operator=(JoystickDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void simInit(ros::NodeHandle &nh) override;

private:
    double read_hz_{50};
    using DEVICE_TYPE = std::conditional_t<SIM, SimJoystickDevice, JoystickDevice>;
    std::vector<std::unique_ptr<DEVICE_TYPE>> devices_;
    std::shared_ptr<hardware_interface::JoystickStateInterface> state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif