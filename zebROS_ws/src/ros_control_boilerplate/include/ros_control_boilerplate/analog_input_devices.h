#ifndef ANALOG_INPUT_DEVICES_H_INC__
#define ANALOG_INPUT_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class AnalogInputDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class RemoteJointInterface;
}

class AnalogInputDevices : public Devices
{

public:
    explicit AnalogInputDevices(ros::NodeHandle &root_nh);
    AnalogInputDevices(const AnalogInputDevices &) = delete;
    AnalogInputDevices(AnalogInputDevices &&) noexcept = delete;
    ~AnalogInputDevices() override;

    AnalogInputDevices &operator=(const AnalogInputDevices &) = delete;
    AnalogInputDevices &operator=(AnalogInputDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    bool setSimValue(const std::string &name, const size_t index, const double value);

private:
    std::vector<std::unique_ptr<AnalogInputDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif