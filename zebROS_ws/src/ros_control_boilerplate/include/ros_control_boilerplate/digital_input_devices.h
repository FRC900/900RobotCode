#ifndef DIGITAL_INPUT_DEVICES_H_INC__
#define DIGITAL_INPUT_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"
#include "ros_control_boilerplate/LineBreakSensors.h"

class DigitalInputDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class RemoteJointInterface;
}

class DigitalInputDevices : public Devices
{

public:
    DigitalInputDevices(ros::NodeHandle &root_nh);
    DigitalInputDevices(const DigitalInputDevices &) = delete;
    DigitalInputDevices(DigitalInputDevices &&) noexcept = delete;
    virtual ~DigitalInputDevices();

    DigitalInputDevices &operator=(const DigitalInputDevices &) = delete;
    DigitalInputDevices &operator=(DigitalInputDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void simInit(ros::NodeHandle nh) override;

private:
    std::vector<std::unique_ptr<DigitalInputDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    hardware_interface::InterfaceManager interface_manager_;

    ros::ServiceServer sim_srv_;
    bool simCallback(ros_control_boilerplate::LineBreakSensors::Request &req, ros_control_boilerplate::LineBreakSensors::Response &res);
};

#endif