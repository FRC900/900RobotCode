#ifndef ROBOT_CONTROLLER_DEVICES_H_INC__
#define ROBOT_CONTROLLER_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class RobotControllerDevice;
namespace hardware_interface
{
    class RobotControllerStateInterface;
}

class RobotControllerDevices : public Devices
{

public:
    RobotControllerDevices(ros::NodeHandle &root_nh);
    RobotControllerDevices(const RobotControllerDevices &) = delete;
    RobotControllerDevices(RobotControllerDevices &&) noexcept = delete;
    virtual ~RobotControllerDevices();

    RobotControllerDevices &operator=(const RobotControllerDevices &) = delete;
    RobotControllerDevices &operator=(RobotControllerDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    std::vector<std::unique_ptr<RobotControllerDevice>> devices_;
    std::shared_ptr<hardware_interface::RobotControllerStateInterface> state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif