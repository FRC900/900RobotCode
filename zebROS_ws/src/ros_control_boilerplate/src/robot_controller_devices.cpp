
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "frc_interfaces/robot_controller_interface.h"
#include "ros_control_boilerplate/robot_controller_devices.h"
#include "ros_control_boilerplate/robot_controller_device.h"

RobotControllerDevices::RobotControllerDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_shared<hardware_interface::RobotControllerStateInterface>()}
{
    devices_.emplace_back(std::make_unique<RobotControllerDevice>(root_nh));
}

RobotControllerDevices::~RobotControllerDevices() = default;

hardware_interface::InterfaceManager *RobotControllerDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    return &interface_manager_;
}

void RobotControllerDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("robot controller data");
    if (isReady())
    {
        for (const auto &d : devices_)
        {
            d->read(time, period);
        }
    }
}