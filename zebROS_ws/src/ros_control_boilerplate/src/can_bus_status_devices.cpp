#include "ros_control_boilerplate/read_config_utils.h"
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "frc_interfaces/can_bus_status_interface.h"
#include "ros_control_boilerplate/can_bus_status_device.h"
#include "ros_control_boilerplate/can_bus_status_devices.h"

CANBusStatusDevices::CANBusStatusDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::can_bus_status::CANBusStatusStateInterface>()}
{
 	ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    double read_hz{5};
	if(!param_nh.param("can_bus_status_read_hz", read_hz, read_hz)) 
    {
		ROS_WARN("Failed to read can_bus_status_read_hz in frc_robot_interface");
	}
    ros::NodeHandle nh(root_nh, "hardware_interface"); // TODO : this shouldn't be hard-coded?
    XmlRpc::XmlRpcValue joint_param_list;
    if (!nh.getParam("joints", joint_param_list))
    {
        throw std::runtime_error("No joints were specified->");
    }
	for (int i = 0; i < joint_param_list.size(); i++)
	{
		const XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
        std::string joint_name;
        std::string joint_type;
        readStringRequired(joint_params, "name", joint_name);
        readStringRequired(joint_params, "type", joint_type, joint_name);

        if (joint_type == "can_bus_status")
        {
            devices_.emplace_back(std::make_unique<CANBusStatusDevice>(joint_name, read_hz));
        }
    }
}

CANBusStatusDevices::~CANBusStatusDevices() = default;

hardware_interface::InterfaceManager *CANBusStatusDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    return &interface_manager_;
}

void CANBusStatusDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("CAN bus status");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}