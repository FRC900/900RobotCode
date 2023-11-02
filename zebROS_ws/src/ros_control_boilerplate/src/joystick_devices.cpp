
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager

#include "frc_interfaces/joystick_interface.h"
#include "ros_control_boilerplate/joystick_devices.h"
#include "ros_control_boilerplate/joystick_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

template <class DEVICE_TYPE>
JoystickDevices<DEVICE_TYPE>::JoystickDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_shared<hardware_interface::JoystickStateInterface>()}
{
 	ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
	if(!param_nh.param("joystick_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read joystick_read_hz in frc_robot_interface");
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

        if (joint_type == "joystick")
        {
            int id;
            readIntRequired(joint_params, "id", id, joint_name);

            devices_.emplace_back(std::make_unique<DEVICE_TYPE>(i, joint_name, id, read_hz_));
        }
    }
}

template <class DEVICE_TYPE>
JoystickDevices<DEVICE_TYPE>::~JoystickDevices() = default;

template <class DEVICE_TYPE>
hardware_interface::InterfaceManager *JoystickDevices<DEVICE_TYPE>::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    return &interface_manager_;
}

template <class DEVICE_TYPE>
void JoystickDevices<DEVICE_TYPE>::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("joysticks");
    if (isReady())
    {
        for (auto &d : devices_)
        {
            d->read(time, period);
        }
    }
}

template <class DEVICE_TYPE>
void JoystickDevices<DEVICE_TYPE>::simInit(ros::NodeHandle &nh)
{
    for (size_t i = 0; i < devices_.size(); i++)
    {
        devices_[i]->simInit(nh, i);
    }
}