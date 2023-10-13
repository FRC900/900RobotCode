#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "frc_interfaces/pdh_command_interface.h"
#include "ros_control_boilerplate/pdh_devices.h"
#include "ros_control_boilerplate/pdh_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

PDHDevices::PDHDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::PDHStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::PDHCommandInterface>()}
    , remote_state_interface_{std::make_unique<hardware_interface::RemotePDHStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("pdh_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read pdh_read_hz in frc_robot_interface");
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

        if (joint_type == "pdh")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_read;
            bool local_write;
            readJointLocalReadWriteParams(joint_params, joint_name, local, saw_local_keyword, local_read, local_write);
            int module = 0;
            const bool has_module = readIntOptional(joint_params, "module", module, joint_name);
            if (!local_read && !local_write && has_module)
            {
				throw std::runtime_error("A PDH module was specified with local read and write == false for joint " + joint_name);
            }
			if ((local_read || local_write) && !has_module)
            {
                throw std::runtime_error("A PDH module was not specified for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<PDHDevice>(root_nh.getNamespace(), i, joint_name, module, local_read, local_write, read_hz_));
        }
    }
}

PDHDevices::~PDHDevices() = default;

hardware_interface::InterfaceManager *PDHDevices::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_, *remote_state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    interface_manager_.registerInterface(remote_state_interface_.get());
    return &interface_manager_;
}

void PDHDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PDHs");
    for (auto &d : devices_)
    {
        d->read(time, period);
    }
}

void PDHDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PDHs");
    for (auto &d : devices_)
    {
        d->write(time, period);
    }
}