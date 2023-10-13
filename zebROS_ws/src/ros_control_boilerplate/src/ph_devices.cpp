#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "frc_interfaces/ph_command_interface.h"
#include "ros_control_boilerplate/ph_devices.h"
#include "ros_control_boilerplate/ph_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

PHDevices::PHDevices(ros::NodeHandle &root_nh)
    : ph_state_interface_{std::make_unique<hardware_interface::PHStateInterface>()}
    , ph_command_interface_{std::make_unique<hardware_interface::PHCommandInterface>()}
    , remote_ph_state_interface_{std::make_unique<hardware_interface::RemotePHStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("ph_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read ph_read_hz in frc_robot_interface");
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

        if (joint_type == "ph")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_read;
            bool local_write;
            readJointLocalReadWriteParams(joint_params, joint_name, local, saw_local_keyword, local_read, local_write);

            int ph_id = 0;
            const bool has_ph_id = readIntOptional(joint_params, "ph_id", ph_id, joint_name);
            if (!local_read && !local_write && has_ph_id)
            {
				throw std::runtime_error("A PH ph_id was specified with local read and write == false for joint " + joint_name);
            }
			if ((local_read || local_write) && !has_ph_id)
            {
                throw std::runtime_error("A PH ph_id was not specified for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<PHDevice>(root_nh.getNamespace(), i, joint_name, ph_id, local_read, local_write, read_hz_));
        }
    }
}

PHDevices::~PHDevices() = default;

hardware_interface::InterfaceManager *PHDevices::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*ph_state_interface_, *ph_command_interface_, *remote_ph_state_interface_);
    }
    interface_manager_.registerInterface(ph_state_interface_.get());
    interface_manager_.registerInterface(ph_command_interface_.get());
    interface_manager_.registerInterface(remote_ph_state_interface_.get());
    return &interface_manager_;
}

void PHDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PHs");
    for (auto &d : devices_)
    {
        d->read(time, period);
    }
}

void PHDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PHs");
    for (auto &d : devices_)
    {
        d->write(time, period);
    }
}