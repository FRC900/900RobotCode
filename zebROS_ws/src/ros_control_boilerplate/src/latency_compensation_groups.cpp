#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager

#include "ros_control_boilerplate/latency_compensation_group.h"
#include "ros_control_boilerplate/latency_compensation_groups.h"
#include "ros_control_boilerplate/read_config_utils.h"
#include "ctre_interfaces/latency_compensation_state_interface.h"

LatencyCompensationGroups::LatencyCompensationGroups(ros::NodeHandle &root_nh,
                                                     const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices)
    : state_interface_{std::make_unique<hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface>()}
{
    // TODO : this shouldn't be hard-coded?
	ros::NodeHandle nh(root_nh, "hardware_interface");
    XmlRpc::XmlRpcValue joint_param_list;
    if (!nh.getParam("joints", joint_param_list))
    {
        throw std::runtime_error("No joints were specified.");
    }
	for (int i = 0; i < joint_param_list.size(); i++)
	{
		const XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
        std::string joint_name;
        std::string joint_type;
        readStringRequired(joint_params, "name", joint_name);
        readStringRequired(joint_params, "type", joint_type, joint_name);
        if (joint_type == "latency_compensation")
        {
            double update_frequency;
            readDoubleRequired(joint_params, "update_frequency", update_frequency);

            XmlRpc::XmlRpcValue entries_array;
            readArrayRequired(joint_params, "entries", entries_array, joint_name);

            devices_.emplace_back(std::make_unique<LatencyCompensationGroup>(entries_array, joint_name, update_frequency, devices));
        }
    }
}

LatencyCompensationGroups::~LatencyCompensationGroups() = default; 

hardware_interface::InterfaceManager *LatencyCompensationGroups::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    return &interface_manager_;
}

void LatencyCompensationGroups::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("latencycompensation");
    for (const auto &d : devices_)
    {
        d->read();
    }
}
