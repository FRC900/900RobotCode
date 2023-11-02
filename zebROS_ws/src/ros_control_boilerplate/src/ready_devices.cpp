#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include <hardware_interface/joint_command_interface.h>
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/ready_devices.h"
#include "ros_control_boilerplate/ready_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

ReadyDevices::ReadyDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::JointStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::JointCommandInterface>()}
    , remote_joint_interface_{std::make_unique<hardware_interface::RemoteJointInterface>()}
{
    // TODO : this shouldn't be hard-coded?
	ros::NodeHandle nh(root_nh, "hardware_interface");
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

        if (joint_type == "ready")
        {
            bool local;
            readBooleanRequired(joint_params, "local", local, joint_name);
            devices_.emplace_back(std::make_unique<ReadyDevice>(i, joint_name, local));
        }
    }
}

ReadyDevices::~ReadyDevices() = default;

hardware_interface::InterfaceManager *ReadyDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_, *remote_joint_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    interface_manager_.registerInterface(remote_joint_interface_.get());
    return &interface_manager_;
}

bool ReadyDevices::areReady(void) const
{
    return std::all_of(devices_.cbegin(),
                       devices_.cend(),
                       [](const std::unique_ptr<ReadyDevice> &d) { return d->isReady(); });
}