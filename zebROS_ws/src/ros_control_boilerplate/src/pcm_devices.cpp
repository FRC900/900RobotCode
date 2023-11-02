
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include <hardware_interface/joint_command_interface.h>
#include "frc_interfaces/pcm_state_interface.h"
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/pcm_devices.h"
#include "ros_control_boilerplate/pcm_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

PCMDevices::PCMDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::JointStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::JointCommandInterface>()}
    , position_joint_interface_{std::make_unique<hardware_interface::PositionJointInterface>()}
    , remote_joint_interface_{std::make_unique<hardware_interface::RemoteJointInterface>()}
    , pcm_state_interface_{std::make_unique<hardware_interface::PCMStateInterface>()}
    , remote_pcm_state_interface_{std::make_unique<hardware_interface::RemotePCMStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("pcm_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read pcm_read_hz in frc_robot_interface");
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

        if (joint_type == "pcm")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_update;
            bool local_hardware;
            int pcm_id = 0;
            readJointLocalParams(joint_params, joint_name, local, saw_local_keyword, local_update, local_hardware);
            const bool has_pcm_id = readIntOptional(joint_params, "pcm_id", pcm_id, joint_name);
            if (!local && has_pcm_id)
            {
				throw std::runtime_error("A PCM pcm_id was specified with local_hardware == false for joint " + joint_name);
            }
			if (local && !has_pcm_id)
			{
                throw std::runtime_error("A PCM pcm_id was not specified for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<PCMDevice>(root_nh.getNamespace(), i, joint_name, pcm_id, local_update, local_hardware, read_hz_));
        }
    }
}

PCMDevices::~PCMDevices() = default;

hardware_interface::InterfaceManager *PCMDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_, *position_joint_interface_, *remote_joint_interface_, *pcm_state_interface_, *remote_pcm_state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    interface_manager_.registerInterface(position_joint_interface_.get());
    interface_manager_.registerInterface(remote_joint_interface_.get());
    interface_manager_.registerInterface(pcm_state_interface_.get());
    interface_manager_.registerInterface(remote_pcm_state_interface_.get());
    return &interface_manager_;
}

void PCMDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PCMs");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}

void PCMDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("PCMs");
    for (const auto &d : devices_)
    {
        d->write(time, period);
    }
}