#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ctre_interfaces/cancoder_command_interface.h"
#include "ros_control_boilerplate/cancoder_devices.h"
#include "ros_control_boilerplate/cancoder_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

CANCoderDevices::CANCoderDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::cancoder::CANCoderStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::cancoder::CANCoderCommandInterface>()}
    , remote_state_interface_{std::make_unique<hardware_interface::cancoder::RemoteCANcoderStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("cancoder_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read cancoder_read_hz in frc_robot_interface");
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

        if (joint_type == "cancoder")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_update;
            bool local_hardware;
            readJointLocalParams(joint_params, joint_name, local, saw_local_keyword, local_update, local_hardware);

            int can_id = 0;
            const bool has_can_id = readIntOptional(joint_params, "can_id", can_id, joint_name);
            if (!local && has_can_id)
            {
				throw std::runtime_error("A CANCoder can_id was specified with local_hardware == false for joint " + joint_name);
            }
			if (local && !has_can_id)
			{
                throw std::runtime_error("A CANCoder can_id was not specified for joint " + joint_name);
            }

            std::string can_bus;
            const bool has_can_bus = readStringOptional(joint_params, "can_bus", can_bus, joint_name);
            if (!local && has_can_bus)
            {
				throw std::runtime_error("A CANCoder can_bus was specified with local_hardware == false for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<CANCoderDevice>(root_nh.getNamespace(), i, joint_name, can_id, can_bus, local_update, local_hardware, read_hz_));
        }
    }
}

CANCoderDevices::~CANCoderDevices() = default;

hardware_interface::InterfaceManager *CANCoderDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_, *remote_state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    interface_manager_.registerInterface(remote_state_interface_.get());
    return &interface_manager_;
}

void CANCoderDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("cancoder");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}

void CANCoderDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("cancoder");
    for (const auto &d : devices_)
    {
        d->write(time, period);
    }
}

void CANCoderDevices::appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const
{
    for (auto &d : devices_)
    {
        const auto ptr = d->getParentDevice();
        if (ptr)
        {
            device_map.emplace(d->getName(), ptr);
        }
    }
}
