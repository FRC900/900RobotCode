#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ctre_interfaces/candle_command_interface.h"
#include "ros_control_boilerplate/candle_devices.h"
#include "ros_control_boilerplate/candle_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

CANdleDevices::CANdleDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::candle::CANdleStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::candle::CANdleCommandInterface>()}
    , remote_state_interface_{std::make_unique<hardware_interface::candle::RemoteCANdleStateInterface>()}
{
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

        if (joint_type == "candle")
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
				throw std::runtime_error("A CANdle can_id was specified with local_hardware == false for joint " + joint_name);
            }
            if (local && !has_can_id)
            {
                throw std::runtime_error("A CANdle can_id was not specified for joint " + joint_name);
            }

            std::string can_bus;
            const bool has_can_bus = readStringOptional(joint_params, "can_bus", can_bus, joint_name);
            if (!local && has_can_bus)
            {
				throw std::runtime_error("A CANdle can_bus was specified with local_hardware == false for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<CANdleDevice>(root_nh.getNamespace(), i, joint_name, can_id, can_bus, local_update, local_hardware));
        }
    }
}

CANdleDevices::~CANdleDevices() = default;

hardware_interface::InterfaceManager *CANdleDevices::registerInterface()
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

void CANdleDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("candle");
    for (auto &d : devices_)
    {
        d->write(time, period);
    }
}