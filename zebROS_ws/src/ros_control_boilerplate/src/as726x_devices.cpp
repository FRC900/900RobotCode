#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "as726x_interface/as726x_interface.h"
#include "ros_control_boilerplate/as726x_devices.h"
#include "ros_control_boilerplate/as726x_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

template<bool SIMFLAG>
AS726xDevices<SIMFLAG>::AS726xDevices(ros::NodeHandle &root_nh)
    : as726x_state_interface_{std::make_unique<hardware_interface::as726x::AS726xStateInterface>()}
    , as726x_command_interface_{std::make_unique<hardware_interface::as726x::AS726xCommandInterface>()}
    , remote_as726x_state_interface_{std::make_unique<hardware_interface::as726x::RemoteAS726xStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("as726x_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read as726x_read_hz in frc_robot_interface");
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

        if (joint_type == "as726x")
        {
            bool local = true;
            const bool saw_local_keyword = readBooleanOptional(joint_params, "local", local, joint_name);
            bool local_update;
            bool local_hardware;
            readJointLocalParams(joint_params, joint_name, local, saw_local_keyword, local_update, local_hardware);
            std::string port;
            const bool has_port = readStringOptional(joint_params, "port", port, joint_name);
            if (!local && has_port)
            {
				throw std::runtime_error("A AS726x port was specified with local_hardware == false for joint " + joint_name);
            }
			if (local)
			{
				if (!has_port)
                {
					throw std::runtime_error("A AS726x port was not specified for joint " + joint_name);
                }

                /*
                auto it = std::find(as726x_ports_.cbegin(), as726x_ports_.cend(), as726x_port);
                if (it != as726x_ports_.cend())
                {
                    ROS_WARN_STREAM("A duplicate digital output port was specified for joint " << joint_name);
                }
                */
            }
            int address = 0;
            const bool has_address = readIntOptional(joint_params, "address", address, joint_name);
            if (!local && has_address)
            {
				throw std::runtime_error("A AS726x address was specified with local_hardware == false for joint " + joint_name);
            }
			if (local)
			{
				if (!has_address)
                {
					throw std::runtime_error("A AS726x address was not specified for joint " + joint_name);
                }

                /*
                auto it = std::find(as726x_addresss_.cbegin(), as726x_addresss_.cend(), as726x_address);
                if (it != as726x_addresss_.cend())
                {
                    ROS_WARN_STREAM("A duplicate digital output address was specified for joint " << joint_name);
                }
                */
            }

            devices_.emplace_back(std::make_unique<AS726xDevice<SIMFLAG>>(root_nh.getNamespace(), i, joint_name, port, address, local_update, local_hardware, read_hz_));
        }
    }
}

template <bool SIMFLAG>
AS726xDevices<SIMFLAG>::~AS726xDevices() = default;

template <bool SIMFLAG>
hardware_interface::InterfaceManager *AS726xDevices<SIMFLAG>::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*as726x_state_interface_, *as726x_command_interface_, *remote_as726x_state_interface_);
    }
    interface_manager_.registerInterface(as726x_state_interface_.get());
    interface_manager_.registerInterface(as726x_command_interface_.get());
    interface_manager_.registerInterface(remote_as726x_state_interface_.get());
    return &interface_manager_;
}

template <bool SIMFLAG>
void AS726xDevices<SIMFLAG>::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("AS726xs");
    for (auto &d : devices_)
    {
        d->read(time, period);
    }
}

template <bool SIMFLAG>
void AS726xDevices<SIMFLAG>::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("AS726xs");
    for (auto &d : devices_)
    {
        d->write(time, period);
    }
}