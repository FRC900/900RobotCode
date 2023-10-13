
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/analog_input_devices.h"
#include "ros_control_boilerplate/analog_input_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

AnalogInputDevices::AnalogInputDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::JointStateInterface>()}
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

        if (joint_type == "analog_input")
        {
            bool local = true;
            readBooleanOptional(joint_params, "local", local, joint_name);

            int ain_channel = 0;
            const bool has_ain_channel = readIntOptional(joint_params, "ain_channel", ain_channel, joint_name);
			if (!local && has_ain_channel)
            {
				throw std::runtime_error("A Analog Input ain_channel was specified with local_hardware == false for joint " + joint_name);
            }
			if (local)
			{
				if (!has_ain_channel)
                {
					throw std::runtime_error("A Analog Input ain_channel was not specified for joint " + joint_name);
                }

                /*
                auto it = std::find(analog_input_ain_channels_.cbegin(), analog_input_ain_channels_.cend(), analog_input_ain_channel);
                if (it != analog_input_ain_channels_.cend())
                {
                    ROS_WARN_STREAM("A duplicate analog input ain_channel was specified for joint " << joint_name);
                }
                */
            }

            double a = 1.0;
            if (readDoubleOptional(joint_params, "analog_a", a, joint_name) && !local)
			{
                throw std::runtime_error("An Analog Input joint analog_a value was specified for non-local hardware for joint " + joint_name);
            }
            double b = 0.0;
            if (readDoubleOptional(joint_params, "analog_b", b, joint_name) && !local)
			{
                throw std::runtime_error("An Analog Input joint analog_b value was specified for non-local hardware for joint " + joint_name);
            }
            devices_.emplace_back(std::make_unique<AnalogInputDevice>(i, joint_name, ain_channel, a, b, local));
        }
    }
}

AnalogInputDevices::~AnalogInputDevices() = default;

hardware_interface::InterfaceManager *AnalogInputDevices::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *remote_joint_interface_);
    }
    interface_manager_.registerInterface(&(*state_interface_));
    interface_manager_.registerInterface(&(*remote_joint_interface_));
    return &interface_manager_;
}

void AnalogInputDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("analog inputs");
    for (auto &d : devices_)
    {
        d->read(time, period);
    }
}

// Set the HAL sim value for the requested device
// If name is set, set that named analog input. Else set the index-th entry in devices_
bool AnalogInputDevices::setSimValue(const std::string &name, const size_t index, const double value)
{
    if (name.size())
    {
        for (auto &d : devices_)
        {
            if (d->getName() == name)
            {
                d->setSimValue(value);
                return true;
            }
        }
        ROS_INFO_STREAM(name << " not set to " << static_cast<int>(value) << " : name not found in device list");
        return false;
    }
    if (index >= devices_.size())
    {
        ROS_INFO_STREAM("Analog input index "<< index << " not set to " << static_cast<int>(value) << " : index out of bounds");
        return false;
    }
    devices_[index]->setSimValue(value);
    return true;
}
