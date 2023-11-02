
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/digital_input_devices.h"
#include "ros_control_boilerplate/digital_input_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

DigitalInputDevices::DigitalInputDevices(ros::NodeHandle &root_nh)
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

        if (joint_type == "digital_input")
        {
            bool local = true;
            readBooleanOptional(joint_params, "local", local, joint_name);

            int dio_channel = 0;
            const bool has_dio_channel = readIntOptional(joint_params, "dio_channel", dio_channel, joint_name);
			if (!local && has_dio_channel)
            {
				throw std::runtime_error("A Digital Input dio_channel was specified with local_hardware == false for joint " + joint_name);
            }
			if (local)
			{
				if (!has_dio_channel)
                {
					throw std::runtime_error("A Digital Input dio_channel was not specified for joint " + joint_name);
                }

                /*
                auto it = std::find(digital_input_dio_channels_.cbegin(), digital_input_dio_channels_.cend(), digital_input_dio_channel);
                if (it != digital_input_dio_channels_.cend())
                {
                    ROS_WARN_STREAM("A duplicate digital input dio_channel was specified for joint " << joint_name);
                }
                */
            }

			bool invert = false;
            if (readBooleanOptional(joint_params, "invert", invert, joint_name) && !local)
			{
                throw std::runtime_error("A Digital Input joint invert was specified for non-local hardware for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<DigitalInputDevice>(i, joint_name, dio_channel, invert, local));
        }
    }
}

DigitalInputDevices::~DigitalInputDevices() = default;

hardware_interface::InterfaceManager *DigitalInputDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *remote_joint_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(remote_joint_interface_.get());
    return &interface_manager_;
}

void DigitalInputDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("digital inputs");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}

void DigitalInputDevices::simInit(ros::NodeHandle &nh)
{
    sim_srv_ = nh.advertiseService("linebreak_service_set", &DigitalInputDevices::simCallback, this);
}

// Set the HAL sim value for the requested device
// If name is set, set that named digital input. Else set the index-th entry in devices_
bool DigitalInputDevices::simCallback(ros_control_boilerplate::LineBreakSensors::Request &req,
                                      ros_control_boilerplate::LineBreakSensors::Response & /*res*/)
{
    if (req.name.size())
    {
        for (const auto &d : devices_)
        {
            if (d->getName() == req.name)
            {
                d->setSimValue(req.value);
                return true;
            }
        }
        ROS_INFO_STREAM(req.name << " not set to " << static_cast<int>(req.value) << " : name not found in device list");
        return false;
    }
    if (req.j >= devices_.size())
    {
        ROS_INFO_STREAM("Digital input index "<< req.j << " not set to " << static_cast<int>(req.value) << " : index out of bounds");
        return false;
    }
    devices_[req.j]->setSimValue(req.value);
    return true;
}
