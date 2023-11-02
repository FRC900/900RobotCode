#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ctre_interfaces/orchestra_command_interface.h"
#include "ros_control_boilerplate/talon_orchestra_devices.h"
#include "ros_control_boilerplate/talon_orchestra_device.h"
#include "ros_control_boilerplate/read_config_utils.h"

template<bool SIMFLAG>
TalonOrchestraDevices<SIMFLAG>::TalonOrchestraDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::OrchestraStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::OrchestraCommandInterface>()}
{
	ros::NodeHandle nh(root_nh, "hardware_interface"); // TODO : this shouldn't be hard-coded?
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

        if (joint_type == "orchestra")
        {
            devices_.emplace_back(std::make_unique<TalonOrchestraDevice<SIMFLAG>>(root_nh.getNamespace(), i, joint_name));
        }
    }
}

template <bool SIMFLAG>
TalonOrchestraDevices<SIMFLAG>::~TalonOrchestraDevices() = default;

template <bool SIMFLAG>
hardware_interface::InterfaceManager *TalonOrchestraDevices<SIMFLAG>::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    return &interface_manager_;
}

template <bool SIMFLAG>
void TalonOrchestraDevices<SIMFLAG>::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("talon orchestras");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}

template <bool SIMFLAG>
void TalonOrchestraDevices<SIMFLAG>::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("talon orchestras");
    for (const auto &d : devices_)
    {
        d->write(time, period, talonfxs_);
    }
}

template <bool SIMFLAG>
void TalonOrchestraDevices<SIMFLAG>::setTalonFXData(const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &talonfxs)
{
    talonfxs_ = talonfxs;
}

template class TalonOrchestraDevices<false>;
template class TalonOrchestraDevices<true>;