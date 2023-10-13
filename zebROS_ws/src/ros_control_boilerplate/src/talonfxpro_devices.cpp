#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

#include "ros_control_boilerplate/talonfxpro_devices.h"
#include "ros_control_boilerplate/talonfxpro_device.h"
#include "ros_control_boilerplate/read_config_utils.h"
#include "ctre_interfaces/talonfxpro_command_interface.h"

TalonFXProDevices::TalonFXProDevices(ros::NodeHandle &root_nh) 
    : state_interface_{std::make_unique<hardware_interface::talonfxpro::TalonFXProStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::talonfxpro::TalonFXProCommandInterface>()}
{
	ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop");
	if(!param_nh.param("talonfxpro_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read talonfxpro_read_hz in frc_robot_interface");
	}
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

        if (joint_type == "talonfxpro")
        {
            int can_id = 0;
            readIntRequired(joint_params, "can_id", can_id, joint_name);
            std::string can_bus;
            readStringRequired(joint_params, "can_bus", can_bus, joint_name);

            devices_.emplace_back(std::make_unique<TalonFXProDevice>(nh.getNamespace(), i, joint_name, can_id, can_bus, read_hz_));
        }
    }
}

TalonFXProDevices::~TalonFXProDevices() = default; 

// Calling code needs to call something like
//  registerInterfaceManager(devices->registerInterface());
hardware_interface::InterfaceManager *TalonFXProDevices::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    return &interface_manager_;
}

void TalonFXProDevices::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("talonfxpro");
    for (auto &d : devices_)
    {
        d->read(time, period);
    }
}

void TalonFXProDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("talonfxpro");
    for (auto &d : devices_)
    {
        d->write(time, period, isEnabled(), prev_robot_enabled_);
    }
    prev_robot_enabled_ = isEnabled();
}

void TalonFXProDevices::simInit(ros::NodeHandle nh)
{
    if (devices_.size() > 0)
    {
        sim_limit_switch_srv_ = nh.advertiseService("set_talonfxpro_limit_switch", &TalonFXProDevices::setlimit, this);
        sim_current_srv_ = nh.advertiseService("set_talonfxpro_current", &TalonFXProDevices::setcurrent, this);
    }
}

void TalonFXProDevices::simRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("talonfxpro FeedEnable");
    if (devices_.size() > 0)
    {
        ctre::phoenix::unmanaged::FeedEnable(2. * 1000. / read_hz_);
    }
    tracer.start_unique("talonfxpro sim");
    for (auto &d : devices_)
    {
        d->simRead(time, period);
    }
}

bool TalonFXProDevices::setlimit(ros_control_boilerplate::set_limit_switch::Request &req,
                                 ros_control_boilerplate::set_limit_switch::Response & /*res*/)
{
    for (auto &d : devices_)
    {
        if (((req.target_joint_name.length() == 0) && (d->getCANID() == req.target_joint_id)) ||
            (req.target_joint_name == d->getName()))
        {
            return d->setSimLimitSwitches(req.forward, req.reverse);
        }
    }
    return true;
}

bool TalonFXProDevices::setcurrent(ros_control_boilerplate::set_current::Request &req,
                                   ros_control_boilerplate::set_current::Response & /*res*/)
{
    for (auto &d : devices_)
	{
        if(((req.target_joint_name.length() == 0) && (d->getCANID() == req.target_joint_id)) ||
		   (req.target_joint_name == d->getName()))
		{
            return d->setSimCurrent(req.current, req.current);
        }
    }
	return true;
}

void TalonFXProDevices::appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const
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
