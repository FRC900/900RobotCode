#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ctre_interfaces/talon_command_interface.h"
#include "ros_control_boilerplate/ctre_v5_motor_controllers.h"
#include "ros_control_boilerplate/ctre_v5_motor_controller.h"
#include "ros_control_boilerplate/read_config_utils.h"

CTREV5MotorControllers::CTREV5MotorControllers(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::TalonStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::TalonCommandInterface>()}
    , remote_state_interface_{std::make_unique<hardware_interface::RemoteTalonStateInterface>()}
{
    ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop"); // TODO : this shouldn't be hard-coded?
    if(!param_nh.param("ctre_mc_read_hz", read_hz_, read_hz_)) 
    {
		ROS_WARN("Failed to read ctre_mc_read_hz in frc_robot_interface");
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

		if ((joint_type == "can_talon_srx") || (joint_type == "can_victor_spx") || (joint_type == "can_talon_fx"))
        {
            bool local = true;
            readBooleanOptional(joint_params, "local", local, joint_name);

            int can_id = 0;
            const bool has_can_id = readIntOptional(joint_params, "can_id", can_id, joint_name);
            if (!local && has_can_id)
            {
                throw std::runtime_error("A CAN Talon SRX / Victor SPX / Talon FX can_id was specified with local_hardware == false for joint " + joint_name);
            }
            if (local && !has_can_id)
            {
                throw std::runtime_error("A CAN Talon SRX / Victor SPX / TalonFX can_id was not specified for joint " + joint_name);
            }

            std::string can_bus;
            const bool has_can_bus = readStringOptional(joint_params, "can_bus", can_bus, joint_name);
            if (!local && has_can_bus)
            {
                throw std::runtime_error("A CAN Talon SRX / Victor SPX / Talon FX can_bus was specified with local_hardware == false for joint " + joint_name);
            }

            devices_.emplace_back(std::make_unique<CTREV5MotorController>(root_nh.getNamespace(), i, joint_name, joint_type, can_id, can_bus, local, read_hz_));
        }
    }
}

CTREV5MotorControllers::~CTREV5MotorControllers() = default;

hardware_interface::InterfaceManager *CTREV5MotorControllers::registerInterface()
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

void CTREV5MotorControllers::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("ctre v5 motor controllers");
    for (auto &d : devices_)
    {
        d->read(time, period);
    }
}

void CTREV5MotorControllers::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("ctre v5 motor controllers");
    CTREV5MotorController::resetCanConfigCount();
    for (auto &d : devices_)
    {
        d->write(time, period, isEnabled(), prev_robot_enabled_);
    }
    prev_robot_enabled_ = isEnabled();
}

void CTREV5MotorControllers::simInit(ros::NodeHandle nh)
{
    if (devices_.size() > 0)
    {
        sim_limit_switch_srv_ = nh.advertiseService("set_limit_switch", &CTREV5MotorControllers::setlimit, this);
        sim_current_srv_ = nh.advertiseService("set_current", &CTREV5MotorControllers::setcurrent, this);
    }
}

bool CTREV5MotorControllers::setlimit(ros_control_boilerplate::set_limit_switch::Request &req,
                             ros_control_boilerplate::set_limit_switch::Response & /*res*/)
{
    for (auto &d : devices_)
	{
        if(((req.target_joint_name.length() == 0) && (d->getId() == req.target_joint_id)) ||
		   (req.target_joint_name == d->getName()))
		{
            return d->setSimLimitSwitches(req.forward, req.reverse);
        }
    }
	return true;
}

bool CTREV5MotorControllers::setcurrent(ros_control_boilerplate::set_current::Request &req,
                               ros_control_boilerplate::set_current::Response & /*res*/)
{
    for (auto &d : devices_)
	{
        if(((req.target_joint_name.length() == 0) && (d->getId() == req.target_joint_id)) ||
		   (req.target_joint_name == d->getName()))
		{
            return d->setSimCurrent(req.current, req.current);
        }
    }
	return true;
}

void CTREV5MotorControllers::simRead(const ros::Time &time, const ros::Duration &period, Tracer &tracer)
{
	tracer.start_unique("FeedEnable");
    if (devices_.size() > 0)
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(2 * 1000. / read_hz_);
    }
    tracer.start_unique("Update sim CTRE values");
    for (auto &d : devices_)
    {
        d->updateSimValues(time, period);
    }
}