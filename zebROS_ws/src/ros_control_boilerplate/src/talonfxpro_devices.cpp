#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

#include "ros_control_boilerplate/talonfxpro_devices.h"
#include "ros_control_boilerplate/sim_talonfxpro_device.h"
#include "ros_control_boilerplate/read_config_utils.h"
#include "ctre_interfaces/talonfxpro_command_interface.h"
#include "ctre_interfaces/talonfxpro_sim_command_interface.h"
#include "ctre_interfaces/cancoder_sim_command_interface.h"
#include "frc/simulation/BatterySim.h"

template <bool SIM>
TalonFXProDevices<SIM>::TalonFXProDevices(ros::NodeHandle &root_nh) 
    : state_interface_{std::make_unique<hardware_interface::talonfxpro::TalonFXProStateInterface>()}
    , command_interface_{std::make_unique<hardware_interface::talonfxpro::TalonFXProCommandInterface>()}
{
	ros::NodeHandle param_nh(root_nh, "generic_hw_control_loop");
    if constexpr (SIM)
    {
        // turns out if simulator timesteps are different from motor controller read timesteps (which sim uses) you get problems...
        // that took hours to find out :'(
        ROS_INFO_STREAM("We are running in simulation. Setting talonfxpro_read_hz to loop_hz to avoid simulator issues.");
        if(!param_nh.param("loop_hz", read_hz_, read_hz_)) 
        {
            ROS_WARN("Failed to read loop_hz in frc_robot_interface");
        }
    }
    else
    {
        if(!param_nh.param("talonfxpro_read_hz", read_hz_, read_hz_)) 
        {
            ROS_WARN("Failed to read talonfxpro_read_hz in frc_robot_interface");
        }
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
            std::string simulator;
            readStringOptional(joint_params, "simulator", simulator, joint_name);

            XmlRpc::XmlRpcValue simulator_info;

            devices_.emplace_back(std::make_unique<DEVICE_TYPE>(nh.getNamespace(), i, joint_name, can_id, can_bus, read_hz_));
        }
    }
}

template <bool SIM>
TalonFXProDevices<SIM>::~TalonFXProDevices() = default; 

// Calling code needs to call something like
//  registerInterfaceManager(devices->registerInterface());
template <bool SIM>
hardware_interface::InterfaceManager *TalonFXProDevices<SIM>::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *command_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(command_interface_.get());
    if constexpr (SIM)
    {
        for (const auto &d : devices_)
        {
            d->registerSimInterface(*state_interface_, *sim_fields_.sim_command_interface_);
        }
        interface_manager_.registerInterface(sim_fields_.sim_command_interface_.get());
    }
    return &interface_manager_;
}

template <bool SIM>
void TalonFXProDevices<SIM>::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    // ROS_INFO_STREAM("2: read TalonFX");
    tracer.start_unique("talonfxpro");
    for (const auto &d : devices_)
    {
        d->read(time, period);
    }
}

template <bool SIM>
void TalonFXProDevices<SIM>::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("talonfxpro");
    for (const auto &d : devices_)
    {
        d->write(time, period, isEnabled(), prev_robot_enabled_);
    }
    prev_robot_enabled_ = isEnabled();
}

template <bool SIM>
void TalonFXProDevices<SIM>::appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const
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

/// Sim-only functions below

template <bool SIM>
bool TalonFXProDevices<SIM>::setlimit(ros_control_boilerplate::set_limit_switch::Request &req,
                                      ros_control_boilerplate::set_limit_switch::Response & /*res*/)
{
    if constexpr (SIM)
    {
        for (const auto &d : devices_)
        {
            if (((req.target_joint_name.length() == 0) && (d->getCANID() == req.target_joint_id)) ||
                (req.target_joint_name == d->getName()))
            {
                return d->setSimLimitSwitches(req.forward, req.reverse);
            }
        }
    }
    return true;
}

template <bool SIM>
void TalonFXProDevices<SIM>::simInit(ros::NodeHandle &nh)
{
    if constexpr (SIM)
    {
        if (!devices_.empty())
        {
            sim_fields_.sim_limit_switch_srv_ = nh.advertiseService("set_talonfxpro_limit_switch", &TalonFXProDevices::setlimit, this);
        }
    }
}

template <bool SIM>
void TalonFXProDevices<SIM>::simPreRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    if constexpr (SIM)
    {
        // ROS_INFO_STREAM("1: preRead TalonFX: reading CTRE sim outputs");
        tracer.start_unique("talonfxpro FeedEnable");
        if (!devices_.empty())
        {
            ctre::phoenix::unmanaged::FeedEnable(2. * 1000. / read_hz_);
        }
    }
}

// Run after read() and simRead() to write queued updates to the simulated TalonFXPro CTRE libs
template <bool SIM>
void TalonFXProDevices<SIM>::simPostRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    // ROS_INFO_STREAM("4: postRead TalonFX: writing CTRE sim commands");
    if constexpr (SIM)
    {
        tracer.start_unique("talonfxpro simWrite");
        for (const auto &d : devices_)
        {
            d->simWrite(time, period);
        }
        tracer.start_unique("talonfxpro FeedEnable");
        if (!devices_.empty())
        {
            ctre::phoenix::unmanaged::FeedEnable(2. * 1000. / read_hz_);
        }
        tracer.start_unique("talonfxpro battery sim");
        const auto names = state_interface_->getNames();
        std::vector<units::ampere_t> currents;
        for (const auto &name : names)
        {
            currents.push_back(units::ampere_t{state_interface_->getHandle(name)->getSupplyCurrent()});
        }
        units::volt_t battery = frc::sim::BatterySim::Calculate(currents);
        tracer.start_unique("talonfxpro sim");
        for (const auto &d : devices_)
        {
            d->simRead(time, period, getRobotHW()->get<hardware_interface::cancoder::CANCoderSimCommandInterface>(), battery.value()); // should we be running this every loop iteration?
        }
    }
}

template <bool SIM>
 bool TalonFXProDevices<SIM>::gazeboSimInit(const ros::NodeHandle &/*nh*/, boost::shared_ptr<gazebo::physics::Model> parent_model)
{
    if constexpr (SIM)
    {
        for (auto &d : devices_)
        {
            if (!d->gazeboInit(parent_model))
            {
                return false;
            }
        }
    }
    return true;
}