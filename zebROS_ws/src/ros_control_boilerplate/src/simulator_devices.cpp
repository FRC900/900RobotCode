#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager

#include "ros_control_boilerplate/simulator_device.h"
#include "ros_control_boilerplate/simulator_devices.h"
#include "ros_control_boilerplate/read_config_utils.h"
#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "ctre_interfaces/talonfxpro_state_types.h"


SimulatorDevices::SimulatorDevices(ros::NodeHandle &root_nh, const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices)
    : state_interface_{std::make_unique<hardware_interface::talonfxpro::TalonFXProStateInterface>()}
{
	ros::NodeHandle nh(root_nh, "hardware_interface");
    XmlRpc::XmlRpcValue joint_param_list;
    if (!nh.getParam("joints", joint_param_list))
    {
        throw std::runtime_error("No joints were specified.");
    }

    loader_ = std::make_unique<pluginlib::ClassLoader<simulator_base::Simulator>>("simulator_interface", "simulator_base::Simulator");

	for (int i = 0; i < joint_param_list.size(); i++)
	{
		const XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
        std::string joint_name;
        std::string joint_type;
        readStringRequired(joint_params, "name", joint_name);
        readStringRequired(joint_params, "type", joint_type, joint_name);
        if (joint_type == "simulator")
        {
            XmlRpc::XmlRpcValue simulator_info;
            if (!root_nh.getParam(joint_name, simulator_info))
            {
                ROS_ERROR_STREAM("A simulator '" << joint_name << "' was specified, but no details were found.");
            }

            XmlRpc::XmlRpcValue joints_array;
            readArrayRequired(joint_params, "joints", joints_array, joint_name);

            try {
                simulators_[joint_name] = loader_->createInstance(simulator_info["type"]);
                std::string sim_type = simulator_info["type"];
                simulator_types_[joint_name] = sim_type;
            } catch (...) {
                ROS_ERROR_STREAM("Failed to load simulator " << joint_name);
                simulators_[joint_name] = nullptr;
            }

            // Initialize the simulator
            try {
                simulators_[joint_name]->init(simulator_info);
            } catch (...) {
                ROS_ERROR_STREAM("Failed to initialize simulator " << joint_name);
            }

            devices_.emplace_back(std::make_unique<SimulatorDevice>(joint_name, joints_array, simulators_[joint_name], devices));
        }
    }
}

SimulatorDevices::~SimulatorDevices() {
    for (const auto &d : devices_)
    {
        d->~SimulatorDevice();
    }
    for (auto& pair : simulators_) {
        pair.second.reset();
        loader_->unloadLibraryForClass(simulator_types_[pair.first]);
    }
}

void SimulatorDevices::simInit(ros::NodeHandle &nh)
{
    for (const auto &d : devices_)
    {
        d->simInit(nh);
    }
}

hardware_interface::InterfaceManager *SimulatorDevices::registerInterface()
{
    for (const auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    return &interface_manager_;
}

void SimulatorDevices::simPostRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start("simulator devices");
    for (const auto &d : devices_)
    {
        d->simPostRead(time, period, tracer);
    }
    tracer.stop("simulator devices");
}
