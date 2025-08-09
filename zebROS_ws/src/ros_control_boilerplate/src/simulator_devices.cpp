#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager

#include "ros_control_boilerplate/simulator_device.h"
#include "ros_control_boilerplate/simulator_devices.h"
#include "ros_control_boilerplate/read_config_utils.h"
#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "ctre_interfaces/talonfxpro_sim_command_interface.h"
#include "ctre_interfaces/cancoder_sim_command_interface.h"

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

            for (int j = 0; j < joints_array.size(); j++)
            {
                std::string controlled_joint_name;
                readStringRequired(joints_array[j], "name", controlled_joint_name);
                controlled_joints_.push_back(controlled_joint_name);
            }
        }
    }

    // Add default simulator which has control of all non-controlled TalonFX joints
    std::map<std::string, XmlRpc::XmlRpcValue> all_talonfxpros;
    for (int i = 0; i < joint_param_list.size(); i++)
	{
		const XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
        std::string joint_name;
        std::string joint_type;
        readStringRequired(joint_params, "name", joint_name);
        readStringRequired(joint_params, "type", joint_type, joint_name);
        
        // If we've found a TalonFX (talonfxpro) not controlled by any simulators, add it to the all_joints list
        // This is a bit of a hack, but it's the easiest way to get all the joints that aren't controlled by a simulator (thanks Copilot lol)
        
        if (joint_type == "talonfxpro")
        {
            ROS_INFO_STREAM("Checking if " << joint_name << " is controlled by a simulator");
            if (std::find(controlled_joints_.begin(), controlled_joints_.end(), joint_name) == controlled_joints_.end())
            {
                ROS_INFO_STREAM(joint_name << " is not controlled by a simulator, adding to default simulator");
                all_talonfxpros[joint_name] = joint_params;
            }
        }
    }

    if (!all_talonfxpros.empty())
    {
        XmlRpc::XmlRpcValue xmlrpc_talonfxpros;
        xmlrpc_talonfxpros.setSize(all_talonfxpros.size());
        int i = 0;
        for (const auto &pair : all_talonfxpros)
        {
            xmlrpc_talonfxpros[i] = pair.second;
            i++;
        }
        simulators_["default"] = loader_->createInstance("general_simulators/DefaultSimulator");
        simulator_types_["default"] = "general_simulators/DefaultSimulator";
        XmlRpc::XmlRpcValue default_simulator_info;
        default_simulator_info["type"] = "general_simulators/DefaultSimulator";
        simulators_["default"]->init(default_simulator_info);
        devices_.emplace_back(std::make_unique<SimulatorDevice>("default", xmlrpc_talonfxpros, simulators_["default"], devices));
    }
}

SimulatorDevices::~SimulatorDevices() {
    devices_.clear();
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

// preRead for this does not work and breaks flywheel
void SimulatorDevices::simPostRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    // ROS_INFO_STREAM("3: postRead Simulator: stepping simulators");
    tracer.start("simulator devices");
    for (const auto &d : devices_)
    {
        d->simStep(time, period, getRobotHW()->get<hardware_interface::talonfxpro::TalonFXProSimCommandInterface>(), getRobotHW()->get<hardware_interface::cancoder::CANCoderSimCommandInterface>(), tracer);
    }
    tracer.stop("simulator devices");
}
