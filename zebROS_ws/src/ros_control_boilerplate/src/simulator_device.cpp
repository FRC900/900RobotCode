#include "ros/ros.h"

#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"

#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "ros_control_boilerplate/simulator_device.h"
#include "ros_control_boilerplate/read_config_utils.h"
#include "ros_control_boilerplate/tracer.h"

SimulatorDevice::SimulatorDevice(const std::string &name, const XmlRpc::XmlRpcValue &joints, const boost::shared_ptr<simulator_base::Simulator> &simulator, const std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &devices)
{
    simulator_name_ = name;
    ctre_devices_ = devices;
    simulator_ = simulator;

    ROS_INFO_STREAM("SimulatorDevice: Initializing simulator device " << name << ", joints = " << joints);

    for (int i = 0; i < joints.size(); i++) {
        const XmlRpc::XmlRpcValue &joint_params = joints[i];

        std::string joint_name;
        std::string joint_type;
        readStringRequired(joint_params, "name", joint_name);
        readStringRequired(joint_params, "type", joint_type, joint_name);

        // Some C++ wizardry courtesy of Kevin
        auto check_for_correct_pointer_entry = [&]<typename T>()
        {
            const auto &[map_begin, map_end] = devices.equal_range(joint_name);
            for (auto map_entry = map_begin; map_entry != map_end; ++map_entry)
            {
                const auto pointer = dynamic_cast<T *>(map_entry->second);
                if (pointer)
                {
                    return pointer;
                }
            }
            return static_cast<T *>(nullptr);
        };

        if (joint_type == "talonfxpro") {
            auto talon_fx_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CoreTalonFX>(); // more wizardry
            ROS_INFO_STREAM(name << ": Got device ID " << talon_fx_ptr->GetDeviceID() << " for joint " << joint_name);
            names_.push_back(joint_name);
            talonfxs_[joint_name].reset(talon_fx_ptr);
        }
        
    }
}

SimulatorDevice::~SimulatorDevice() {
    simulator_.reset();
}

void SimulatorDevice::simInit(ros::NodeHandle &nh)
{

}

void SimulatorDevice::simStep(const ros::Time& time, const ros::Duration& period, hardware_interface::talonfxpro::TalonFXProSimCommandInterface *sim_talonfxpro_if, Tracer &tracer) {
    // For each TalonFXPro controlled by this simulator:
    // Call update() on the simulator

    // ROS_INFO_STREAM("SimulatorDevice: Updating simulator " << simulator_name_);

    tracer.start(simulator_name_);
    for (std::string joint : names_)
    {
        // virtual void update(const std::string &name, const ros::Time &time, const ros::Duration &period, hardware_interface::talonfxpro::TalonFXProSimCommand *talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state) {
        hardware_interface::talonfxpro::TalonFXProSimCommandHandle handle = sim_talonfxpro_if->getHandle(joint);
        // ROS_INFO_STREAM("SimulatorDevice: Updating simulator " << simulator_name_ << " for joint " << joint);
        simulator_->update(joint, time, period, talonfxs_[joint], handle.state());
    }
}