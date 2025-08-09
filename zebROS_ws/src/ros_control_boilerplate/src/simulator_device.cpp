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
        auto check_for_correct_pointer_entry = [&devices, &joint_name]<typename T>()
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
            const auto talon_fx_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CoreTalonFX>(); // more wizardry
            ROS_INFO_STREAM(name << ": Got device ID " << talon_fx_ptr->GetDeviceID() << " for joint " << joint_name);
            names_.push_back(joint_name);
        }
        
    }
}

SimulatorDevice::~SimulatorDevice() {
    simulator_.reset();
}

void SimulatorDevice::simInit(ros::NodeHandle &/*nh*/)
{

}

void SimulatorDevice::simStep(const ros::Time& time, const ros::Duration& period, hardware_interface::talonfxpro::TalonFXProSimCommandInterface *sim_talonfxpro_if, hardware_interface::cancoder::CANCoderSimCommandInterface *sim_cancoder_if, Tracer &tracer) {
    // For each TalonFXPro controlled by this simulator:
    // Call update() on the simulator

    // ROS_INFO_STREAM("SimulatorDevice: Updating simulator " << simulator_name_);

    tracer.start(simulator_name_);
    for (std::string joint : names_)
    {
        hardware_interface::talonfxpro::TalonFXProSimCommandHandle tfxpro_sim_handle = sim_talonfxpro_if->getHandle(joint);
        // ROS_INFO_STREAM("SimulatorDevice: Updating simulator " << simulator_name_ << " for joint " << joint);

        using hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder;
        using hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANcoder;
        using hardware_interface::talonfxpro::FeedbackSensorSource::SyncCANcoder;
        if ((tfxpro_sim_handle.state()->getFeedbackSensorSource() == FusedCANcoder || tfxpro_sim_handle.state()->getFeedbackSensorSource() == RemoteCANcoder || tfxpro_sim_handle.state()->getFeedbackSensorSource() == SyncCANcoder) &&
            (!cancoder_ids_.contains(joint) || (tfxpro_sim_handle.state()->getFeedbackRemoteSensorID() != cancoder_ids_[joint]))) 
        {
            bool found_cancoder = false;
            // We have the cancoder id from Talon states, but handles are looked up by name
            // so we need to find the name of the cancoder with the given id
            const auto names = sim_cancoder_if->getNames();
            for (const auto &name : names)
            {
                auto handle = sim_cancoder_if->getHandle(name);
                if (handle.state()->getDeviceNumber() == tfxpro_sim_handle.state()->getFeedbackRemoteSensorID())
                {
                    cancoder_handles_[joint] = handle;
                    cancoder_ids_[joint] = tfxpro_sim_handle.state()->getFeedbackRemoteSensorID();
                    found_cancoder = true;
                    break;
                }
            }
            if (!found_cancoder)
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, "SimulatorDevice for " << joint << " : Could not find cancoder with id " << tfxpro_sim_handle.state()->getFeedbackRemoteSensorID());
            }
        }

        // Note: if CANcoder ID changes or is detached during simulation, cancoder_handles_[joint] will not be cleared and we'll still be updating the old CANcoder
        // This shouldn't be an issue in simulation, but just something to remember
        simulator_->update(joint, time, period, tfxpro_sim_handle.operator->(), tfxpro_sim_handle.state(), cancoder_handles_.contains(joint) ? std::make_optional<hardware_interface::cancoder::CANCoderSimCommandHandle>(cancoder_handles_[joint]) : std::nullopt);
    }
}