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

        if (joint_type == "talonfx") {
            const auto talon_fx_ptr = check_for_correct_pointer_entry.operator()<ctre::phoenix6::hardware::core::CoreTalonFX>(); // more wizardry
            ROS_INFO_STREAM(name << ": Got device ID " << talon_fx_ptr->GetDeviceID() << " for joint " << joint_name);
            states_[joint_name] = std::make_unique<hardware_interface::talonfxpro::TalonFXProHWState>(talon_fx_ptr->GetDeviceID());
            talonfxs_[joint_name].reset(talon_fx_ptr);
        }
        
    }
}

SimulatorDevice::~SimulatorDevice() = default;

// Note: registerInterface is called before simInit

void SimulatorDevice::simInit(ros::NodeHandle &nh)
{
    // Copilot's suggestion:
    // Load the simulator from XMLRPC values retrieved by _devices.cpp
    // Example YAML:
    /*
    top_left_shooter_simulator:
        joints: [top_left_shooter_joint]
        type: general_simulators/FlywheelSimulator
        gear_ratio: 1.0 # already accounted for by sensor to mechanism ratio
        inertia: 0.1 # kg m^2
    */
    // ROS_INFO_STREAM("SimulatorDevice: Initializing simulator " << simulator_name_);
    // XmlRpc::XmlRpcValue simulator_info;
    // if (!nh.getParam(simulator_name_, simulator_info))
    // {
    //     ROS_ERROR_STREAM("A simulator '" << simulator_name_ << "' was specified, but no details were found.");
    // }

    // Initialize the simulator
    // try {
    //     simulator_->init(simulator_info);
    // } catch (...) {
    //     ROS_ERROR_STREAM("Failed to initialize simulator " << simulator_name_);
    // }

    // ROS_INFO_STREAM("SimulatorDevice: Initialized simulator " << simulator_name_);
    // ROS_INFO_STREAM("SimulatorDevice: Loading joints for simulator " << simulator_name_);
    // XmlRpc::XmlRpcValue joints;
    // readArrayRequired(simulator_info, "joints", joints, simulator_name_);
    // for (int i = 0; i < joints.size(); i++)
    // {
    //     std::string joint_name;
    //     readStringRequired(joints[i], "name", joint_name);
    //     joints_.push_back(joint_name);
    //     ROS_INFO_STREAM("SimulatorDevice: Loaded joint " << joint_name << " for simulator " << simulator_name_);
    // }
    // ROS_INFO_STREAM("SimulatorDevice: Loaded joints for simulator " << simulator_name_);

    // Set up CANcoders
    // Basically:
    // 1. Iterate through the states_ map and
    // 2. Check feedback sensor source
    // 3. If the feedback sensor source is one of the three CANcoder modes,
    //    then get the CAN ID of the associated CANcoder
    // 4. Create a new CoreCANcoder object with that CAN ID
    // 5. Store the CoreCANcoder object in the cancoders_ map
    ROS_INFO_STREAM("SimulatorDevice: Setting up CANcoders for simulator " << simulator_name_);
    for (const auto &[joint, state] : states_)
    {
        const auto feedback_sensor_source = state->getFeedbackSensorSource();
        if (feedback_sensor_source == hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder ||
            feedback_sensor_source == hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANcoder ||
            feedback_sensor_source == hardware_interface::talonfxpro::FeedbackSensorSource::SyncCANcoder)
        {
            const auto can_id = state->getFeedbackRemoteSensorID();
            cancoders_[joint] = std::make_unique<ctre::phoenix6::hardware::core::CoreCANcoder>(can_id);
            // Store CANcoder invert value
            ctre::phoenix6::configs::MagnetSensorConfigs magnet_configs;
            cancoders_[joint]->GetConfigurator().Refresh(magnet_configs);
            cancoder_inverts_[joint] = magnet_configs.SensorDirection == ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive ? -1.0 : 1.0;
        }
    }
}

void SimulatorDevice::registerInterfaces(hardware_interface::talonfxpro::TalonFXProStateInterface &state_interface)
{
    // Need CAN IDs to create std::make_unique<hardware_interface::talonfxpro::TalonFXProHWState>(can_id) (i.e. state_)
    ROS_INFO_STREAM("FRCRobotInterface: Registering interfaces for joints of: " << simulator_name_);
    for (const std::string &joint : joints_) {
        // This links states_[joint] to the actual state interface, if I'm understanding what's happening in talonfxpro_device.cpp correctly
        ROS_INFO_STREAM(simulator_name_ << ": registering handle for state of " << joint);
        hardware_interface::talonfxpro::TalonFXProStateHandle state_handle(joint, states_[joint].get());
        state_interface.registerHandle(state_handle);
    }
    return;
}

void SimulatorDevice::simPostRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer) {
    // For each TalonFXPro controlled by this simulator:
    // Call update() on the simulator
    // If the TalonFXPro is using a CANcoder, update the CANcoder's simulated state

    tracer.start(simulator_name_);
    for (auto &[joint, state] : states_)
    {
        // Call update() on the simulator
        // ROS_INFO_STREAM("SimulatorDevice: Updating simulator for joint " << joint);
        simulator_->update(joint, time, period, talonfxs_[joint], state);

        // If the TalonFXPro is using a CANcoder, update the CANcoder's state
        if (cancoders_.count(joint))
        {
            // ROS_INFO_STREAM("SimulatorDevice: Updating CANcoder for joint " << joint);
            auto &cancoder = cancoders_[joint];
            const units::radians_per_second_t cancoder_velocity{state->getRotorVelocity() / state->getRotorToSensorRatio() * cancoder_inverts_[joint]};
            if (cancoder) {
                cancoder->GetSimState().SetVelocity(cancoder_velocity);
                cancoder->GetSimState().AddPosition(cancoder_velocity * units::second_t{period.toSec()});
            }
        }
    }
}