#include "ros/node_handle.h"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ros_control_boilerplate/sim_talonfxpro_device.h"
#include "ctre_interfaces/cancoder_sim_command_interface.h" 
#include "ctre_interfaces/talonfxpro_sim_command_interface.h"
#include "ctre_interfaces/talonfxpro_state_interface.h"
#include "gazebo/physics/Joint.hh"                          // for Joint
#include "gazebo/physics/Model.hh"                          // for Model

SimTalonFXProDevice::SimTalonFXProDevice(const std::string &name_space,
                                         const int joint_index,
                                         const std::string &joint_name,
                                         const int can_id,
                                         const std::string &can_bus,
                                         double read_hz)
    : TalonFXProDevice(name_space, joint_index, joint_name, can_id, can_bus, read_hz)
{
}

SimTalonFXProDevice::~SimTalonFXProDevice() = default;

void SimTalonFXProDevice::registerSimInterface(hardware_interface::talonfxpro::TalonFXProStateInterface &state_interface,
                                               hardware_interface::talonfxpro::TalonFXProSimCommandInterface &sim_command_interface) const
{
    sim_command_interface.registerHandle(hardware_interface::talonfxpro::TalonFXProSimCommandHandle(state_interface.getHandle(getName()), sim_command_.get()));
}

void SimTalonFXProDevice::simRead(const ros::Time &time, const ros::Duration &period, hardware_interface::cancoder::CANCoderSimCommandInterface *sim_cancoder_if, const double battery_voltage)
{
    using hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder;
    using hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANcoder;
    using hardware_interface::talonfxpro::FeedbackSensorSource::SyncCANcoder;
    if (state_->getFeedbackSensorSource() == FusedCANcoder || state_->getFeedbackSensorSource() == RemoteCANcoder || state_->getFeedbackSensorSource() == SyncCANcoder)
    {
        if (!cancoder_id_ || (state_->getFeedbackRemoteSensorID() != *cancoder_id_))
        {
            cancoder_id_ = std::nullopt;
            // We have the cancoder id from Talon states, but handles are looked up by name
            // so we need to find the name of the cancoder with the given id
            const auto names = sim_cancoder_if->getNames();
            for (const auto &name : names)
            {
                auto handle = sim_cancoder_if->getHandle(name);
                if (handle.state()->getDeviceNumber() == state_->getFeedbackRemoteSensorID())
                {
                    cancoder_ = handle;
                    cancoder_id_ = state_->getFeedbackRemoteSensorID();
                    break;
                }
            }
            if (!cancoder_id_)
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, "SimTalonFXDevice " << getName() << " : Could not find cancoder with id " << state_->getFeedbackRemoteSensorID());
            }
        }
    }
    else if (cancoder_id_)
    {
        cancoder_id_ = std::nullopt;
    }
    if (gazebo_joint_)
    {
        const double position = gazebo_joint_->Position(0);
        const double velocity = gazebo_joint_->GetVelocity(0);
        // get motor voltage, us as input to motor model to get torque out
        const double motor_voltage = state_->getMotorVoltage();
        ROS_INFO_STREAM_THROTTLE(2, "Gazebo : " << " Joint Name " << "p =" << position << " v = " << velocity << " voltage = " << motor_voltage);
        //gazebo_joint_->SetPosition(0, counter_);
        //counter_++;
        // Call gazebo_joint_->SetForce() with the torque calc'd from the motor model
    }

    // TODO DO NOT UPDATE SIMULATION STATE IF UNDER A SIMULATOR DEVICE
    // the writes will occasionally conflict
    // Note - since all of these are setting raw rotor positions but setpoints
    // are relative to mechanism positions, need to multiply the values written
    // to the raw positions/velocities by the sensor to mechanism ratio
    // TODO - maybe also rotor to sensor ratio?

    if (gazebo_joint_)
    {
        gazebo_joint_->SetPosition(0, state_->getRotorPosition()); // always set position
    }

    // Set simulation state supply voltages
    auto &sim_state = talonfxpro_->GetSimState();
    sim_state.SetSupplyVoltage(units::voltage::volt_t{battery_voltage});
    if (cancoder_id_) { cancoder_->setSupplyVoltage(battery_voltage); }

}

void SimTalonFXProDevice::simWrite(const ros::Time &time, const ros::Duration &period)
{
    auto &sim_collection = talonfxpro_->GetSimState();
    if (double supply_voltage; sim_command_->supplyVoltageChanged(supply_voltage))
    {
        if (safeCall(sim_collection.SetSupplyVoltage(units::volt_t{supply_voltage}), "talonfxpro sim->SetSupplyVoltage"))
        {
            // ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            sim_command_->resetSupplyVoltage();
            return;
        }
    }
    if (bool forward_limit; sim_command_->forwardLimitChanged(forward_limit))
    {
        if (safeCall(sim_collection.SetForwardLimit(forward_limit), "talonfxpro sim->SetForwardLimit"))
        {
            // ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            sim_command_->resetForwardLimit();
            return;
        }
    }
    if (bool reverse_limit; sim_command_->reverseLimitChanged(reverse_limit))
    {
        if (safeCall(sim_collection.SetReverseLimit(reverse_limit), "talonfxpro sim->SetReverseLimit"))
        {
            // ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            sim_command_->resetReverseLimit();
            return;
        }
    }
    if (double raw_rotor_position; sim_command_->rawPositionChanged(raw_rotor_position))
    {
        if (safeCall(sim_collection.SetRawRotorPosition(units::radian_t{raw_rotor_position}), "talonfxpro sim->SetRawRotorPosition"))
        {
            // ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            sim_command_->resetRawRotorPosition();
            return;
        }
    }
    if (double add_rotor_position; sim_command_->addPositionChanged(add_rotor_position))
    {
        if (safeCall(sim_collection.AddRotorPosition(units::radian_t{add_rotor_position}), "talonfxpro sim->AddRotorPosition"))
        {
            // ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            sim_command_->resetAddRotorPosition();
            return;
        }
    }   
    if (double rotor_velocity; sim_command_->rotorVelocityChanged(rotor_velocity))
    {
        if (safeCall(sim_collection.SetRotorVelocity(units::radians_per_second_t{rotor_velocity}), "talonfxpro sim->SetRotorVelocity"))
        {
            // ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            sim_command_->resetRotorVelocity();
            return;
        }
    }
    if (double rotor_acceleration; sim_command_->accelerationChanged(rotor_acceleration))
    {
        if (safeCall(sim_collection.SetRotorAcceleration(units::radians_per_second_squared_t{rotor_acceleration}), "talonfxpro sim->SetRotorAcceleration"))
        {
            // ROS_INFO_STREAM("CANcoder id = " << getId() << " = " << getName() << " : Set position to " << position);
            // Don't set state - it will be updated in next read() loop
        }
        else
        {
            sim_command_->resetRotorAcceleration();
            return;
        }
    }
}

bool SimTalonFXProDevice::setSimLimitSwitches(const bool forward_limit, const bool reverse_limit)
{
    sim_command_->setForwardLimit(forward_limit);
    sim_command_->setReverseLimit(reverse_limit);
    return true;
}

bool SimTalonFXProDevice::gazeboInit(boost::shared_ptr<gazebo::physics::Model> parent_model)
{
    ROS_INFO_STREAM("Connecting TalonFXPro motor " << getName() << " to Gazebo");
    gazebo_joint_ = parent_model->GetJoint(getName());
    if (!gazebo_joint_)
    {
        ROS_ERROR_STREAM("Joint " << getName() << " not found in Gazebo");
        return false;
    }
    return true;
}
