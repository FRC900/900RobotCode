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

void SimTalonFXProDevice::simRead(const ros::Time &time, const ros::Duration &period, hardware_interface::cancoder::CANCoderSimCommandInterface *sim_cancoder_if, const units::voltage::volt_t battery_voltage)
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

    auto &sim_state = talonfxpro_->GetSimState();
    // Note - since all of these are setting raw rotor positions but setpoints
    // are relative to mechanism positions, need to multiply the values written
    // to the raw positions/velocities by the sensor to mechanism ratio
    // TODO - maybe also rotor to sensor ratio?

    if (gazebo_joint_)
    {
        gazebo_joint_->SetPosition(0, state_->getRotorPosition()); // always set position
    }

    double cancoder_invert = 1.0;
    double cancoder_offset = 0.0;
    if (cancoder_id_)
    {
        cancoder_invert = cancoder_.state()->getSensorDirection() == hardware_interface::cancoder::SensorDirection::Clockwise_Positive ? -1.0 : 1.0;
        cancoder_offset = cancoder_.state()->getMagnetOffset();
    }

    const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;

    // Set simulation state supply voltages
    sim_state.SetSupplyVoltage(battery_voltage);
    if (cancoder_id_) { cancoder_->setSupplyVoltage(battery_voltage.value()); }

    // Update our motor state from simulation state
    state_->setMotorVoltage(sim_state.GetMotorVoltage().value());
    state_->setDutyCycle(sim_state.GetMotorVoltage() / battery_voltage);
    state_->setSupplyCurrent(sim_state.GetSupplyCurrent().value());
    state_->setTorqueCurrent(sim_state.GetTorqueCurrent().value());

    // Update CANcoder, if one exists
    // This is fine to do here because it's called in preRead, I think
    // So the control flow looks like preRead (update pos/vel here), real read to state, postRead to update sim, preRead before next loop iter
    if (cancoder_id_) {
        double cancoder_velocity{state_->getRotorVelocity() * cancoder_invert}; // *really* feels like you should need to divide by rotor to sensor ratio, but this works
        cancoder_->setVelocity(cancoder_velocity);
        cancoder_->setAddPosition(cancoder_velocity * period.toSec());
    }
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

bool SimTalonFXProDevice::setSimCurrent(const double /*stator_current*/, const double /*supply_current*/)
{
    ROS_ERROR_STREAM("Error settings sim current on TalonFXPro device " << getName() << " : Not supported");
    return false;
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
