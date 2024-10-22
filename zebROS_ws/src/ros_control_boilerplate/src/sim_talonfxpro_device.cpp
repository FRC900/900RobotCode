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

void SimTalonFXProDevice::simRead(const ros::Time &/*time*/, const ros::Duration &period, hardware_interface::cancoder::CANCoderSimCommandInterface *sim_cancoder_if)
{
    if (state_->getFeedbackSensorSource() == hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder ||
        state_->getFeedbackSensorSource() == hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANcoder ||
        state_->getFeedbackSensorSource() == hardware_interface::talonfxpro::FeedbackSensorSource::SyncCANcoder)
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

    switch (state_->getControlMode())
    {
    case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
        // NEED TO FILL IN FOR SWERVE
        // Special case for 0 output, set rotor velocity to 0
        if (state_->getControlOutput() == 0.0)
        {
            sim_command_->setRotorVelocity(0.0);
        }
        sim_command_->setSupplyVoltage(12.5);
        
        //gazebo_joint_->SetForce()
        break;
    case hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC:
        break;
    case hardware_interface::talonfxpro::TalonMode::VoltageOut:
        break;
    case hardware_interface::talonfxpro::TalonMode::PositionDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::PositionVoltage:
    case hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC:
    {
        const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
        const double position{invert * state_->getControlPosition() * state_->getSensorToMechanismRatio()};
        const double velocity{invert * state_->getControlVelocity() * state_->getSensorToMechanismRatio()}; // should be 0 in all cases?
        sim_command_->setRawRotorPosition(position);
        sim_command_->setRotorVelocity(velocity);
        sim_command_->setRotorAcceleration(0.0);
        if (cancoder_id_)
        {
            const double cancoder_position{state_->getControlPosition() * cancoder_invert - cancoder_offset};
            // ROS_INFO_STREAM("TalonFXPro simRead PositionVoltage : cancoder id = " << *cancoder_id_ << " control position = "  << state_->getControlPosition() << " cancoder_position = " << cancoder_position);
            cancoder_->setRawPosition(cancoder_position);
            cancoder_->setAddPosition(0);
            cancoder_->setVelocity(0);
        }
        sim_command_->setSupplyVoltage(12.5);
        if (gazebo_joint_)
        {
            gazebo_joint_->SetPosition(0, state_->getRotorPosition());
        }
        // ROS_ERROR_STREAM("IN POSITION MODE");
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
    case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
    {
        // TODO : model accel by keeping track of prev + current velocity values?
        const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
        const double setpoint{invert * state_->getControlVelocity() * state_->getSensorToMechanismRatio()};
        sim_command_->setRotorVelocity(setpoint);
        const double delta_position{invert * state_->getControlVelocity() * period.toSec() * state_->getSensorToMechanismRatio()};
        sim_command_->setAddRotorPosition(delta_position); // VERY IMPORTANT SO CTRE SIM KNOWS MOTORS MOVE
        sim_command_->setSupplyVoltage(12.5);
        
// TODO - need to have a cleaner way of getting sim_state Get() values into here
#if 0
         /*
          * Using the 971-style first order system model. V = I * R + Kv * w
          * torque = Kt * I
          * torque = 0.0192 * (V - (Kv * w)) / R
          */
        if (gazebo_joint_)
        {
            constexpr double kT = 0.0192; // Nm/A
            double kV = 509.2;            // convert from rpm/V to rad/s/V
            kV *= 2.0 * M_PI / 60.0;
            kV = 1.0 / kV;              // convert from (rad/s)/V to V/(rad/s)
            constexpr double R = 0.039; // ohms, resistance of motor
            constexpr double gear_ratio = 1.0 / 6.75;
            /*
            T=kt * (V - kv*ω) / R
            */

            double torque_current = ((invert * sim_state.GetMotorVoltage().value()) - (kV * state_->getControlVelocity() * gear_ratio)) / R;

            //         idk  vvvv
            double torque = -1.0 * kT * torque_current;

            // (Nm / A) * (V - (rad/s/V * rad/s)) / (ohms) = Nm

            // gazebo_joint_->SetPosition(0, state_->getRotorPosition());
            gazebo_joint_->SetForce(0, torque);
            // gazebo_joint_->SetVelocity(0, state_->getControlVelocity());
            ROS_ERROR_STREAM_THROTTLE_NAMED(1, std::to_string(state_->getCANID()), "IN VELOCITY MODE " << torque << " " << sim_state.GetMotorVoltage().value() << " " << state_->getControlVelocity() << " " << state_->getSensorToMechanismRatio());
        }
#endif
        // -3, -169, 1
        // -3, 169, 1 
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage: 
    {
        // TODO : is invert needed for velocity as well?
        const double position{invert * state_->getClosedLoopReference() * state_->getSensorToMechanismRatio()};
        const double velocity{state_->getClosedLoopReferenceSlope() * state_->getSensorToMechanismRatio()};
        sim_command_->setRawRotorPosition(position);
        sim_command_->setRotorVelocity(velocity);
        sim_command_->setSupplyVoltage(12.5);
        if (cancoder_id_)
        {
            // ROS_WARN_STREAM("cancoder id = " << *cancoder_id_ << " cancoder_value = " << cancoder_position.value());
            // TODO : debug, cancoder offset doesn't seem to matter here?
            // Does cancoder invert matter?  If not, there's no need to read anything from the cancoder here, get rid of cancoder handle
            const double cancoder_position{state_->getClosedLoopReference() * cancoder_invert};
            const auto cancoder_velocity { state_->getRotorVelocity() / state_->getRotorToSensorRatio() * cancoder_invert};
            cancoder_->setRawPosition(cancoder_position);
            cancoder_->setVelocity(cancoder_velocity);
        }
        if (gazebo_joint_)
        {
            gazebo_joint_->SetPosition(0, state_->getRotorPosition());
            // ROS_WARN_STREAM_THROTTLE(1, "Motion Magic torque current " << sim_state.GetTorqueCurrent().value() << " " << state_->getRotorPosition());
        }
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoTorqueCurrentFOC:
    // TODO : test the modes below when/if we actually use them
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC:
    case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage:
    case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC:
    {
        // TODO : debug, check sim Orientation field
        const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
        double target_position{invert * state_->getClosedLoopReference() * state_->getSensorToMechanismRatio()};
        double cancoder_target_position{cancoder_invert * state_->getClosedLoopReference() - cancoder_offset};
        double target_velocity{invert * state_->getClosedLoopReferenceSlope() * state_->getSensorToMechanismRatio()};
        sim_command_->setRawRotorPosition(target_position);
        if (cancoder_id_) { cancoder_->setRawPosition(cancoder_target_position); }
        sim_command_->setRotorVelocity(target_velocity);
        sim_command_->setSupplyVoltage(12.5);
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::Follower:
        break;
    case hardware_interface::talonfxpro::TalonMode::StrictFollower:
        break;
    case hardware_interface::talonfxpro::TalonMode::NeutralOut:
        break;
    case hardware_interface::talonfxpro::TalonMode::CoastOut:
        break;
    case hardware_interface::talonfxpro::TalonMode::StaticBrake:
        break;
    // TODO : support differential modes, somehow
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
