#include "ros/node_handle.h"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ros_control_boilerplate/sim_talonfxpro_device.h"
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

void SimTalonFXProDevice::simRead(const ros::Time &/*time*/, const ros::Duration &period)
{
    if (gazebo_joint_)
    {
        const double position = gazebo_joint_->Position(0);
        const double velocity = gazebo_joint_->GetVelocity(0);
        // get motor voltage, us as input to motor model to get torque out
        const double motor_voltage = state_->getMotorVoltage();
        ROS_INFO_STREAM_THROTTLE(2, "Gazebo : " << " Joint Name " << "p =" << position << " v = " << velocity << " voltage = " << motor_voltage);
        //gazebo_joint_->SetPosition(0, counter);
        //counter++;
        // Call gazebo_joint_->SetForce() with the torque calc'd from the motor model
    }

    auto &sim_state = talonfxpro_->GetSimState();
    // Note - since all of these are setting raw rotor positions but setpoints
    // are relative to mechanism positions, need to multiply the values written
    // to the raw positions/velocities by the sensor to mechanism ratio
    // TODO - maybe also rotor to sensor ratio?

    if (gazebo_joint_)
    {
        gazebo_joint_->SetPosition(0, state_->getRotorPosition()); // always set position
    }

    switch (state_->getControlMode())
    {
    case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
        // NEED TO FILL IN FOR SWERVE
        
        
        //sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
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
        units::radian_t setpoint{invert * state_->getControlOutput() * state_->getSensorToMechanismRatio()}; // isn't this velocity?
        sim_state.SetRawRotorPosition(setpoint);
        sim_state.SetRotorVelocity(units::angular_velocity::turns_per_second_t{0});
        sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
        if (gazebo_joint_)
        {
            gazebo_joint_->SetPosition(0, state_->getRotorPosition());
        }
        ROS_ERROR_STREAM("IN POSITION MODE");
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
    case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
    {
         /*
          * Using the 971-style first order system model. V = I * R + Kv * w
          * torque = Kt * I
          * torque = 0.0192 * (V - (Kv * w)) / R
          */
        const double invert = state_->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
        units::angular_velocity::radians_per_second_t setpoint{invert * state_->getControlVelocity() * state_->getSensorToMechanismRatio()};
        sim_state.SetRotorVelocity(setpoint);
        units::radian_t delta_position{invert * state_->getControlVelocity() * period.toSec() * state_->getSensorToMechanismRatio()};
        sim_state.AddRotorPosition(delta_position); // VERY IMPORTANT SO CTRE SIM KNOWS MOTORS MOVE
        sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
        
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

            double torque_current = (((invert * sim_state.GetMotorVoltage().value()) - (kV * state_->getControlVelocity() * gear_ratio))) / R;

            //         idk  vvvv
            double torque = -1.0 * kT * torque_current;

            // (Nm / A) * (V - (rad/s/V * rad/s)) / (ohms) = Nm

            // gazebo_joint_->SetPosition(0, state_->getRotorPosition());
            gazebo_joint_->SetForce(0, torque);
            // gazebo_joint_->SetVelocity(0, state_->getControlVelocity());
            ROS_ERROR_STREAM_THROTTLE_NAMED(1, std::to_string(state_->getCANID()), "IN VELOCITY MODE " << torque << " " << sim_state.GetMotorVoltage().value() << " " << state_->getControlVelocity() << " " << state_->getSensorToMechanismRatio());
        }
        // -3, -169, 1
        // -3, 169, 1 
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
    case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage: 
    {
        units::radian_t position{state_->getClosedLoopReference() * state_->getSensorToMechanismRatio()};
        units::angular_velocity::radians_per_second_t velocity{state_->getClosedLoopReferenceSlope() * state_->getSensorToMechanismRatio()};
        sim_state.SetRawRotorPosition(position);
        sim_state.SetRotorVelocity(velocity);
        sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
        if (gazebo_joint_)
        {
            gazebo_joint_->SetPosition(0, state_->getRotorPosition());
            ROS_WARN_STREAM_THROTTLE(1, "Motion Magic torque current " << sim_state.GetTorqueCurrent().value() << " " << state_->getRotorPosition());
        }
        break;
    }
    case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
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
        units::radian_t target_position{invert * state_->getClosedLoopReference() * state_->getSensorToMechanismRatio()};
        units::angular_velocity::radians_per_second_t target_velocity{invert * state_->getClosedLoopReferenceSlope() * state_->getSensorToMechanismRatio()};
        sim_state.SetRawRotorPosition(target_position);
        sim_state.SetRotorVelocity(target_velocity);
        sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
        ROS_ERROR_STREAM("IN MOTION MAGIC MODE");
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

bool SimTalonFXProDevice::setSimLimitSwitches(const bool forward_limit, const bool reverse_limit)
{
    auto &sim_state = talonfxpro_->GetSimState();
    sim_state.SetForwardLimit(forward_limit);
    sim_state.SetReverseLimit(reverse_limit);
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
