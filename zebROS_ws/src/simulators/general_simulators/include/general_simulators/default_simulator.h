#ifndef GENERAL_SIMULATORS_DEFAULT_SIMULATOR_H_
#define GENERAL_SIMULATORS_DEFAULT_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"

namespace general_simulators
{
class DefaultSimulator : public simulator_base::Simulator
{
    public:
        DefaultSimulator()
        {

        }

        void init(const XmlRpc::XmlRpcValue &simulator_info) override
        {
            // ROS_INFO_STREAM("DefaultSimulator init");
            // do nothing lol
        }

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, hardware_interface::talonfxpro::TalonFXProSimCommand *talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state) override
        {
            // This simulator is the default (non-physics) simulator, so it instantly goes to the setpoint
            switch (state->getControlMode())
            {
            case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = DutyCycleOut");
                // Special-case 0V for duty cycle mode
                if (state->getControlOutput() == 0.0)
                {
                    talonfxpro->setRotorVelocity(0.0);
                }
                break;
            case hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = TorqueCurrentFOC");
                break;
            case hardware_interface::talonfxpro::TalonMode::VoltageOut:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = VoltageOut");
                break;
            case hardware_interface::talonfxpro::TalonMode::PositionDutyCycle:
            case hardware_interface::talonfxpro::TalonMode::PositionVoltage:
            case hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC:
            {
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = PositionDutyCycle/Voltage/TorqueCurrentFOC");
                // Position control mode, set position and velocity to setpoints
                talonfxpro->setRawRotorPosition(invert * state->getControlPosition() * state->getSensorToMechanismRatio());

                // We'll also have a velocity setpoint, so set that here
                talonfxpro->setRotorVelocity(invert * state->getControlVelocity() * state->getSensorToMechanismRatio());
                break;
            }
            case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
            case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
            case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
            {
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = VelocityDutyCycle/Voltage/TorqueCurrentFOC");
                // Calculate velocity setpoint and position delta, applying invert and sensor : mechanism ratio
                units::angular_velocity::radians_per_second_t velocity_setpoint{invert * state->getControlVelocity() * state->getSensorToMechanismRatio()};
                units::radian_t delta_position{velocity_setpoint * units::second_t{period.toSec()}};

                // ROS_INFO_STREAM("Velocity setpoint: " << velocity_setpoint.value() << " delta position: " << delta_position.value());

                // Velocity control mode, add position delta and set velocity
                talonfxpro->setRotorVelocity(velocity_setpoint.value());
                talonfxpro->setAddRotorPosition(delta_position.value()); // VERY IMPORTANT SO CTRE SIM KNOWS MOTORS MOVE

                break;
            }
            case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
            case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage: 
            case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage:
            case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoDutyCycle:
            {
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = MotionMagicDutyCycle/Voltage/ExpoVoltage/ExpoDutyCycle");
                units::radian_t position{invert * state->getClosedLoopReference() * state->getSensorToMechanismRatio()};
                const units::angular_velocity::radians_per_second_t velocity{invert * state->getClosedLoopReferenceSlope() * state->getSensorToMechanismRatio()};
                talonfxpro->setRawRotorPosition(position.value());
                // talonfxpro->GetSimState().AddRotorPosition(invert * velocity * units::second_t{period.toSec()} * state->getRotorToSensorRatio());
                talonfxpro->setRotorVelocity(velocity.value());
                break;
            }
            case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
            case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoTorqueCurrentFOC:
            case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle:
            case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage:
            case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC:
            case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle:
            case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage:
            case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC:
            {
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = kitchen sink");
                // TODO : debug, check sim Orientation field

                // Motion magic, controls both position and velocity
                units::radian_t target_position{invert * state->getClosedLoopReference() * state->getSensorToMechanismRatio()};
                units::angular_velocity::radians_per_second_t target_velocity{invert * state->getClosedLoopReferenceSlope() * state->getSensorToMechanismRatio()};

                // Set rotor position and velocity
                talonfxpro->setAddRotorPosition(target_position.value() - state->getRotorPosition());
                talonfxpro->setRotorVelocity(target_velocity.value());

                break;
            }
            case hardware_interface::talonfxpro::TalonMode::Follower:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = Follower");
                break;
            case hardware_interface::talonfxpro::TalonMode::StrictFollower:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = StrictFollower");
                break;
            case hardware_interface::talonfxpro::TalonMode::NeutralOut:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = NeutralOut");
                break;
            case hardware_interface::talonfxpro::TalonMode::CoastOut:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = CoastOut");
                break;
            case hardware_interface::talonfxpro::TalonMode::StaticBrake:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = StaticBrake");
                break;
            default:
                // ROS_ERROR_STREAM("DefaultSimulator update, name = " << name << ", mode = " << static_cast<int>(state->getControlMode()));
                break;
            // TODO : support differential modes, somehow
            }
        }

        ~DefaultSimulator() override
        {

        }
    private:
        double invert = 1.0;
};

};

#endif
