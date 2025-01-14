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

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, std::unique_ptr<ctre::phoenix6::hardware::core::CoreTalonFX> &talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state) override
        {
            // This should run before simRead
            const double invert = state->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
            // This simulator is the default (non-physics) simulator, so it instantly goes to the setpoint
            switch (state->getControlMode())
            {
            case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
                // ROS_INFO_STREAM("DefaultSimulator update, name = " << name << ", mode = DutyCycleOut");
                // Special-case 0V for duty cycle mode
                if (state->getControlOutput() == 0.0)
                {
                    talonfxpro->GetSimState().SetRotorVelocity(units::angular_velocity::radians_per_second_t{0.0});
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
                units::radian_t pos{invert * state->getControlPosition() * state->getSensorToMechanismRatio()};
                units::angular_velocity::radians_per_second_t vel{invert * state->getControlVelocity() * state->getSensorToMechanismRatio()};
                talonfxpro->GetSimState().SetRawRotorPosition(pos);

                // We'll also have a velocity setpoint, so set that here
                talonfxpro->GetSimState().SetRotorVelocity(vel);
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
                talonfxpro->GetSimState().SetRotorVelocity(velocity_setpoint);
                talonfxpro->GetSimState().AddRotorPosition(delta_position); // VERY IMPORTANT SO CTRE SIM KNOWS MOTORS MOVE

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
                talonfxpro->GetSimState().SetRawRotorPosition(position);
                // talonfxpro->GetSimState().AddRotorPosition(delta_position);
                talonfxpro->GetSimState().SetRotorVelocity(velocity);
                /*
                units::radian_t position{state_->getClosedLoopReference() * state_->getSensorToMechanismRatio()};
                units::radian_t cancoder_position{(state_->getClosedLoopReference() - cancoder_offset - M_PI / 2) * cancoder_invert};
                units::angular_velocity::radians_per_second_t velocity{state_->getClosedLoopReferenceSlope() * state_->getSensorToMechanismRatio()};
                sim_state.SetRawRotorPosition(position);
                if (cancoder_) { cancoder_->GetSimState().SetRawPosition(cancoder_position); }
                sim_state.SetRotorVelocity(velocity);
                sim_state.SetSupplyVoltage(units::voltage::volt_t{12.5});
                */
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
                talonfxpro->GetSimState().AddRotorPosition(target_position);
                talonfxpro->GetSimState().SetRotorVelocity(target_velocity);

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
};

};

#endif
