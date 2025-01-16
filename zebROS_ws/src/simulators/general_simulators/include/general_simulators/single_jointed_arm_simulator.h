#ifndef GENERAL_SIMULATORS_SINGLE_JOINTED_ARM_SIMULATOR_H_
#define GENERAL_SIMULATORS_SINGLE_JOINTED_ARM_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include "wpimath/MathShared.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
#include "ros/ros.h"

// Use 6328's code as an example
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/superstructure/arm/ArmIOSim.java

// Something that would be really interesting to do is to do system identification from a bag file
// Subscribe to talonfxpro_states, simulate using bagged voltages, see what the output is, and then compare the two
// Optimize for the best fit for the moment of inertia, all of the other things are pretty well known
// This optimization could be done using a genetic algorithm or something similar (interesting suggestion, Copilot)
// Maybe gradient descent could work also? I'm not sure how to even find the gradient with noncontinuous data though, should think about it more

namespace general_simulators
{
class SingleJointedArmSimulator : public simulator_base::Simulator
{
    public:
        SingleJointedArmSimulator()
        {

        }

        void init(const XmlRpc::XmlRpcValue &simulator_info) override
        {
            // Get parameters from the parameter server in our namespace
            double gearing = simulator_info["gearing"]; // unitless
            double moment_of_inertia = simulator_info["moment_of_inertia"]; // kg m^2
            double arm_length = simulator_info["arm_length"]; // meters
            min_angle_ = simulator_info["min_angle"]; // radians
            max_angle_ = simulator_info["max_angle"]; // radians
            bool simulate_gravity = simulator_info["simulate_gravity"]; // bool
            starting_angle_ = simulator_info["starting_angle"]; // radians

            ros::NodeHandle nh_;

            frc::DCMotor motor = frc::DCMotor::KrakenX60FOC(simulator_info["joints"].size());

            single_jointed_arm_sim_ = std::make_unique<frc::sim::SingleJointedArmSim>(motor, gearing, units::kilogram_square_meter_t{moment_of_inertia}, units::meter_t{arm_length}, units::radian_t{min_angle_}, units::radian_t{max_angle_}, simulate_gravity, units::radian_t{starting_angle_});
        }

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, hardware_interface::talonfxpro::TalonFXProSimCommand *talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state) override
        {
            single_jointed_arm_sim_->SetState(units::radian_t{state->getRotorPosition() / state->getSensorToMechanismRatio() / state->getRotorToSensorRatio()}, units::radians_per_second_t{state->getRotorVelocity() / state->getSensorToMechanismRatio() / state->getRotorToSensorRatio()});

            units::voltage::volt_t motor_voltage{state->getMotorVoltage()};

            // ROS_INFO_STREAM("WPILib updates, object is " << single_jointed_arm_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
            single_jointed_arm_sim_->SetInputVoltage(motor_voltage);
            single_jointed_arm_sim_->Update(units::second_t{period.toSec()});

            // ROS_INFO_STREAM("WPILib outputs");
            auto angular_velocity = single_jointed_arm_sim_->GetVelocity() * state->getSensorToMechanismRatio() * state->getRotorToSensorRatio();

            auto angle = single_jointed_arm_sim_->GetAngle();

            // ROS_INFO_STREAM("Write back to state");
            ROS_INFO_STREAM("arm in, " << state->getClosedLoopReference() << " desired pos, "
                            << state->getPosition() << " actual pos, " 
                            << state->getMotorVoltage() << " V, "  
                            << state->getClosedLoopReferenceSlope() << " desired vel, " 
                            << angular_velocity.value() << " set velocity, "
                            << state->getRotorVelocity() << " actual vel");
            talonfxpro->setRawRotorPosition(angle.value());
            talonfxpro->setAddRotorPosition(angular_velocity.value() * period.toSec());
            talonfxpro->setRotorVelocity(angular_velocity.value());
        }

        ~SingleJointedArmSimulator() override
        {

        }

    private:
        std::unique_ptr<frc::sim::SingleJointedArmSim> single_jointed_arm_sim_;
        bool set_initial_position_ = false;
        double min_angle_, max_angle_, starting_angle_;
};

};

#endif
