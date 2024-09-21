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
            double starting_angle = simulator_info["starting_angle"]; // radians

            // Set up node handle
            ros::NodeHandle nh_;

            // Add dynamic reconfigure for these parameters
            ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(nh_);

            // ddr_->registerVariable<double>(
            //     "gearing",
            //     [this]()
            //     { return gearing; },
            //     [this](double b)
            //     { gearing = b; },
            //     "Gearing",
            //     0.0, 100.0);
            
            // ddr_->registerVariable<double>(
            //     "moment_of_inertia",
            //     [this]()
            //     { return moment_of_inertia; },
            //     [this](double b)
            //     { moment_of_inertia = b; },
            //     "Moment of Inertia",
            //     0.0, 10.0);
            
            // ddr_->registerVariable<double>(
            //     "arm_length",
            //     [this]()
            //     { return arm_length; },
            //     [this](double b)
            //     { arm_length = b; },
            //     "Arm Length",
            //     0.0, 10.0);
            
            // ddr_->registerVariable<double>(
            //     "min_angle",
            //     [this]()
            //     { return min_angle_; },
            //     [this](double b)
            //     { min_angle_ = b; },
            //     "Min Angle",
            //     0, 2*M_PI);
            
            // ddr_->registerVariable<double>(
            //     "max_angle",
            //     [this]()
            //     { return max_angle_; },
            //     [this](double b)
            //     { max_angle_ = b; },
            //     "Max Angle",
            //     0, 2*M_PI);

            // ddr_->registerVariable<bool>(
            //     "simulate_gravity",
            //     [this]()
            //     { return simulate_gravity; },
            //     [this](bool b)
            //     { simulate_gravity = b; },
            //     "Simulate Gravity",
            //     false, true);
            
            // ddr_->registerVariable<double>(
            //     "starting_angle",
            //     [this]()
            //     { return starting_angle; },
            //     [this](double b)
            //     { starting_angle = b; },
            //     "Starting Angle",
            //     0, 2*M_PI);
            
            // ddr_->publishServicesTopics();

            // Create a DCMotor object for the arm.
            // If we're not using Krakens here, we're doing something wrong :)
            frc::DCMotor motor = frc::DCMotor::KrakenX60FOC(simulator_info["joints"].size());

            // Create a FlywheelSim object
            single_jointed_arm_sim_ = std::make_unique<frc::sim::SingleJointedArmSim>(motor, gearing, units::kilogram_square_meter_t{moment_of_inertia}, units::meter_t{arm_length}, units::radian_t{min_angle_}, units::radian_t{max_angle_}, simulate_gravity, units::radian_t{starting_angle});
            // ROS_INFO_STREAM("Created flywheel sim");
        }

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, std::unique_ptr<ctre::phoenix6::hardware::core::CoreTalonFX> &talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state) override
        {
            if (!set_initial_position_)
            {
                double position = state->getPosition();
                ROS_INFO_STREAM(name << ": position unset, setting to " << position << " rad");
                single_jointed_arm_sim_->SetState(units::radian_t{position}, units::radians_per_second_t{0.0});
                set_initial_position_ = true;
            }

            // ROS_INFO_STREAM("About to get motor voltage");
            // Get the motor voltage from the state
            units::voltage::volt_t motor_voltage = talonfxpro->GetSimState().GetMotorVoltage();

            // ROS_INFO_STREAM("WPILib updates, object is " << single_jointed_arm_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
            // Update the arm simulation
            single_jointed_arm_sim_->SetInputVoltage(motor_voltage);
            single_jointed_arm_sim_->Update(units::second_t{period.toSec()});

            // ROS_INFO_STREAM("WPILib outputs");
            // Get output angular velocity
            auto angular_velocity = single_jointed_arm_sim_->GetVelocity() * state->getSensorToMechanismRatio();
            auto angular_rotor_velocity = angular_velocity * state->getRotorToSensorRatio();

            // Get angle
            auto angle = single_jointed_arm_sim_->GetAngle();

            // ROS_INFO_STREAM("Write back to state");
            // Set the velocity of the simulated motor
            talonfxpro->GetSimState().SetRotorVelocity(angular_rotor_velocity);

            // Add position delta
            talonfxpro->GetSimState().AddRotorPosition(angular_rotor_velocity * units::second_t{period.toSec()});

            // ROS_INFO_STREAM("FLYWHEEL SIM IS BEING SIMMED YAYYYYYY");
        }

        ~SingleJointedArmSimulator() override
        {

        }

    private:
        std::unique_ptr<frc::sim::SingleJointedArmSim> single_jointed_arm_sim_;
        bool set_initial_position_ = false;
        double min_angle_, max_angle_;
		std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
};

};

#endif
