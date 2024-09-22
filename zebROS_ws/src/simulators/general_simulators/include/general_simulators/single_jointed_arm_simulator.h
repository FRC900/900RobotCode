#ifndef GENERAL_SIMULATORS_SINGLE_JOINTED_ARM_SIMULATOR_H_
#define GENERAL_SIMULATORS_SINGLE_JOINTED_ARM_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include "wpimath/MathShared.h"

// Use 6328's code as an example
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/superstructure/arm/ArmIOSim.java

namespace general_simulators
{
  class SingleJointedArmSimulator : public simulator_base::Simulator
  {
    public:
      SingleJointedArmSimulator() {

      }

      void init(const XmlRpc::XmlRpcValue &simulator_info) override {
        // Get parameters from the parameter server in our namespace
        double gearing = simulator_info["gearing"]; // unitless
        double moment_of_inertia = simulator_info["moment_of_inertia"]; // kg m^2
        double arm_length = simulator_info["arm_length"]; // meters
        double min_angle = simulator_info["min_angle"]; // radians
        double max_angle = simulator_info["max_angle"]; // radians
        bool simulate_gravity = simulator_info["simulate_gravity"]; // bool
        double starting_angle = simulator_info["starting_angle"]; // radians

        // Create a DCMotor object for the arm.
        // If we're not using Krakens here, we're doing something wrong :)
        frc::DCMotor motor = frc::DCMotor::KrakenX60(simulator_info["joints"].size());

        // Create a FlywheelSim object
        single_jointed_arm_sim_ = std::make_unique<frc::sim::SingleJointedArmSim>(motor, gearing, units::kilogram_square_meter_t{moment_of_inertia}, units::meter_t{arm_length}, units::radian_t{min_angle}, units::radian_t{max_angle}, simulate_gravity, units::radian_t{starting_angle});
        // ROS_INFO_STREAM("Created flywheel sim");
      }

      void update(const std::string &name, const ros::Time &time, const ros::Duration &period, std::unique_ptr<ctre::phoenix6::hardware::core::CoreTalonFX> &talonfxpro, std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWState> &state) override {
        if (!set_initial_position_) {
          ROS_INFO_STREAM(name << ": position unset, setting to " << state->getPosition() << " rad");
          single_jointed_arm_sim_->SetState(units::radian_t{state->getPosition()}, units::radians_per_second_t{0.0});
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

        // Get angle
        auto angle = single_jointed_arm_sim_->GetAngle();

        // ROS_INFO_STREAM("Write back to state");
        // Set the velocity of the simulated motor
        talonfxpro->GetSimState().SetRotorVelocity(angular_velocity);

        // Add position delta
        talonfxpro->GetSimState().AddRotorPosition(angular_velocity * units::second_t{period.toSec()});

        // ROS_INFO_STREAM("FLYWHEEL SIM IS BEING SIMMED YAYYYYYY");
      }

      ~SingleJointedArmSimulator() override {

      }
    
    private:
      std::unique_ptr<frc::sim::SingleJointedArmSim> single_jointed_arm_sim_;
      bool set_initial_position_ = false;
  };

};

#endif
