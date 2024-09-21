#ifndef GENERAL_SIMULATORS_FLYWHEEL_SIMULATOR_H_
#define GENERAL_SIMULATORS_FLYWHEEL_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include "wpimath/MathShared.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

extern "C" {

}

// #define DEBUG

namespace general_simulators
{
class FlywheelSimulator : public simulator_base::Simulator
{
    public:
        FlywheelSimulator()
        {
            ddr_.registerVariable<double>("gearing", [&](){return gearing_;}, [&](double val){gearing_ = val; flywheel_sim_ = std::make_unique<frc::sim::FlywheelSim>(motor_, gearing_, units::kilogram_square_meter_t{moment_of_inertia_});}, "unitless", 0, 10.0);
            ddr_.registerVariable<double>("moment_of_inertia", [&](){return moment_of_inertia_;}, [&](double val){moment_of_inertia_ = val; flywheel_sim_ = std::make_unique<frc::sim::FlywheelSim>(motor_, gearing_, units::kilogram_square_meter_t{moment_of_inertia_});}, "kg m^2", 0, 1.0);
            ddr_.publishServicesTopics();
        }

        void init(const XmlRpc::XmlRpcValue &simulator_info) override
        {
            // Get parameters from the parameter server in our namespace
            // Need: gearing and moment of inertia in kg m^2
            gearing_ = simulator_info["gearing"];
            moment_of_inertia_ = simulator_info["moment_of_inertia"];

            // Create a DCMotor object for the flywheel.
            // If we're not using Krakens here, we're doing something wrong :)
            motor_ = frc::DCMotor::KrakenX60FOC(simulator_info["joints"].size());

            // Create a FlywheelSim object
            flywheel_sim_ = std::make_unique<frc::sim::FlywheelSim>(motor_, gearing_, units::kilogram_square_meter_t{moment_of_inertia_});
            // ROS_INFO_STREAM("Created flywheel sim");
        }

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, std::unique_ptr<ctre::phoenix6::hardware::core::CoreTalonFX> &talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state) override
        {
            // ROS_INFO_STREAM("About to get motor voltage");
            // Get the motor voltage from the state
            units::voltage::volt_t motor_voltage = talonfxpro->GetSimState().GetMotorVoltage();

            // ROS_INFO_STREAM("WPILib updates, object is " << flywheel_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
            // Update the flywheel simulation
            flywheel_sim_->SetInputVoltage(motor_voltage);
            flywheel_sim_->Update(units::second_t{period.toSec()});

            // ROS_INFO_STREAM("WPILib outputs");
            // Get output angular velocity
            auto angular_velocity = flywheel_sim_->GetAngularVelocity();

            // ROS_INFO_STREAM("Write back to state");
            // Set the flywheel velocity of the simulated motor
            talonfxpro->GetSimState().SetRotorVelocity(angular_velocity * state->getRotorToSensorRatio());

            // Add position delta
            talonfxpro->GetSimState().AddRotorPosition(angular_velocity * state->getRotorToSensorRatio() * units::second_t{period.toSec()});

            // ROS_INFO_STREAM("FLYWHEEL SIM IS BEING SIMMED YAYYYYYY");
        }

        ~FlywheelSimulator() override
        {

        }

    private:
        frc::DCMotor motor_ = frc::DCMotor::KrakenX60FOC(1);
        std::unique_ptr<frc::sim::FlywheelSim> flywheel_sim_;
        
        ddynamic_reconfigure::DDynamicReconfigure ddr_;

        double gearing_ = 1.0;
        double moment_of_inertia_ = 0.001;
};

};

#endif
