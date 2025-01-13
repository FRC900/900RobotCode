#ifndef GENERAL_SIMULATORS_FLYWHEEL_SIMULATOR_H_
#define GENERAL_SIMULATORS_FLYWHEEL_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
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
            ddr_.registerVariable<double>("gearing", [&](){return gearing_;}, [&](double val){
                gearing_ = val;
                frc::LinearSystem<1, 1, 1> m_plant{frc::LinearSystemId::FlywheelSystem(motor_, units::kilogram_square_meter_t{moment_of_inertia_}, gearing_)};
                flywheel_sim_ = std::make_unique<frc::sim::FlywheelSim>(m_plant, motor_);
            }, "unitless", 0, 10.0);
            ddr_.registerVariable<double>("moment_of_inertia", [&](){return moment_of_inertia_;}, [&](double val){
                moment_of_inertia_ = val;
                frc::LinearSystem<1, 1, 1> m_plant{frc::LinearSystemId::FlywheelSystem(motor_, units::kilogram_square_meter_t{moment_of_inertia_}, gearing_)};
                flywheel_sim_ = std::make_unique<frc::sim::FlywheelSim>(m_plant, motor_);
            }, "kg m^2", 0, 1.0);
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
            frc::LinearSystem<1, 1, 1> m_plant{frc::LinearSystemId::FlywheelSystem(motor_, units::kilogram_square_meter_t{moment_of_inertia_}, gearing_)};
            flywheel_sim_ = std::make_unique<frc::sim::FlywheelSim>(m_plant, motor_);
            // ROS_INFO_STREAM("Created flywheel sim");
        }

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, hardware_interface::talonfxpro::TalonFXProSimCommand *talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state) override
        {
            // The flywheel simulator requires the roboRIO battery voltage
            // /home/ubuntu/900RobotCode/zebROS_ws/devel/lib/ros_control_boilerplate/frcrobot_sim_main: symbol lookup error: /home/ubuntu/900RobotCode/zebROS_ws/devel/lib//libgeneral_simulators.so: undefined symbol: _ZN3frc15RobotController17GetBatteryVoltageEv
            // So we're going to remove that from WPILib for now so it runs, but probably eventually move this into ros_control_boilerplate so we can link to WPILib sim stuff

            // Get the motor voltage from the state
            // Note: we have voltage in the normal state from the read done in sim_talonfxpro_device, which is called in preRead by talonfxpro_devices when SIM=true
            units::voltage::volt_t motor_voltage{state->getMotorVoltage()};
            ROS_INFO_STREAM("Motor voltage = " << motor_voltage.value());

            // ROS_INFO_STREAM("WPILib updates, object is " << flywheel_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
            // Update the flywheel simulation
            flywheel_sim_->SetInputVoltage(motor_voltage);
            flywheel_sim_->Update(units::second_t{period.toSec()});

            ROS_INFO_STREAM("WPILib outputs");
            // Get output angular velocity
            auto angular_velocity = flywheel_sim_->GetAngularVelocity().value();

            ROS_INFO_STREAM("Write back to state");
            // Set the flywheel velocity of the simulated motor
            talonfxpro->setRotorVelocity(angular_velocity * state->getRotorToSensorRatio());

            // Add position delta
            talonfxpro->setAddRotorPosition(angular_velocity * state->getRotorToSensorRatio() * period.toSec());

            ROS_INFO_STREAM("FLYWHEEL SIM IS BEING SIMMED YAYYYYYY");
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
