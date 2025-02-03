#ifndef GENERAL_SIMULATORS_ELEVATOR_SIMULATOR_H_
#define GENERAL_SIMULATORS_ELEVATOR_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include "wpimath/MathShared.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

// #define DEBUG

// Piecewise weight function based on elevator height (multiple stages, gets heavier as you lift because you have to lift the stages and not just the carraige)
// But WPILib makes it impossible (I think) to update elevator carraige mass
// So we'll just ignore this for now.
// Although we could make a new sim and set the state or something like that

/*
const DCMotor& gearbox, double gearing,
units::kilogram_t carriageMass, units::meter_t drumRadius,
units::meter_t minHeight, units::meter_t maxHeight,
bool simulateGravity, units::meter_t startingHeight,
const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});
*/

namespace general_simulators
{
class ElevatorSimulator : public simulator_base::Simulator
{
    public:
        ElevatorSimulator()
        {

        }

        void init(const XmlRpc::XmlRpcValue &simulator_info) override
        {
            // Get parameters from the parameter server in our namespace
            gearing_ = simulator_info["gearing"]; // unitless
            carraige_mass_ = simulator_info["carraige_mass"]; // kg
            drum_radius_ = simulator_info["drum_radius"]; // m
            min_height_ = simulator_info["min_height"]; // m
            max_height_ = simulator_info["max_height"]; // m
            simulate_gravity_ = simulator_info["simulate_gravity"]; // bool
            starting_height_ = simulator_info["starting_height"]; // m

            motor_ = frc::DCMotor::KrakenX60FOC(simulator_info["joints"].size());

            elevator_sim_ = std::make_unique<frc::sim::ElevatorSim>(motor_, gearing_, units::kilogram_t{carraige_mass_}, units::meter_t{drum_radius_}, units::meter_t{min_height_}, units::meter_t{max_height_}, simulate_gravity_, units::meter_t{starting_height_});
        }

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, hardware_interface::talonfxpro::TalonFXProSimCommand *talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state, std::optional<hardware_interface::cancoder::CANCoderSimCommandHandle> cancoder) override
        {
            const double invert = state->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
            
            units::voltage::volt_t motor_voltage{state->getMotorVoltage()};

            // Update the elevator simulation
            elevator_sim_->SetInputVoltage(motor_voltage);
            elevator_sim_->Update(units::second_t{period.toSec()});

            // Get output position and velocity
            auto position = elevator_sim_->GetPosition();
            auto velocity = elevator_sim_->GetVelocity();

            // Write back to motor
            talonfxpro->setRawRotorPosition(invert * position.value() * state->getSensorToMechanismRatio());
            talonfxpro->setRotorVelocity(invert * velocity.value() * state->getSensorToMechanismRatio());

            this->update_cancoder(talonfxpro, state, cancoder);
        }

        ~ElevatorSimulator() override
        {

        }

    private:
        frc::DCMotor motor_ = frc::DCMotor::KrakenX60FOC(1);
        std::unique_ptr<frc::sim::ElevatorSim> elevator_sim_;

        double gearing_ = 1.0;
        double carraige_mass_ = 0.001;
        double drum_radius_ = 0.05;
        double min_height_ = 0.0;
        double max_height_ = 1.0;
        bool simulate_gravity_ = true;
        double starting_height_ = 0.0;
};

};

#endif
