#ifndef GENERAL_SIMULATORS_FLYWHEEL_SIMULATOR_H_
#define GENERAL_SIMULATORS_FLYWHEEL_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include "wpimath/MathShared.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

// #define DEBUG

/*
The flywheel simulator works fine, these are just comments about the state of this as a whole (especially motion magic)

CURRENT ORDER:
- preRead TalonFX: handle reading CTRE simulation outputs (updating motor voltage + torque current, feeding rotor pos/vel into CANcoder)
- read TalonFX: normal things
- postRead simulator:
    - runs simulation step
    - writes motor states directly to TalonFXs

If we want to move writing motor states to TalonFXs to the TalonFX postRead function through command interface, then
the simulation step must be run before that but after CTRE simulation outputs are read (since we need motor voltage etc).

And this is weird I think, because that would require us to put simulation in preRead. But it needs to go after the CTRE sim reads in preRead.
So keep it in postRead! But it needs to go before the motor states are written in postRead.

Hmm.

Putting it in preRead didn't work, it did the weird thing where swerve angle motor gets a really high velocity (guessing one loop iter behind?)

OH WAIT MOTION MAGIC NEEDS CLOSED LOOP REFERENCE SLOPE FOR SIMULATION

so if it goes in preRead before those values are read in normal **read**, it'll have stuff from the previous iteration

this also fails when it's in postRead if the talon read thread hasn't started yet which ig makes sense if calling read() is a dependency?

so we can do a little hacky thing and move simulator devices to before talonfx devices

Now order is:
- preRead TalonFX: handle reading CTRE simulation outputs (updating motor voltage + torque current, feeding rotor pos/vel into CANcoder)
- read TalonFX: normal things
- postRead simulator: runs simulation step and sends sim commands
- postRead TalonFX: writes sim commands directly to TalonFXs

big bug for velocity was setAddRotorPosition didn't set it again if the same value was passed in, RIP constant velocity :(

now, it kinda works using the command interface, but one of the swerve angle motors invariably takes an insanely fast velocity at startup

(and only one)

given that reading is happening in a separate thread and that this new order creates more of a gap between (3) and (4) and that the swerve angle motor affected is random every time...

could this be a threading issue?

Actual order is:
- preRead TalonFX: handle reading CTRE simulation outputs (updating motor voltage + torque current, feeding rotor pos/vel into CANcoder)
?? read thread read maybe ?? <-- ideally we read here. we've just gotten the latest sim states including closed loop reference/referenceslope
- read TalonFX: copy normal values (including closed loop reference/referenceslope) from most recent read thread state
?? read thread read maybe ?? <-- now our values are outdated and won't be picked up for a while
- postRead simulator: runs simulation step and sends sim commands
?? read thread read maybe ?? <-- very bad, we don't have the latest motor position or velocity (i think, but also might be just trying to justify this since it would explain weird behavior, this gap doesn't exist when using TalonFX pointer)
- postRead TalonFX: writes sim commands directly to TalonFXs
?? read thread read maybe ?? <-- ok fine
- write TalonFX: send control commands to motors
?? read thread read maybe ?? <-- not ideal, don't have latest inputs

I'm not sure if threading is the issue, but it definitely seems like a possible cause given how random the issue is. Something seems to be maybe racing.
*/

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
            const double invert = state->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
            
            units::voltage::volt_t motor_voltage{state->getMotorVoltage()};
            // ROS_INFO_STREAM("Motor voltage = " << motor_voltage.value());

            // ROS_INFO_STREAM("WPILib updates, object is " << flywheel_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
            // Update the flywheel simulation
            flywheel_sim_->SetInputVoltage(motor_voltage);
            flywheel_sim_->Update(units::second_t{period.toSec()});

            // ROS_INFO_STREAM("WPILib outputs");
            // Get output angular velocity
            auto angular_velocity = flywheel_sim_->GetAngularVelocity();

            // ROS_INFO_STREAM("Write back to state");
            // Set the flywheel velocity of the simulated motor
            talonfxpro->setRotorVelocity(invert * angular_velocity.value() * state->getSensorToMechanismRatio());

            // Add position delta
            talonfxpro->setAddRotorPosition(invert * angular_velocity.value() * state->getSensorToMechanismRatio() * period.toSec());

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
