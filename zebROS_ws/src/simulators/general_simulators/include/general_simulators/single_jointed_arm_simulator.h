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

/*
[INFO] [1737045687.322484755]: writing 0.900000 radians to shooter_pivot_controller
[INFO] [1737045687.326438509]: read vel = 0
[INFO] [1737045687.326510341]: read thread state vel = 0
[INFO] [1737045687.350798212]: copy from read vel = 0
[INFO] [1737045687.350879277]: arm in, 0.355884 desired pos, 0.357 set pos, 0.355884 actual pos, 0 V, 0 desired vel, 0 set velocity, 0 actual vel
[INFO] [1737045687.351087756]: shooter pivot update, pos = 0.355884, vel = 0, rotor vel = 0

[INFO] [1737045687.426471043]: read vel = 0
[INFO] [1737045687.426540878]: read thread state vel = 0
[INFO] [1737045687.450791903]: copy from read vel = 0
[INFO] [1737045687.450863827]: arm in, 0.361157 desired pos, 0.357 set pos, 0.355884 actual pos, 0.23 V, 0.0981748 desired vel, 0 set velocity, 0 actual vel
[INFO] [1737045687.451022273]: shooter pivot update, pos = 0.355884, vel = 0, rotor vel = 0

[INFO] [1737045687.526433293]: read vel = 0
[INFO] [1737045687.526504344]: read thread state vel = 0
[INFO] [1737045687.550770597]: copy from read vel = 0
[INFO] [1737045687.550879273]: arm in, 0.408135 desired pos, 0.439699 set pos, 0.355884 actual pos, 1.01 V, 0.687223 desired vel, 1.07344 set velocity, 0 actual vel
**writing the simulated rotor velocity to TalonFX SimState** [INFO] [1737045687.550925417]: sim write rotor velocity 1.07344
[INFO] [1737045687.551081034]: shooter pivot update, pos = 0.355884, vel = 0, rotor vel = 0

**the TalonFX rotor velocity is read correctly** [INFO] [1737045687.626444295]: read vel = 1.06765
[INFO] [1737045687.626516882]: read thread state vel = 1.06765
[INFO] [1737045687.650790552]: copy from read vel = 1.06765
[INFO] [1737045687.650865334]: arm in, 0.504967 desired pos, 0.662433 set pos, 0.355884 actual pos, 2.03 V, 1.1781 desired vel, 2.57615 set velocity, 1.06765 actual vel
[INFO] [1737045687.650905406]: sim write rotor velocity 2.57615
**but the TalonFX regular velocity is not updated yet** [INFO] [1737045687.651086242]: shooter pivot update, pos = 0.355884, vel = 0, rotor vel = 1.06765

[INFO] [1737045687.726470199]: read vel = 2.57709
[INFO] [1737045687.726533542]: read thread state vel = 2.57709
[INFO] [1737045687.750790426]: copy from read vel = 2.57709
[INFO] [1737045687.750855092]: arm in, 0.649737 desired pos, 0.961119 set pos, 0.355884 actual pos, 2.51 V, 1.5708 desired vel, 3.34752 set velocity, 2.57709 actual vel
[INFO] [1737045687.750888169]: sim write rotor velocity 3.34752
[INFO] [1737045687.751021478]: shooter pivot update, pos = 0.355884, vel = 0, rotor vel = 2.57709

[INFO] [1737045687.826472412]: read vel = 3.33794
[INFO] [1737045687.826536487]: read thread state vel = 3.33794
[INFO] [1737045687.850789609]: copy from read vel = 3.33794
[INFO] [1737045687.850865274]: arm in, 0.781084 desired pos, 1.2297 set pos, 0.546097 actual pos, 2.16 V, 1.07992 desired vel, 2.94426 set velocity, 3.33794 actual vel
[INFO] [1737045687.850908791]: sim write rotor velocity 2.94426
**it only updates two timesteps later, over here** [INFO] [1737045687.851088458]: shooter pivot update, pos = 0.546097, vel = 1.06765, rotor vel = 3.33794

[INFO] [1737045687.926433276]: read vel = 2.94524
[INFO] [1737045687.926498462]: read thread state vel = 2.94524
[INFO] [1737045687.950790579]: copy from read vel = 2.94524
[INFO] [1737045687.950867922]: arm in, 0.866412 desired pos, 1.2479 set pos, 0.920388 actual pos, 0.1 V, 0.687223 desired vel, 0.0134826 set velocity, 2.94524 actual vel
[INFO] [1737045687.950916177]: sim write rotor velocity 0.0134826
[INFO] [1737045687.951100055]: shooter pivot update, pos = 0.920388, vel = 2.57709, rotor vel = 2.94524
*/

// rotor velocity is out of sync with normal velocity by two timesteps!

/*
https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/simulation/simulation-intro.html
As a part of high-fidelity simulation, the influence of the CAN bus is simulated at a level similar to what happens on a real robot. This means that the timing behavior of control and status signals in simulation will align to the same framing intervals seen on a real CAN bus. In simulation, this may appear as a delay between setting a signal and getting its real value, or between setting its real value and getting it in API.

In unit tests, it may be useful to increase the update rate of status signals to avoid erroneous failures and minimize delays. The update rate can be modified for simulation by wrapping the signal update frequency in a Utils.isSimulation() (Java, C++) condition.

I was running at 10 Hz for these examples though, so the default update rate should be more than fast enough? idk

https://www.chiefdelphi.com/t/ctre-start-of-2024-open-alpha-for-phoenix-6/441193/92?u=benbean18
*/

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
            // single_jointed_arm_sim_->SetState(units::radian_t{state->getRotorPosition() / state->getSensorToMechanismRatio()}, units::radians_per_second_t{state->getRotorVelocity() / state->getSensorToMechanismRatio()});

            units::voltage::volt_t motor_voltage{state->getMotorVoltage()};

            // ROS_INFO_STREAM("WPILib updates, object is " << single_jointed_arm_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
            single_jointed_arm_sim_->SetInputVoltage(motor_voltage);
            single_jointed_arm_sim_->Update(units::second_t{period.toSec()});

            // ROS_INFO_STREAM("WPILib outputs");
            auto angular_velocity = single_jointed_arm_sim_->GetVelocity() * state->getSensorToMechanismRatio();

            auto angle = single_jointed_arm_sim_->GetAngle();

            // ROS_INFO_STREAM("Write back to state");
            ROS_INFO_STREAM("arm in, " << state->getClosedLoopReference() << " desired pos, "
                            << angle.value() << " set pos, "
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
