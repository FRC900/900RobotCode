#ifndef GENERAL_SIMULATORS_SWERVE_SIMULATOR_H_
#define GENERAL_SIMULATORS_SWERVE_SIMULATOR_H_
#include "simulator_interface/simulator_base.h"
#include <frc/system/plant/DCMotor.h>
#include "wpimath/MathShared.h"
#include "frc/EigenCore.h"

// https://introcontrol.mit.edu/fall23/prelabs/prelab10/ss_1
// thanks 22377 also :)

class DCMotorModel
{
    // TODO make this unit aware
    public:
        DCMotorModel(double R, double L, double Kt, double J, double gearing, double Vsupply) {
            this->R = R; // resistance
            this->L = L; // inductance
            this->Kt = Kt; // torque constant
            this->J = J; // inertia
            this->gearing = gearing; // gear ratio
            this->Vsupply = Vsupply;
        }

        // Actually simulate the motor
        // State vector = x = [angular velocity, current]
        // Input/control vector = u = [applied voltage, external torque]

        // Derivative of state vector = ẋ = [angular acceleration, there isn't really a name for dCurrent/dt]
        Eigen::Vector2d update(Eigen::Vector2d x, Eigen::Vector2d u) {
            // WPILib model doesn't account for external torque but this does

            // ẋ = Ax + Bu
            // ẋ = A[ω, I]ᵀ + B[V, τ_ext]ᵀ
            /*
            from dynamics code in swerve MPC
            xdot = [((-b/J)*x[0] + (Kt/J)*x[1]/gear_ratio + u[1]/J),
                   (-Kt/L)*x[0]*gear_ratio + (-R/L)*x[1] + (V/L)*u[0]]
            */
            // from MIT 6.310: J * ̇ω = τ_motor + τ_ext
            // ̇ω = (τ_motor + τ_ext)/J
            // τ_motor = Kt * I and I don't feel like dealing with gear ratios
            // so, ̇ω = (Kt * I)/J + τ_ext/J which matches the equation above and the matrices below, yay!

            // and I don't really know what to do with the derivative of current so will just ignore it for now and hope it works
            Eigen::Matrix2d A;
            A <<             0.0, (Kt/J)/gearing,
                 (-Kt/L)*gearing, -R/L;
            
            Eigen::Matrix2d B;
            B <<       0.0, 1.0/J,
                 Vsupply/L, 0.0;
            
            return A*x + B*u;
        }

    private:
        double R, L, Kt, J, gearing, Vsupply;
};

// namespace general_simulators
// {
// template <size_t WHEELCOUNT>
// class SwerveSimulator : public simulator_base::Simulator
// {
//     public:
//         SwerveSimulator()
//         {

//         }

//         void init(const XmlRpc::XmlRpcValue &simulator_info) override
//         {
//             // Get parameters from the parameter server in our namespace
//             double gearing = simulator_info["gearing"]; // unitless
//             double B = simulator_info["pacejka_constants"]["B"];
//             double C = simulator_info["pacejka_constants"]["C"];
//             double D = simulator_info["pacejka_constants"]["D"];
//             double E = simulator_info["pacejka_constants"]["E"];

//             // Create a DCMotor object for the arm.
//             // If we're not using Krakens here, we're doing something wrong :)
//             frc::DCMotor motor = frc::DCMotor::KrakenX60FOC(1);

//             // Create a FlywheelSim object
//             single_jointed_arm_sim_ = std::make_unique<frc::sim::SingleJointedArmSim>(motor, gearing, units::kilogram_square_meter_t{moment_of_inertia}, units::meter_t{arm_length}, units::radian_t{min_angle}, units::radian_t{max_angle}, simulate_gravity, units::radian_t{starting_angle});
//             // ROS_INFO_STREAM("Created flywheel sim");
//         }

//         void update(const std::string &name, const ros::Time &time, const ros::Duration &period, std::unique_ptr<ctre::phoenix6::hardware::core::CoreTalonFX> &talonfxpro, std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWState> &state) override
//         {
//             if (!set_initial_position_)
//             {
//                 ROS_INFO_STREAM(name << ": position unset, setting to " << state->getPosition() << " rad");
//                 single_jointed_arm_sim_->SetState(units::radian_t{state->getPosition()}, units::radians_per_second_t{0.0});
//                 set_initial_position_ = true;
//             }

//             // ROS_INFO_STREAM("About to get motor voltage");
//             // Get the motor voltage from the state
//             units::voltage::volt_t motor_voltage = talonfxpro->GetSimState().GetMotorVoltage();

//             // ROS_INFO_STREAM("WPILib updates, object is " << single_jointed_arm_sim_ << " , motor voltage is " << motor_voltage.value() << "V");
//             // Update the arm simulation
//             single_jointed_arm_sim_->SetInputVoltage(motor_voltage);
//             single_jointed_arm_sim_->Update(units::second_t{period.toSec()});

//             // ROS_INFO_STREAM("WPILib outputs");
//             // Get output angular velocity
//             auto angular_velocity = single_jointed_arm_sim_->GetVelocity() * state->getSensorToMechanismRatio();

//             // Get angle
//             auto angle = single_jointed_arm_sim_->GetAngle();

//             // ROS_INFO_STREAM("Write back to state");
//             // Set the velocity of the simulated motor
//             talonfxpro->GetSimState().SetRotorVelocity(angular_velocity);

//             // Add position delta
//             talonfxpro->GetSimState().AddRotorPosition(angular_velocity * units::second_t{period.toSec()});

//             // ROS_INFO_STREAM("FLYWHEEL SIM IS BEING SIMMED YAYYYYYY");
//         }

//         ~SingleJointedArmSimulator() override
//         {

//         }

//     private:
//         std::unique_ptr<frc::sim::SingleJointedArmSim> single_jointed_arm_sim_;
//         std::vector<std::string> drive_joints_;
//         std::vector<std::string> turn_joints_;
//         bool set_initial_position_ = false;

//         /*
//         # Tire models
//         def pacejka(constants, slip): # constants: [B,C,D,E] (same names as formula mentioned here: https://en.wikipedia.org/wiki/Hans_B._Pacejka)
//             b = constants[0]
//             c = constants[1]
//             d = constants[2]
//             e = constants[3]
//             return d*cs.sin(c*cs.atan(b*slip-e*(b*slip-cs.atan(b*slip))))

//         def kiencke(constants, slip):
//             a = constants[0]
//             b = constants[1]
//             return (2*a*b*slip)/(b*b + slip*slip)
//         */

//         // Longitudinal force
//         units::newton_t pacejka_model(double B, double C, double D, double E, double slip) {
//             return units::newton_t{D * sin(C * atan(B * slip - E * (B * slip - atan(B * slip))))};
//         }

// };

// };

#endif
