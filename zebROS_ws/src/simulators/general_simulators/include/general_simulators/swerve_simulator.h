#ifndef GENERAL_SIMULATORS_SWERVE_SIMULATOR_H_
#define GENERAL_SIMULATORS_SWERVE_SIMULATOR_H_
#include "units/current.h"
#include "units/moment_of_inertia.h"
#include "simulator_interface/simulator_base.h"
#include <frc/system/plant/DCMotor.h>
#include "frc/EigenCore.h"
#include <units/inductance.h>
#include <units/velocity.h>
#include <angles/angles.h>
#include "geometry_msgs/TwistStamped.h"
#include "general_simulators/SwerveDebug.h"
#include "std_srvs/Empty.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

// https://introcontrol.mit.edu/fall23/prelabs/prelab10/ss_1
// thanks 22377 also :)

// Something that would be really interesting to do is to do system identification from a bag file
// Subscribe to talonfxpro_states, simulate using bagged voltages, see what the output is, and then compare the two
// Optimize for the best fit for the tire model parameters, all of the other things are pretty well known

using newton_meters_per_ampere_t =
      units::unit_t<units::compound_unit<units::newton_meters,
                                         units::inverse<units::ampere>>>;

class DCMotorModel
{
    // TODO make this unit aware
    public:
        DCMotorModel() = default;

        DCMotorModel(units::ohm_t R, units::inductance::henry_t L, newton_meters_per_ampere_t Kt, units::kilogram_square_meter_t J, double gearing, units::volt_t Vsupply, double b) 
            : R_{R} // resistance
            , L_{L} // inductance
            , Kt_{Kt} // torque constant
            , J_{J} // inertia
            , gearing_{gearing} // gear ratio
            , Vsupply_{Vsupply}
            , b_{b}
        {
        }

        DCMotorModel(const frc::DCMotor &motor, units::inductance::henry_t L, units::kilogram_square_meter_t J, double gearing, units::volt_t Vsupply, double b) 
            : R_{motor.R} // resistance
            , L_{L} // inductance
            , Kt_{motor.Kt} // torque constant
            , J_{J} // inertia
            , gearing_{gearing} // gear ratio
            , Vsupply_{Vsupply} // supply voltage
            , b_{b}
        {
            ROS_INFO_STREAM("Initializing motor model with R=" << R_.value() << ",L=" << L_.value() << ",Kt=" << Kt_.value() << ",J=" << J.value() << ",gearing=" << gearing_ << ",Vsupply=" << Vsupply_.value());
        }

        // Actually simulate the motor
        // State vector = x = [angular velocity, current]
        // Input/control vector = u = [applied voltage, external torque]

        // Derivative of state vector = ẋ = [angular acceleration, there isn't really a name for dCurrent/dt]
        Eigen::Vector2d update(Eigen::Vector2d x, Eigen::Vector2d u) const {
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
            // damping friction is 0.1 for now, idk what the right value is
            Eigen::Matrix2d A;
            A <<                    -b_/J_.value(), (Kt_.value()/J_.value())/gearing_,
                 (-Kt_.value()/L_.value())*gearing_, -R_.value()/L_.value();
            
            Eigen::Matrix2d B;
            B <<       0.0, 1.0/J_.value(),
                 Vsupply_.value()/L_, 0.0;

            // ROS_INFO_STREAM("motor model, x=" << x << ",u=" << u << ",A=" << A << ",B=" << B << ",y=" << A*x + B*u);
            
            return A*x + B*u;
        }
    
        double getB() const {
            return b_;
        }

        void setB(const double b) {
            b_ = b;
        }

        units::kilogram_square_meter_t getJ() const {
            return J_;
        }
        void setJ(const units::kilogram_square_meter_t J) {
            J_ = J;
        }
    private:

        units::ohm_t R_; // resistance
        units::inductance::henry_t L_; // inductance
        newton_meters_per_ampere_t Kt_; // torque constant
        units::kilogram_square_meter_t J_; // inertia
        double gearing_; // gear ratio
        units::volt_t Vsupply_;
        double b_; // damping friction
};

struct PacejkaConstants {
    double B;
    double C;
    double D;
    double E;
};

struct KienckeConstants {
    double A;
    double B;
};

template <size_t WHEELCOUNT>
class SwerveDynamics {
public:
    PacejkaConstants pacejka_constants_;
    KienckeConstants kiencke_constants_;
    DCMotorModel drive_motor_;
    DCMotorModel turn_motor_;
    units::meter_t wheel_radius_;
    Eigen::Matrix<double, WHEELCOUNT, 2> wheel_positions_;
    units::kilogram_t mass_;
    units::kilogram_square_meter_t moment_of_inertia_;
    ros::Publisher debug_pub_;

    SwerveDynamics() {
        // Initialize publisher
        ros::NodeHandle nh;
        debug_pub_ = nh.advertise<general_simulators::SwerveDebug>("swerve_debug", 1);
    }

    ~SwerveDynamics() = default;

    // Longitudinal force
    units::newton_t pacejka_model(double slip) {
        return units::newton_t{pacejka_constants_.D * sin(pacejka_constants_.C * atan(pacejka_constants_.B * slip - pacejka_constants_.E * (pacejka_constants_.B * slip - atan(pacejka_constants_.B * slip))))};
    }

    // Lateral force
    units::newton_t kiencke_model(double slip) {
        return units::newton_t{(2 * kiencke_constants_.A * kiencke_constants_.B * slip) / (kiencke_constants_.B * kiencke_constants_.B + slip * slip)};
    }

    std::pair<Eigen::Matrix<double, 5, 1>, Eigen::Vector2d> single_module(Eigen::Matrix<double, 5, 1> x, Eigen::Vector2d ground_speed, Eigen::Matrix<double, 2, 1> u) {
        // Ground speed should be relative to the robot reference frame
        
        // https://github.com/frc971/971-Robot-Code/blob/main/frc971/control_loops/swerve/swerve_notes.tex

        // x_motor = [ω_drive, I_drive, ω_turn, I_turn, theta]
        // u_motor = [V_drive, V_turn]
        // ẋ_motor = Ax + Bu
        // ẋ_motor = A[ω_drive, I_drive, ω_turn, I_turn, theta]ᵀ + B[V_drive, V_turn]ᵀ
        // y_motor = [force_in_x_direction, force_in_y_direction]

        Eigen::Matrix<double, 5, 1> xdot;

        units::meters_per_second_t wheel_speed{x[0] * wheel_radius_.value()};
        units::radian_t ground_speed_angle{atan2(ground_speed[1], ground_speed[0])};
        units::radian_t normalized_module_angle{atan2(sin(x[4]), cos(x[4]))};
        units::radian_t slip_angle = normalized_module_angle - ground_speed_angle;
        units::meters_per_second_t ground_speed_in_wheel_direction{ground_speed[0] * cos(x[4]) + ground_speed[1] * sin(x[4])}; 
        units::meters_per_second_t slip = wheel_speed - ground_speed_in_wheel_direction;

        units::newton_t longitudinal_force = pacejka_model(slip.value());
        units::newton_t lateral_force = kiencke_model(slip_angle.value());
        units::newton_t self_aligning_torque = -lateral_force * 0.0;

        units::newton_t x_direction_force = longitudinal_force * cos(x[4]) - lateral_force * sin(x[4]);
        units::newton_t y_direction_force = longitudinal_force * sin(x[4]) + lateral_force * cos(x[4]);

        xdot << drive_motor_.update(x.head(2), Eigen::Vector2d(u[0], -longitudinal_force.value())), // angular acceleration, derivative of current (drive),
                turn_motor_.update(x.segment(2, 2), Eigen::Vector2d(u[1], self_aligning_torque.value())), // angular acceleration, derivative of current (turn),
                x[2]; // angular velocity of azimuth
        
        Eigen::Vector2d y = Eigen::Vector2d(x_direction_force.value(), y_direction_force.value());

        // Print out the state vector and the output vector using ROS_INFO_STREAM
        // ROS_INFO_STREAM_THROTTLE(0.1, "x=" << x << ",xdot=" << xdot << ",u=" << u << ",y=" << y);
        general_simulators::SwerveDebug debug_msg;
        debug_msg.x = std::vector<double>(x.data(), x.data() + x.size());
        debug_msg.ground_speed = std::vector<double>(ground_speed.data(), ground_speed.data() + ground_speed.size());
        debug_msg.xdot = std::vector<double>(xdot.data(), xdot.data() + xdot.size());
        debug_msg.u = std::vector<double>(u.data(), u.data() + u.size());
        debug_msg.y = std::vector<double>(y.data(), y.data() + y.size());
        debug_pub_.publish(debug_msg);

        return std::make_pair(xdot, y);
    }

    std::pair<Eigen::Matrix<double, 5 * WHEELCOUNT + 2 + 1 + 2 + 1, 1>, Eigen::Vector2d> update(Eigen::Matrix<double, 5 * WHEELCOUNT + 2 + 1 + 2 + 1, 1> x, Eigen::Matrix<double, 2 * WHEELCOUNT, 1> u) {
        Eigen::Matrix<double, 5 * WHEELCOUNT + 2 + 1 + 2 + 1, 1> xdot;
        Eigen::Vector2d y;

        //      0       -          19    20    21    22   
        // x = [x_motor * WHEELCOUNT] + [xvel, yvel, omega]
        // y = [force_in_x_direction, force_in_y_direction]

        for (size_t i = 0; i < WHEELCOUNT; i++) {
            Eigen::Matrix<double, 5, 1> x_motor = x.segment(i * 5, 5);
            Eigen::Matrix<double, 2, 1> u_motor = u.segment(i * 2, 2);

            std::pair<Eigen::Matrix<double, 5, 1>, Eigen::Vector2d> result = single_module(x_motor, x.segment(5 * WHEELCOUNT, 2), u_motor);

            // Position of the module relative to the robot
            Eigen::Matrix<double, 2, 1> module_position = wheel_positions_.row(i);
            double module_position_angle = atan2(module_position[1], module_position[0]);

            // Angle from robot forward is represented as a rotation matrix
            Eigen::Matrix<double, 2, 2> rotation_matrix;
            rotation_matrix << cos(module_position_angle), -sin(module_position_angle),
                               sin(module_position_angle), cos(module_position_angle);

            // Rotate the forces into the robot reference frame
            Eigen::Vector2d robot_relative_forces = rotation_matrix * result.second;

            // Add module specific states to xdot
            xdot.segment(i * 5, 5) = result.first;

            // Add forces to result vector
            y += robot_relative_forces;

            // Add the acceleration contributed by the force to x and y velocities (971: equation 39)
            xdot.segment(5 * WHEELCOUNT, 2) += robot_relative_forces / mass_.value();

            // Add the torque contributed by the force to the angular acceleration (971: equation 40)
            // This is the cross product of the position vector of the module and the force vector divided by the moment of inertia
            // The vectors are represented as [x component, y component, 0]
            // so by the cross product formula on Wikipedia: {\displaystyle {\begin{bmatrix}s_{1}\\s_{2}\\s_{3}\end{bmatrix}}={\begin{bmatrix}a_{2}b_{3}-a_{3}b_{2}\\a_{3}b_{1}-a_{1}b_{3}\\a_{1}b_{2}-a_{2}b_{1}\end{bmatrix}}}
            // any term involving index 3 is zero, so we get a_{1}b_{2}-a_{2}b_{1}
            // which agrees with what Copilot helpfully told us
            xdot[5 * WHEELCOUNT + 2] += (module_position[0] * robot_relative_forces[1] - module_position[1] * robot_relative_forces[0]) / moment_of_inertia_.value();
        }

        return std::make_pair(xdot, y);
    }
};

namespace general_simulators
{
template <size_t WHEELCOUNT>
class SwerveSimulator : public simulator_base::Simulator
{
    public:
        SwerveSimulator() = default;

        void init(const XmlRpc::XmlRpcValue &simulator_info) override
        {
            // Get parameters from the parameter server in our namespace
            double gearing = simulator_info["gearing"]; // unitless
            PacejkaConstants pacejka_constants;
            pacejka_constants.B = simulator_info["pacejka_constants"]["B"];
            pacejka_constants.C = simulator_info["pacejka_constants"]["C"];
            pacejka_constants.D = simulator_info["pacejka_constants"]["D"];
            pacejka_constants.E = simulator_info["pacejka_constants"]["E"];
            KienckeConstants kiencke_constants;
            kiencke_constants.A = simulator_info["kiencke_constants"]["A"];
            kiencke_constants.B = simulator_info["kiencke_constants"]["B"];
            units::meter_t wheel_radius{simulator_info["wheel_radius"]}; // meters
            Eigen::Matrix<double, WHEELCOUNT, 2> wheel_positions;
            for (size_t i = 0; i < WHEELCOUNT; i++) {
                wheel_positions(i, 0) = simulator_info["wheel_positions"][i][0];
                wheel_positions(i, 1) = simulator_info["wheel_positions"][i][1];
                drive_joints_.push_back(simulator_info["drive_joints"][i]);
                turn_joints_.push_back(simulator_info["turn_joints"][i]);
            }
            units::kilogram_t mass{simulator_info["mass"]}; // kg
            units::kilogram_square_meter_t moment_of_inertia{simulator_info["moment_of_inertia"]}; // kg m^2

            swerve_dynamics_.pacejka_constants_ = pacejka_constants;
            swerve_dynamics_.kiencke_constants_ = kiencke_constants;
            swerve_dynamics_.wheel_radius_ = wheel_radius;
            swerve_dynamics_.wheel_positions_ = wheel_positions;
            swerve_dynamics_.mass_ = mass;
            swerve_dynamics_.moment_of_inertia_ = moment_of_inertia;

            // Zero out state and control vectors
            x.setZero();
            u.setZero();

            // Create a DCMotor object for drive and steer motors
            // If we're not using Krakens here, we're doing something wrong :)
            // (although azimuth motors could be Falcons, that's fine)
            frc::DCMotor drive = frc::DCMotor::KrakenX60FOC(1);
            frc::DCMotor turn = frc::DCMotor::KrakenX60FOC(1);

            // Create a DCMotorModel object
            // units::kilogram_square_meter_t kraken_inertia{0.1 * (0.95 * 0.0254) * (0.95 * 0.0254)}; // thanks 971
            units::kilogram_square_meter_t kraken_inertia(simulator_info["motor_inertia"]);
            swerve_dynamics_.drive_motor_ = DCMotorModel(drive, units::henry_t{0.001}, kraken_inertia, gearing, units::volt_t{12.0}, simulator_info["damping_friction"]);
            swerve_dynamics_.turn_motor_ = DCMotorModel(turn, units::henry_t{0.001}, kraken_inertia, gearing, units::volt_t{12.0}, simulator_info["damping_friction"]);

            // Set up NodeHandle and velocity pub
            ros::NodeHandle nh_;
            vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("swerve_simulator/actual_vel", 1);

            // Reset service
            reset_service_ = nh_.advertiseService("swerve_simulator/reset", &SwerveSimulator::reset, this);

            // Set up dynamic reconfigure
            ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(ros::NodeHandle(nh_, "swerve_simulator"));
            ddr_->registerVariable<double>("pacejka_constants/B", &swerve_dynamics_.pacejka_constants_.B, "B", 0.0, 20.0);
            ddr_->registerVariable<double>("pacejka_constants/C", &swerve_dynamics_.pacejka_constants_.C, "C", 0.0, 3.0);
            ddr_->registerVariable<double>("pacejka_constants/D", &swerve_dynamics_.pacejka_constants_.D, "D", 0.0, 2.0);
            ddr_->registerVariable<double>("pacejka_constants/E", &swerve_dynamics_.pacejka_constants_.E, "E", 0.5, 1.0);
            ddr_->registerVariable<double>("kiencke_constants/A", &swerve_dynamics_.kiencke_constants_.A, "A", 0.0, 100.0);
            ddr_->registerVariable<double>("kiencke_constants/B", &swerve_dynamics_.kiencke_constants_.B, "B", 0.0, 100.0);
            // ddr_->registerVariable<double>("wheel_radius", &swerve_dynamics_.wheel_radius_.value(), "m", 0.0, 0.5);
            ddr_->registerVariable<double>("motor_inertia", [this](){ return swerve_dynamics_.drive_motor_.getJ().value(); }, [this](const double J) { swerve_dynamics_.drive_motor_.setJ(units::kilogram_square_meter_t{J}); swerve_dynamics_.turn_motor_.setJ(units::kilogram_square_meter_t{J}); }, "kg m^2", 0.00001, 0.005);
            ddr_->registerVariable<double>("damping_friction", [this](){ return swerve_dynamics_.drive_motor_.getB(); }, [this](const double b) { swerve_dynamics_.drive_motor_.setB(b); swerve_dynamics_.turn_motor_.setB(b); }, "N m s", 0.00001, 0.01);
            ddr_->publishServicesTopics();

        }

        void update(const std::string &name, const ros::Time &time, const ros::Duration &period, hardware_interface::talonfxpro::TalonFXProSimCommand *talonfxpro, const hardware_interface::talonfxpro::TalonFXProHWState *state, std::optional<hardware_interface::cancoder::CANCoderSimCommandHandle> cancoder) override
        {
            // Find which motor we have here (see if it's in the drive motors list or turn motors list and what index)
            size_t motor_index = std::find(drive_joints_.begin(), drive_joints_.end(), name) - drive_joints_.begin();
            size_t is_turn = 0;
            if (motor_index == drive_joints_.size()) {
                motor_index = std::find(turn_joints_.begin(), turn_joints_.end(), name) - turn_joints_.begin();
                is_turn = 1;
            }

            if (is_turn && !set_initial_angle_[motor_index]) {
                // Set the initial angle of the turn motor
                x[motor_index * 5 + 4] = state->getPosition() + (M_PI / 2.);
                set_initial_angle_[motor_index] = true;
            }

            // Get the motor voltage from the state
            const double invert = state->getInvert() == hardware_interface::talonfxpro::Inverted::Clockwise_Positive ? -1.0 : 1.0;
            units::voltage::volt_t motor_voltage{state->getMotorVoltage() * invert};

            last_read_times_[motor_index * 2 + is_turn] = time;

            // Get the applied torque current from the state and set it in the state vector
            units::ampere_t motor_current{state->getTorqueCurrent()};
            x[motor_index * 5 + 1 + is_turn * 2] = motor_current.value() * (is_turn ? 0 : 1);

            // Update the control vector
            u[motor_index * 2 + is_turn] = motor_voltage.value() * (is_turn ? 0 : 1); // don't sim turn motor for now, just test straight driving forward hopefully

            // ROS_INFO_STREAM("Motor " << name << " voltage: " << motor_voltage.value() << "V, read time is " << last_read_times_[motor_index * 2 + is_turn]);

            // If all of the times in last_read_times_ are equal, update swerve dynamics
            if (std::adjacent_find(std::begin(last_read_times_), std::end(last_read_times_), std::not_equal_to<>()) == std::end(last_read_times_))
            {
                // Update the swerve dynamics
                auto result = swerve_dynamics_.update(x, u);

                // Integrate xdot and add to x
                x += result.first * period.toSec();

                // Update the state vector
                x.segment(5 * WHEELCOUNT, 2) += result.second;

                geometry_msgs::TwistStamped twist;
                twist.header.frame_id = "base_link";
                twist.header.stamp = time;
                twist.twist.linear.x = x[5 * WHEELCOUNT];
                twist.twist.linear.y = x[5 * WHEELCOUNT + 1];
                twist.twist.angular.z = x[5 * WHEELCOUNT + 2];
                // debug info: angular x one wheel speed, angular y another wheel speed
                twist.twist.angular.x = result.first[0];
                twist.twist.angular.y = result.first[5];
                vel_pub_.publish(twist);
            }

            units::radians_per_second_t angular_velocity{x[motor_index * 5 + is_turn * 2]};

            // Set the velocity of the simulated motor to the actual wheel speed
            talonfxpro->setRotorVelocity(angular_velocity.value());

            // Add position delta
            talonfxpro->setAddRotorPosition((angular_velocity * units::second_t{period.toSec()}).value());

            this->update_cancoder(state, cancoder);
        }

        bool reset(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/) {
            // Zero out state and control vectors
            x.setZero();
            u.setZero();
            // for (size_t i = 0; i < WHEELCOUNT; i++) {
            //     set_initial_angle_[i] = false;
            // }
            return true;
        }

        ~SwerveSimulator() override = default;

    private:
        SwerveDynamics<WHEELCOUNT> swerve_dynamics_;
        std::vector<std::string> drive_joints_;
        std::vector<std::string> turn_joints_;
        std::array<bool, WHEELCOUNT> set_initial_angle_;
        Eigen::Matrix<double, 5 * WHEELCOUNT + 2 + 1 + 2 + 1, 1> x;
        Eigen::Matrix<double, 2 * WHEELCOUNT, 1> u;
        std::array<ros::Time, 2 * WHEELCOUNT> last_read_times_;

        ros::Publisher vel_pub_;
        ros::ServiceServer reset_service_;
        std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

        /*
        # Tire models
        def pacejka(constants, slip): # constants: [B,C,D,E] (same names as formula mentioned here: https://en.wikipedia.org/wiki/Hans_B._Pacejka)
            b = constants[0]
            c = constants[1]
            d = constants[2]
            e = constants[3]
            return d*cs.sin(c*cs.atan(b*slip-e*(b*slip-cs.atan(b*slip))))

        def kiencke(constants, slip):
            a = constants[0]
            b = constants[1]
            return (2*a*b*slip)/(b*b + slip*slip)
        */

};

};

#endif
