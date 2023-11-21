#ifndef CTRE_V5_MOTOR_CONTROLLERS_H_INC__
#define CTRE_V5_MOTOR_CONTROLLERS_H_INC__

#include "ros_control_boilerplate/devices.h"
#include "ros_control_boilerplate/set_limit_switch.h"
#include "ros_control_boilerplate/set_current.h"

class CTREV5MotorController;
class SimCTREV5MotorController;
namespace hardware_interface
{
    class TalonStateInterface;
    class TalonCommandInterface;
    class RemoteTalonStateInterface;
}

namespace ctre::phoenix::motorcontrol::can
{
    class TalonFX;
}

template <bool SIM>
class CTREV5MotorControllers : public Devices
{
public:
    explicit CTREV5MotorControllers(ros::NodeHandle &root_nh);
    CTREV5MotorControllers(const CTREV5MotorControllers &) = delete;
    CTREV5MotorControllers(CTREV5MotorControllers &&) noexcept = delete;
    ~CTREV5MotorControllers() override;

    CTREV5MotorControllers &operator=(const CTREV5MotorControllers &) = delete;
    CTREV5MotorControllers &operator=(CTREV5MotorControllers &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void simInit(ros::NodeHandle &nh) override;
    void simRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    bool gazeboSimInit(const ros::NodeHandle& /*nh*/, boost::shared_ptr<gazebo::physics::Model> parent_model) override;
    void gazeboSimRead(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& tracer) override;
    void gazeboSimWrite(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& tracer, const bool e_stop_active) override;

private:
    double read_hz_{100};
    using DEVICE_TYPE = std::conditional_t<SIM, SimCTREV5MotorController, CTREV5MotorController>;
    std::vector<std::unique_ptr<DEVICE_TYPE>> devices_;
    std::unique_ptr<hardware_interface::TalonStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::TalonCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::RemoteTalonStateInterface> remote_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;

    bool setlimit(ros_control_boilerplate::set_limit_switch::Request &req,
                  ros_control_boilerplate::set_limit_switch::Response &res);
    bool setcurrent(ros_control_boilerplate::set_current::Request &req,
                    ros_control_boilerplate::set_current::Response &res);

    ros::ServiceServer sim_limit_switch_srv_;
    ros::ServiceServer sim_current_srv_;

    // Used to disable motors on transition from enabled->disabled
    // Start with this == true so the first iteration through
    // write() forces that enabled->disabled transition disabling all motors on startup
    bool prev_robot_enabled_{true};
};

#endif