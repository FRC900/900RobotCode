#ifndef SIM_CTRE_V5_MOTOR_CONTROLLER_INC__
#define SIM_CTRE_V5_MOTOR_CONTROLLER_INC__

#include <ros/time.h>
#include <string>
#include "ros_control_boilerplate/ctre_v5_motor_controller.h"

namespace gazebo::physics
{
    class Joint;
    class Model;
}

class SimCTREV5MotorController : public CTREV5MotorController
{
public:
    SimCTREV5MotorController(const std::string &name_space,
                             const int joint_index,
                             const std::string &joint_name,
                             const std::string &joint_type,
                             const int can_id,
                             const std::string &can_bus,
                             const bool local_,
                             const double read_hz);
    SimCTREV5MotorController(SimCTREV5MotorController &&other) noexcept = delete;
    SimCTREV5MotorController(const SimCTREV5MotorController &) = delete;
    ~SimCTREV5MotorController() override = default;
    SimCTREV5MotorController &operator=(const SimCTREV5MotorController &) = delete;
    SimCTREV5MotorController &operator=(SimCTREV5MotorController &&) noexcept = delete;

    void updateSimValues(const ros::Time &/*time*/, const ros::Duration &period);

    bool setSimLimitSwitches(const bool forward_limit, const bool reverse_limit);
    bool setSimCurrent(const double stator_current, const double supply_current);

    bool gazeboInit(boost::shared_ptr<gazebo::physics::Model> parent_model);
    void gazeboRead();
    void gazeboWrite(const bool e_stop_active_);

private:
    void setSimCollection(int position, int velocity, int delta_position = 0) const;
    void setSimCollectionTalonSRX(int position, int velocity, int delta_position) const;

    boost::shared_ptr<gazebo::physics::Joint> gazebo_joint_;
};

#endif