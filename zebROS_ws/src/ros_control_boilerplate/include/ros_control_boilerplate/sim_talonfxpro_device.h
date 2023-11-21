#ifndef SIM_TALONFXPRO_DEVICE_INC__
#define SIM_TALONFXPRO_DEVICE_INC__

#include "ros_control_boilerplate/talonfxpro_device.h"

namespace gazebo::physics
{
    class Joint;
    class Model;
}

class SimTalonFXProDevice : public TalonFXProDevice
{
public:
    SimTalonFXProDevice(const std::string &name_space,
                        const int joint_index,
                        const std::string &joint_name,
                        const int can_id,
                        const std::string &can_bus,
                        double read_hz);
    SimTalonFXProDevice(const SimTalonFXProDevice &) = delete;
    SimTalonFXProDevice(SimTalonFXProDevice &&other) noexcept = delete;
    virtual ~SimTalonFXProDevice() = default;

    SimTalonFXProDevice &operator=(const SimTalonFXProDevice &) = delete;
    SimTalonFXProDevice &operator=(SimTalonFXProDevice &&) noexcept = delete;

    // Read and write functions which add additional sim features
    void simRead(const ros::Time& time, const ros::Duration& period);

    bool setSimLimitSwitches(const bool forward_limit, const bool reverse_limit);
    bool setSimCurrent(const double stator_current, const double supply_current);

    bool gazeboInit(boost::shared_ptr<gazebo::physics::Model> parent_model);
private:
    boost::shared_ptr<gazebo::physics::Joint> gazebo_joint_;
    int counter = 0;
};

#endif