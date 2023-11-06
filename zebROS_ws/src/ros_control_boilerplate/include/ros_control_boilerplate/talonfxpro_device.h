#ifndef TALONFXPRO_DEVICE_INC__
#define TALONFXPRO_DEVICE_INC__

#include <mutex>
#include <string>
#include <thread>

#include "ros/node_handle.h"
#include "ros_control_boilerplate/ctre_v6_device.h"

namespace ctre::phoenix6
{
    namespace configs
    {
        class TalonFXConfiguration;
    }
    namespace hardware::core
    {
        class CoreTalonFX;
    }
}

namespace hardware_interface::talonfxpro
{
    class TalonFXProHWState;
    class TalonFXProHWCommand;
    class TalonFXProStateInterface;
    class TalonFXProCommandInterface;
}

class Tracer;

class TalonFXProDevice : public CTREV6Device
{
public:
    TalonFXProDevice(const std::string &name_space,
                     const int joint_index,
                     const std::string &joint_name,
                     const int can_id,
                     const std::string &can_bus,
                     double read_hz);
    TalonFXProDevice(const TalonFXProDevice &) = delete;
    TalonFXProDevice(TalonFXProDevice &&other) noexcept = delete;
    ~TalonFXProDevice() override;

    TalonFXProDevice &operator=(const TalonFXProDevice &) = delete;
    TalonFXProDevice &operator=(TalonFXProDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::talonfxpro::TalonFXProStateInterface &state_interface,
                            hardware_interface::talonfxpro::TalonFXProCommandInterface &command_interface) const;
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period, const bool curr_robot_enabled, const bool prev_robot_enabled);

    // Read and write functions which add additional sim features
    void simRead(const ros::Time& time, const ros::Duration& period);

    int getCANID(void) const;

    bool setSimLimitSwitches(const bool forward_limit, const bool reverse_limit);
    bool setSimCurrent(const double stator_current, const double supply_current);

private:
    const std::string can_bus_;

    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWState> state_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWCommand> command_;
    std::unique_ptr<ctre::phoenix6::configs::TalonFXConfiguration> config_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<hardware_interface::talonfxpro::TalonFXProHWState> read_thread_state_;
    std::unique_ptr<ctre::phoenix6::hardware::core::CoreTalonFX> talonfxpro_;
    std::unique_ptr<std::jthread> read_thread_;

    void read_thread(std::unique_ptr<Tracer> tracer,
                     const int joint_idx,
                     const double poll_frequency);
};

#endif