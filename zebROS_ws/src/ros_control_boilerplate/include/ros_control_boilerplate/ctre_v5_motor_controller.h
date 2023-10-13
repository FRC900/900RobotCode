#ifndef CTRE_V5_MOTOR_CONTROLLER_INC__
#define CTRE_V5_MOTOR_CONTROLLER_INC__

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include "ros_control_boilerplate/ctre_v5_device.h"

namespace ctre::phoenix::motorcontrol
{
    class IMotorController;
    class IMotorControllerEnhanced;
    namespace can
    {
        class TalonFX;
        class TalonSRX;
        class VictorSPX;
    }
}
namespace hardware_interface
{
    class TalonStateInterface;
    class TalonCommandInterface;
    class RemoteTalonStateInterface;
    class TalonHWState;
    class TalonHWCommand;
}
class Tracer;

class CTREV5MotorController : public CTREV5Device
{
public:
    CTREV5MotorController(const std::string &name_space,
                          const int joint_index,
                          const std::string &joint_name,
                          const std::string &joint_type,
                          const int can_id,
                          const std::string &can_bus,
                          const bool local_,
                          const double read_hz);
    CTREV5MotorController(CTREV5MotorController &&other) noexcept = delete;
    CTREV5MotorController(const CTREV5MotorController &) = delete;
    ~CTREV5MotorController();
    CTREV5MotorController &operator=(const CTREV5MotorController &) = delete;
    CTREV5MotorController &operator=(CTREV5MotorController &&) noexcept = delete;

    void registerInterfaces(hardware_interface::TalonStateInterface &state_interface,
                            hardware_interface::TalonCommandInterface &command_interface,
                            hardware_interface::RemoteTalonStateInterface &remote_state_interface);
    void read(const ros::Time &time, const ros::Duration &period);
    void write(const ros::Time &time, const ros::Duration &period, const bool robot_enabled, const bool prev_robot_enabled);

    void updateSimValues(const ros::Time &/*time*/, const ros::Duration &period);

    bool setSimLimitSwitches(const bool forward_limit, const bool reverse_limit);
    bool setSimCurrent(const double stator_current, const double supply_current);

private:
    const bool local_;

    std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController> ctre_mc_;
    std::shared_ptr<ctre::phoenix::motorcontrol::IMotorControllerEnhanced> talon_;
    std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonFX> talon_fx_;
    std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> talon_srx_;
    std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController> victor_spx_;

    std::unique_ptr<hardware_interface::TalonHWState> state_;
    std::unique_ptr<hardware_interface::TalonHWCommand> command_;

    std::unique_ptr<hardware_interface::TalonHWState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::thread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     const double poll_frequency);


    void setSimCollection(double position, double velocity, double delta_position = 0) const;
    void setSimCollectionTalonSRX(double position, double velocity, double delta_position) const;
    void setSimCollectionTalonFX(double position, double velocity, double delta_position) const;



    static constexpr int pidIdx{0};    // 0 for primary closed-loop, 1 for cascaded closed-loop
    static constexpr int timeoutMs{0}; // If nonzero, function will wait for config success and report an error if it times out. If zero, no blocking or checking is performed
    static constexpr int configTimeoutMs{10};
};

#endif