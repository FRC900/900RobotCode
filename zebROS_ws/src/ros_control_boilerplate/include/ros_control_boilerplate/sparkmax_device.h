#ifndef SPARKMAX_DEVICE_INC__
#define SPARKMAX_DEVICE_INC__

#include <mutex>
#include <string>
#include <thread>

#include "rev/REVLibError.h"

namespace rev
{
    class CANSparkMax;
    class SparkMaxPIDController;
}
#include "ctre/phoenix/ErrorCode.h"
namespace hardware_interface
{
    class SparkMaxHWState;
    class SparkMaxHWCommand;
    class SparkMaxStateInterface;
    class SparkMaxCommandInterface;
    class RemoteSparkMaxStateInterface;
    enum MotorType;
}
class Tracer;

template <bool SIMFLAG>
class SparkMaxDevice
{
public:
    SparkMaxDevice(const std::string &name_space,
                   const int joint_index,
                   const std::string &joint_name,
                   const int spark_max_id,
                   const hardware_interface::MotorType motor_type,
                   const bool local_hardware,
                   const bool local_update,
                   const double read_hz);
    SparkMaxDevice(const SparkMaxDevice &) = delete;
    SparkMaxDevice(SparkMaxDevice &&other) noexcept = delete;
    virtual ~SparkMaxDevice();

    SparkMaxDevice &operator=(const SparkMaxDevice &) = delete;
    SparkMaxDevice &operator=(SparkMaxDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::SparkMaxStateInterface &state_interface,
                            hardware_interface::SparkMaxCommandInterface &command_interface,
                            hardware_interface::RemoteSparkMaxStateInterface &remote_state_interface) const;
    void read(const ros::Time &time, const ros::Duration &period);
    void write(const ros::Time &time, const ros::Duration &period);

private:
    const std::string name_;
    const int can_id_;
    const bool local_hardware_;
    const bool local_update_;

    static inline size_t can_error_count_{0};
    static inline bool can_error_sent_{false};
    std::unique_ptr<rev::CANSparkMax> can_spark_max_;
    std::unique_ptr<rev::SparkMaxPIDController> pid_controller_;

    std::unique_ptr<hardware_interface::SparkMaxHWState> state_;
    std::unique_ptr<hardware_interface::SparkMaxHWCommand> command_;

    std::unique_ptr<hardware_interface::SparkMaxHWState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::jthread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     double poll_frequency);

    bool safeCall(const rev::REVLibError can_error,
                  const std::string &spark_max_method_name);
};

#endif