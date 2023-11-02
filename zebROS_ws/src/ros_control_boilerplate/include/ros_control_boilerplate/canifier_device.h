#ifndef CANIFIER_DEVICE_INC__
#define CANIFIER_DEVICE_INC__

#include <string>
#include <thread>
#include "ros_control_boilerplate/ctre_v5_device.h"

namespace ctre::phoenix
{
    class CANifier;
}
namespace hardware_interface::canifier
{
    class CANifierHWState;
    class CANifierHWCommand;
    class CANifierStateInterface;
    class CANifierCommandInterface;
    class RemoteCANifierStateInterface;
}
class Tracer;

template <bool SIMFLAG>
class CANifierDevice : public CTREV5Device
{
public:
    CANifierDevice(const std::string &name_space,
                   const int joint_index,
                   const std::string &joint_name,
                   const int canifier_id,
                   const bool local_hardware,
                   const bool local_update,
                   const double read_hz);
    CANifierDevice(const CANifierDevice &) = delete;
    CANifierDevice(CANifierDevice &&other) noexcept = delete;
    ~CANifierDevice() override;
    CANifierDevice &operator=(const CANifierDevice &) = delete;
    CANifierDevice &operator=(CANifierDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::canifier::CANifierStateInterface &state_interface,
                            hardware_interface::canifier::CANifierCommandInterface &command_interface,
                            hardware_interface::canifier::RemoteCANifierStateInterface &remote_state_interface) const;
    void read(const ros::Time &/*time*/, const ros::Duration &/*period*/);
    void write(const ros::Time &/*time*/, const ros::Duration &/*period*/);

private:
    const bool local_hardware_;
    const bool local_update_;

    std::unique_ptr<ctre::phoenix::CANifier> canifier_{};

    std::unique_ptr<hardware_interface::canifier::CANifierHWState> state_;
    std::unique_ptr<hardware_interface::canifier::CANifierHWCommand> command_;

    std::unique_ptr<hardware_interface::canifier::CANifierHWState> read_thread_state_{};
    std::unique_ptr<std::mutex> read_state_mutex_{};
    std::unique_ptr<std::jthread> read_thread_{};
    void read_thread(std::unique_ptr<Tracer> tracer,
                     double poll_frequency);
};

#endif