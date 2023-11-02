#ifndef AS726X_DEVICE_INC__
#define AS726X_DEVICE_INC__

#include <string>
#include <thread>

namespace as726x
{
    template <bool SIMFLAG> class roboRIO_AS726x;
}
namespace hardware_interface::as726x
{
    class AS726xState;
    class AS726xCommand;
    class AS726xStateInterface;
    class AS726xCommandInterface;
    class RemoteAS726xStateInterface;
}
class Tracer;

template <bool SIMFLAG>
class AS726xDevice
{
public:
    AS726xDevice(const std::string &name_space,
                 const int joint_index,
                 const std::string &joint_name,
                 const std::string &port,
                 const int address,
                 const bool local_hardware,
                 const bool local_update,
                 const double read_hz);
    AS726xDevice(AS726xDevice &&other) noexcept = delete;
    AS726xDevice(const AS726xDevice &) = delete;
    virtual ~AS726xDevice();
    AS726xDevice &operator=(const AS726xDevice &) = delete;
    AS726xDevice &operator=(AS726xDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::as726x::AS726xStateInterface &state_interface,
                            hardware_interface::as726x::AS726xCommandInterface &command_interface,
                            hardware_interface::as726x::RemoteAS726xStateInterface &remote_state_interface) const;
    void read(const ros::Time &/*time*/, const ros::Duration &/*period*/);
    void write(const ros::Time &/*time*/, const ros::Duration &/*period*/);

private:
    const std::string name_;
    const std::string port_;
    const int address_;
    const bool local_hardware_;
    const bool local_update_;

    std::unique_ptr<as726x::roboRIO_AS726x<SIMFLAG>> as726x_;

    std::unique_ptr<hardware_interface::as726x::AS726xState> state_;
    std::unique_ptr<hardware_interface::as726x::AS726xCommand> command_;

    std::unique_ptr<hardware_interface::as726x::AS726xState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::jthread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     double poll_frequency);
};

#endif