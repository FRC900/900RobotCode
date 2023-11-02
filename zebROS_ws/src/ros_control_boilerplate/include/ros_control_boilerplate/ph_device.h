#ifndef PH_DEVICE_INC__
#define PH_DEVICE_INC__

#include <mutex>
#include <string>
#include <thread>

namespace frc
{
    class PneumaticHub;
}
namespace hardware_interface
{
    class PHHWState;
    class PHHWCommand;
    class PHStateInterface;
    class PHCommandInterface;
    class RemotePHStateInterface;
}
class Tracer;

class PHDevice
{
public:
    PHDevice(const std::string &name_space,
             const int joint_index,
             const std::string &joint_name,
             const int ph_id,
             const bool local_read,
             const bool local_write,
             const double read_hz);
    PHDevice(const PHDevice &) = delete;
    PHDevice(PHDevice &&other) noexcept = delete;
    virtual ~PHDevice();

    PHDevice &operator=(const PHDevice &) = delete;
    PHDevice &operator=(PHDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::PHStateInterface &state_interface,
                            hardware_interface::PHCommandInterface &command_interface,
                            hardware_interface::RemotePHStateInterface &remote_state_interface);
    void read(const ros::Time &time, const ros::Duration &period);
    void write(const ros::Time &time, const ros::Duration &period);

private:
    const std::string name_;
    const int ph_id_;
    const bool local_read_;
    const bool local_write_;
    std::unique_ptr<frc::PneumaticHub> ph_;

    std::unique_ptr<hardware_interface::PHHWState> state_;
    std::unique_ptr<hardware_interface::PHHWCommand> command_;

    std::unique_ptr<hardware_interface::PHHWState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::jthread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     double poll_frequency);
};

#endif