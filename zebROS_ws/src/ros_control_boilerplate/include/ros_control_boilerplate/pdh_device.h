#ifndef PDH_DEVICE_INC__
#define PDH_DEVICE_INC__

#include <mutex>
#include <thread>

typedef int32_t HAL_Handle;
typedef HAL_Handle HAL_REVPDHHandle;

#include <string>

namespace frc
{
    class PneumaticHub;
}
namespace hardware_interface
{
    class PDHHWState;
    class PDHHWCommand;
    class PDHStateInterface;
    class PDHCommandInterface;
    class RemotePDHStateInterface;
}
class Tracer;

class PDHDevice
{
public:
    PDHDevice(const std::string &name_space,
              const int joint_index,
              const std::string &joint_name,
              const int pdh_id,
              const bool local_read,
              const bool local_write,
              const double read_hz);
    PDHDevice(const PDHDevice &) = delete;
    PDHDevice(PDHDevice &&other) noexcept = delete;
    ~PDHDevice();

    PDHDevice &operator=(const PDHDevice &) = delete;
    PDHDevice &operator=(PDHDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::PDHStateInterface &state_interface,
                            hardware_interface::PDHCommandInterface &command_interface,
                            hardware_interface::RemotePDHStateInterface &remote_state_interface);
    void read(const ros::Time &/*time*/, const ros::Duration &/*period*/);
    void write(const ros::Time &/*time*/, const ros::Duration &/*&period*/);

private:
    const std::string name_;
    const int pdh_module_;
    const bool local_read_;
    const bool local_write_;

    HAL_REVPDHHandle pdh_handle_;
    std::unique_ptr<hardware_interface::PDHHWState> state_;
    std::unique_ptr<hardware_interface::PDHHWCommand> command_;

    std::unique_ptr<hardware_interface::PDHHWState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::thread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     const double poll_frequency);
};

#endif