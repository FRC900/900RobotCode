#ifndef PDP_DEVICE_INC__
#define PDP_DEVICE_INC__

#include <mutex>
#include <string>
#include <thread>

namespace frc
{
    class PDP;
}
namespace hardware_interface
{
    class PDPHWState;
    class PDPStateInterface;
    class RemotePDPStateInterface;
}
class PeriodicIntervalCounter;
class Tracer;

class PDPDevice
{
public:
    PDPDevice(const std::string &name_space,
              const int joint_index,
              const std::string &joint_name,
              const int pdp_module,
              const bool local,
              const double read_hz_);
    PDPDevice(const PDPDevice &) = delete;
    PDPDevice(PDPDevice &&other) noexcept = delete;
    ~PDPDevice();

    PDPDevice &operator=(const PDPDevice &) = delete;
    PDPDevice &operator=(PDPDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::PDPStateInterface &state_interface,
                            hardware_interface::RemotePDPStateInterface &remote_state_interface);
    void read(const ros::Time &time, const ros::Duration &period);

private:
    const std::string name_;
    const int32_t pdp_module_;
    const bool local_;

    std::unique_ptr<hardware_interface::PDPHWState> state_;
    std::unique_ptr<hardware_interface::PDPHWState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::thread> read_thread_;
    void read_thread(int32_t pdp,
                     std::unique_ptr<Tracer> tracer,
                     double poll_frequency);
};

#endif