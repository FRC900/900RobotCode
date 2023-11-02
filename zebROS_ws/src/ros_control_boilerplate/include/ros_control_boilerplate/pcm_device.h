#ifndef PCM_DEVICE_INC__
#define PCM_DEVICE_INC__

#include <mutex>
#include <string>
#include <thread>

namespace frc { class PneumaticsControlModule; }
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
    class PCMStateInterface;
    class PCMState;
    class RemotePCMStateInterface;
}
class Tracer;

class PCMDevice
{
public:
    PCMDevice(const std::string &name_space,
              const int joint_index,
              const std::string &joint_name,
              const int pcm_id,
              const bool local_hardware,
              const bool local_update,
              const double read_hz);
    PCMDevice(const PCMDevice &) = delete;
    PCMDevice(PCMDevice &&other) noexcept = delete;
    ~PCMDevice();

    PCMDevice &operator=(const PCMDevice &) = delete;
    PCMDevice &operator=(PCMDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::JointCommandInterface &command_interface,
                            hardware_interface::PositionJointInterface &position_joint_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface,
                            hardware_interface::PCMStateInterface &pcm_state_interface,
                            hardware_interface::RemotePCMStateInterface &remote_pcm_state_interface);
    void read(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

private:
    const std::string name_;
    const int pcm_id_;
    const bool local_hardware_;
    const bool local_update_;
    std::unique_ptr<frc::PneumaticsControlModule> pcm_;

    // Default compressors to running and set state to a different
    // value to force a write to HW the first write() iteration
    double enable_state_{std::numeric_limits<double>::max()};
    double enable_command_{1};

    std::unique_ptr<hardware_interface::PCMState> state_;

    std::unique_ptr<hardware_interface::PCMState> read_thread_state_;
    std::unique_ptr<std::mutex> read_state_mutex_;
    std::unique_ptr<std::jthread> read_thread_;
    void read_thread(std::unique_ptr<Tracer> tracer,
                     double poll_frequency);
};

#endif