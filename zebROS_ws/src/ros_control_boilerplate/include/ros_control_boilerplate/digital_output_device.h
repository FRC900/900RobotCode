#ifndef DIGITAL_OUTPUT_DEVICE_INC__
#define DIGITAL_OUTPUT_DEVICE_INC__

#include <memory>
#include <string>

namespace frc { class DigitalOutput; }
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class DigitalOutputDevice
{
public:
    DigitalOutputDevice(const int joint_index,
                        const std::string &joint_name,
                        const int dio_channel,
                        const bool invert,
                        const bool local_hardware,
                        const bool local_update);
    DigitalOutputDevice(const DigitalOutputDevice &) = delete;
    DigitalOutputDevice(DigitalOutputDevice &&other) noexcept = delete;
    virtual ~DigitalOutputDevice();

    DigitalOutputDevice &operator=(const DigitalOutputDevice &) = delete;
    DigitalOutputDevice &operator=(DigitalOutputDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::JointCommandInterface &command_interface,
                            hardware_interface::PositionJointInterface &position_joint_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface);
    void write(const ros::Time& time, const ros::Duration& period);

private:
    const std::string name_;
    const int dio_channel_;
    const bool invert_;
    const bool local_hardware_;
    const bool local_update_;
    std::unique_ptr<frc::DigitalOutput> digital_output_;

    double state_{std::numeric_limits<double>::max()};
    double command_{0};
};

#endif