#ifndef DIGITAL_INPUT_DEVICE_INC__
#define DIGITAL_INPUT_DEVICE_INC__

#include <memory>
#include <string>

namespace frc { class DigitalInput; }
namespace hardware_interface
{
    class JointStateInterface;
    class RemoteJointInterface;
}

class DigitalInputDevice
{
public:
    DigitalInputDevice(const int joint_index,
                       const std::string &joint_name,
                       const int dio_channel,
                       const bool invert,
                       const bool local);
    DigitalInputDevice(const DigitalInputDevice &) = delete;
    DigitalInputDevice(DigitalInputDevice &&other) noexcept = delete;
    virtual ~DigitalInputDevice();

    DigitalInputDevice &operator=(const DigitalInputDevice &) = delete;
    DigitalInputDevice &operator=(DigitalInputDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface);
    void read(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    const std::string &getName(void) const;
    void setSimValue(const bool value) const;

private:
    const std::string name_;
    const int dio_channel_;
    const bool invert_;
    const bool local_;
    std::unique_ptr<frc::DigitalInput> digital_input_;

    double state_{0};
};

#endif