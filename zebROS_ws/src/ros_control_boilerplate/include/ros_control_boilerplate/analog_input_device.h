#ifndef ANALOG_INPUT_DEVICE_INC__
#define ANALOG_INPUT_DEVICE_INC__

#include <memory>
#include <string>

namespace frc { class AnalogInput; }
namespace hardware_interface
{
    class JointStateInterface;
    class RemoteJointInterface;
}

class AnalogInputDevice
{
public:
    AnalogInputDevice(const int joint_index,
                      const std::string &joint_name,
                      const int ain_channel,
                      const double a,
                      const double b,
                      const bool local);
    AnalogInputDevice(const AnalogInputDevice &) = delete;
    AnalogInputDevice(AnalogInputDevice &&) noexcept = delete;
    ~AnalogInputDevice();
    AnalogInputDevice& operator=(const AnalogInputDevice &) = delete;
    AnalogInputDevice& operator=(AnalogInputDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface);
    void read(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    const std::string &getName(void) const;
    void setSimValue(const double value) const;

private:
    std::string name_;
    int ain_channel_;
    double a_;
    double b_;
    bool local_;
    std::unique_ptr<frc::AnalogInput> analog_input_;

    double state_{0};
};

#endif