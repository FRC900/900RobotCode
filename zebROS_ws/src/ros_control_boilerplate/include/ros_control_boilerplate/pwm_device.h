#ifndef PWM_DEVICE_INC__
#define PWM_DEVICE_INC__

#include <memory>
#include <string>

namespace frc { class PWM; }
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class PWMDevice
{
public:
    PWMDevice(const int joint_index,
              const std::string &joint_name,
              const int pwm_channel,
              const double output_max,
              const double deadband_max,
              const double center,
              const double output_min,
              const double deadband_min,
              const int period_multiplier,
              const bool invert,
              const bool local_hardware,
              const bool local_update,
              const bool print = true);
    PWMDevice(const PWMDevice &) = delete;
    PWMDevice(PWMDevice &&other) noexcept = delete;
    virtual ~PWMDevice();

    PWMDevice &operator=(const PWMDevice &) = delete;
    PWMDevice &operator=(PWMDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::JointCommandInterface &command_interface,
                            hardware_interface::PositionJointInterface &position_joint_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface);
    void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

protected:
    virtual void writeImpl(const double value);
    std::unique_ptr<frc::PWM> pwm_;

private:
    const std::string name_;
    const int pwm_channel_;
    const bool invert_;
    const bool local_hardware_;
    const bool local_update_;

    double state_{std::numeric_limits<double>::max()};
    double command_{0};
};

#endif