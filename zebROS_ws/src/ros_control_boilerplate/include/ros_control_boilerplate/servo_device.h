#ifndef SERVO_DEVICE_INC__
#define SERVO_DEVICE_INC__

#include <memory>
#include <string>

#include "ros_control_boilerplate/pwm_device.h"
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
}

class ServoDevice : public PWMDevice
{
public:
    ServoDevice(const int joint_index,
                const std::string &joint_name,
                const int pwm_channel,
                const double output_max,
                const double output_min,
                const int period_multiplier,
                const bool invert,
                const bool local_hardware,
                const bool local_update);
    ServoDevice(const ServoDevice &) = delete;
    ServoDevice(ServoDevice &&other) noexcept = delete;
    ~ServoDevice() override;

    ServoDevice &operator=(const ServoDevice &) = delete;
    ServoDevice &operator=(ServoDevice &&) noexcept = delete;

protected:
    void writeImpl(const double value) override;
};

#endif