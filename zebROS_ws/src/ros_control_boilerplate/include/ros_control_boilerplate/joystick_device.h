#ifndef JOYSTICK_DEVICE_INC__
#define JOYSTICK_DEVICE_INC__

#include <string>

namespace frc { class Joystick; }
namespace hardware_interface
{
    class JoystickState;
    class JoystickStateInterface;
};
class PeriodicIntervalCounter;

class JoystickDevice
{
public:
    JoystickDevice(const int joint_index,
                   const std::string &name,
                   const uint8_t id,
                   const double read_hz);
    JoystickDevice(const JoystickDevice &) = delete;
    JoystickDevice(JoystickDevice &&other) noexcept = delete;
    virtual ~JoystickDevice();

    JoystickDevice &operator=(const JoystickDevice &) = delete;
    JoystickDevice &operator=(JoystickDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JoystickStateInterface &state_interface) const;
    virtual void simInit(ros::NodeHandle /*nh*/, size_t /*joint_index*/) {}
    virtual void read(const ros::Time& /*time*/, const ros::Duration& period);

protected:
    int getId(void) const { return id_; }

private:
    const std::string name_;
    const uint8_t     id_;
    std::unique_ptr<frc::Joystick> joystick_;
    std::unique_ptr<hardware_interface::JoystickState> state_;
    std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
};

#endif