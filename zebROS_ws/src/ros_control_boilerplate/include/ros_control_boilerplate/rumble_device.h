#ifndef Rumble_DEVICE_INC__
#define Rumble_DEVICE_INC__

#include <memory>
#include <string>

namespace hardware_interface
{
    class RumbleHWState;
    class RumbleHWCommand;
    class RumbleStateInterface;
    class RumbleCommandInterface;
    class RemoteRumbleStateInterface;
}

class RumbleDevice
{
public:
    RumbleDevice(const int joint_index,
                 const std::string &joint_name,
                 const int port,
                 const bool local_hardware,
                 const bool local_update);
    RumbleDevice(const RumbleDevice &) = delete;
    RumbleDevice(RumbleDevice &&other) noexcept = delete;
    ~RumbleDevice();

    RumbleDevice &operator=(const RumbleDevice &) = delete;
    RumbleDevice &operator=(RumbleDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::RumbleStateInterface &state_interface,
                            hardware_interface::RumbleCommandInterface &command_interface,
                            hardware_interface::RemoteRumbleStateInterface &remote_state_interface);
    void write(const ros::Time &time, const ros::Duration &period);

private:
    const std::string name_;
    const int port_;
    const bool local_hardware_;
    const bool local_update_;

    std::unique_ptr<hardware_interface::RumbleHWState> state_;
    std::unique_ptr<hardware_interface::RumbleHWCommand> command_;
};

#endif