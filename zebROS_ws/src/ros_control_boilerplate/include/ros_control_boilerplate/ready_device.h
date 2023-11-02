#ifndef READY_DEVICE_INC__
#define READY_DEVICE_INC__

#include <string>

namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class RemoteJointInterface;
}

class ReadyDevice
{
public:
    ReadyDevice(const int joint_index,
                const std::string &joint_name,
                const bool local);
    ReadyDevice(const ReadyDevice &) = delete;
    ReadyDevice(ReadyDevice &&other) noexcept = delete;
    virtual ~ReadyDevice();

    ReadyDevice &operator=(const ReadyDevice &) = delete;
    ReadyDevice &operator=(ReadyDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::JointStateInterface &state_interface,
                            hardware_interface::JointCommandInterface &command_interface,
                            hardware_interface::RemoteJointInterface &remote_joint_interface);
    bool isReady(void) const;

private:
    const std::string name_;
    const bool local_;
    double state_;
};

#endif