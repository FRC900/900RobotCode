#ifndef PCM_DEVICES_H_INC__
#define PCM_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class PCMDevice;
namespace hardware_interface
{
    class JointStateInterface;
    class JointCommandInterface;
    class PositionJointInterface;
    class RemoteJointInterface;
    class PCMStateInterface;
    class RemotePCMStateInterface;
}

class PCMDevices : public Devices
{
public:
    explicit PCMDevices(ros::NodeHandle &root_nh);
    PCMDevices(const PCMDevices &) = delete;
    PCMDevices(PCMDevices &&) noexcept = delete;
    ~PCMDevices() override;

    PCMDevices &operator=(const PCMDevices &) = delete;
    PCMDevices &operator=(PCMDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    double read_hz_{20};
    std::vector<std::unique_ptr<PCMDevice>> devices_;
    std::unique_ptr<hardware_interface::JointStateInterface> state_interface_;
    std::unique_ptr<hardware_interface::JointCommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::PositionJointInterface> position_joint_interface_;
    std::unique_ptr<hardware_interface::RemoteJointInterface> remote_joint_interface_;
    std::unique_ptr<hardware_interface::PCMStateInterface> pcm_state_interface_;
    std::unique_ptr<hardware_interface::RemotePCMStateInterface> remote_pcm_state_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif