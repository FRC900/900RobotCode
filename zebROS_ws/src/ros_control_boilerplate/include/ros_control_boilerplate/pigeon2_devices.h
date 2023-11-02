#ifndef PIGEON2_DEVICES_H_INC__
#define PIGEON2_DEVICES_H_INC__
#include <map>
#include "ros_control_boilerplate/devices.h"

class Pigeon2Device;
namespace hardware_interface
{
    class ImuSensorInterface;
    class RemoteImuSensorInterface;
    namespace pigeon2
    {
        class Pigeon2StateInterface;
        class Pigeon2CommandInterface;
        class RemotePigeon2StateInterface;
    }
}

namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

class Pigeon2Devices : public Devices
{
public:
    explicit Pigeon2Devices(ros::NodeHandle &root_nh);
    Pigeon2Devices(const Pigeon2Devices &) = delete;
    Pigeon2Devices(Pigeon2Devices &&) noexcept = delete;
    ~Pigeon2Devices() override;

    Pigeon2Devices &operator=(const Pigeon2Devices &) = delete;
    Pigeon2Devices &operator=(Pigeon2Devices &&) noexcept = delete;
    
    hardware_interface::InterfaceManager *registerInterface() override;
    void read(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;
    void simInit(ros::NodeHandle &nh) override;
    void simRead(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

    void appendDeviceMap(std::multimap<std::string, ctre::phoenix6::hardware::ParentDevice *> &device_map) const;

private:
    double read_hz_{100};
    std::vector<std::unique_ptr<Pigeon2Device>> devices_;
    std::unique_ptr<hardware_interface::pigeon2::Pigeon2StateInterface> state_interface_;
    std::unique_ptr<hardware_interface::pigeon2::Pigeon2CommandInterface> command_interface_;
    std::unique_ptr<hardware_interface::pigeon2::RemotePigeon2StateInterface> remote_state_interface_;
    std::unique_ptr<hardware_interface::ImuSensorInterface> imu_interface_;
    std::unique_ptr<hardware_interface::RemoteImuSensorInterface> imu_remote_interface_;
    hardware_interface::InterfaceManager interface_manager_;
};

#endif