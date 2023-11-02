#ifndef ROBOT_CONTROLLER_DEVICE_INC__
#define ROBOT_CONTROLLER_DEVICE_INC__

#include <string>

#include <ros/node_handle.h>

namespace frc { class RobotController; }
namespace hardware_interface
{
    class RobotControllerState;
    class RobotControllerStateInterface;
}
class PeriodicIntervalCounter;

class RobotControllerDevice
{
public:
    explicit RobotControllerDevice(ros::NodeHandle &nh);
    RobotControllerDevice(const RobotControllerDevice &) = delete;
    RobotControllerDevice(RobotControllerDevice &&other) noexcept = delete;
    virtual ~RobotControllerDevice();

    RobotControllerDevice &operator=(const RobotControllerDevice &) = delete;
    RobotControllerDevice &operator=(RobotControllerDevice &&) noexcept = delete;

    void registerInterfaces(hardware_interface::RobotControllerStateInterface &state_interface) const;
    void read(const ros::Time& time, const ros::Duration& period);

private:
    std::unique_ptr<hardware_interface::RobotControllerState> state_;
    std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
};

#endif