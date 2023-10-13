#ifndef ROS_ITERATIVE_ROBOT_DEVICES_H_INC__
#define ROS_ITERATIVE_ROBOT_DEVICES_H_INC__

#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/devices.h"

namespace ros_control_boilerplate
{
    class ROSIterativeRobot;
}
class PeriodicIntervalCounter;

class ROSIterativeRobotDevices : public Devices
{
public:
    ROSIterativeRobotDevices(ros::NodeHandle &root_nh);
    ROSIterativeRobotDevices(const ROSIterativeRobotDevices &) = delete;
    ROSIterativeRobotDevices(ROSIterativeRobotDevices &&) noexcept = delete;
    virtual ~ROSIterativeRobotDevices();

    ROSIterativeRobotDevices &operator=(const ROSIterativeRobotDevices &) = delete;
    ROSIterativeRobotDevices &operator=(ROSIterativeRobotDevices &&) noexcept = delete;

    hardware_interface::InterfaceManager *registerInterface() override;
    // Not really a read, just a periodic signalling to the DS via wpilib that robot code is alive
    void read(const ros::Time &time, const ros::Duration &period, Tracer &tracer) override;

private:
    std::unique_ptr<ros_control_boilerplate::ROSIterativeRobot> ros_iterative_robot_;
    std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
    hardware_interface::InterfaceManager interface_manager_;

    bool started_competition_{false};
};

#endif