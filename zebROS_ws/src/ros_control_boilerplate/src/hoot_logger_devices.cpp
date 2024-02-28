#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "ros_control_boilerplate/hoot_logger_devices.h"

#include "ctre/phoenix6/SignalLogger.hpp"

static void safeCall(ctre::phoenix::StatusCode status_code, const std::string &method_name)
{
    if (!status_code.IsOK())
    {
        ROS_ERROR_STREAM("Error : calling " << method_name << " : " << status_code.GetName());
    }
}

HootLoggerDevices::HootLoggerDevices(const ros::NodeHandle &root_nh)
{
    if (!isHALRobot())
    {
        safeCall(ctre::phoenix6::SignalLogger::EnableAutoLogging(false), "ctre::phoenix6::SignalLogger::EnableAutoLogging(false)");
    }
}

HootLoggerDevices::~HootLoggerDevices() = default;

void HootLoggerDevices::write(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("hoot logger");
    if (!isHALRobot())
    {
        return;
    }
    const auto enabled = isEnabled();
    if (prev_robot_enabled_ != enabled)
    {
        if (prev_robot_enabled_) // switching from enabled to disabled
        {
            ROS_INFO_STREAM("Stopping SignalLogger on disable");
            safeCall(ctre::phoenix6::SignalLogger::Stop(), "ctre::phoenix6::SignalLogger::Stop()");
        }
        else // switching from disabled to enabled
        {
            ROS_INFO_STREAM("Starting SignalLogger on enable");
            safeCall(ctre::phoenix6::SignalLogger::Start(), "ctre::phoenix6::SignalLogger::Start()");
        }
    }
    prev_robot_enabled_ = enabled;
}