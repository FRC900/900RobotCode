#include "ros_control_boilerplate/hoot_logger_devices.h"

#include "ctre/phoenix6/SignalLogger.hpp"

static void safeCall(ctre::phoenix::StatusCode status_code, const std::string &method_name)
{
    if (!status_code.IsOK())
    {
        ROS_ERROR_STREAM("Error : calling " << method_name << " : " << status_code.GetName());
    }
}

HootLoggerDevices::HootLoggerDevices(ros::NodeHandle root_nh)
    : force_enable_service_{root_nh.advertiseService("force_enable", &HootLoggerDevices::forceEnableSrv, this)}
    , write_string_service_{root_nh.advertiseService("write_string", &HootLoggerDevices::writeStringSrv, this)}
{
    // Disable auto logging if not on HAL robot
    // This is to prevent the SignalLogger from running on a non-HAL robot
    // and causing issues with the hardware interface.
    if (!isHALRobot())
    {
        safeCall(ctre::phoenix6::SignalLogger::EnableAutoLogging(false), "ctre::phoenix6::SignalLogger::EnableAutoLogging(false)");
    }
}

HootLoggerDevices::~HootLoggerDevices() = default;

void HootLoggerDevices::write(const ros::Time &/*time*/, const ros::Duration &/*period*/, Tracer &tracer)
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

bool HootLoggerDevices::forceEnableSrv(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if (req.data)
    {
        ROS_INFO_STREAM("Forcing SignalLogger to start");
        safeCall(ctre::phoenix6::SignalLogger::Start(), "ctre::phoenix6::SignalLogger::Start()");
    }
    else
    {
        ROS_INFO_STREAM("Forcing SignalLogger to stop");
        safeCall(ctre::phoenix6::SignalLogger::Stop(), "ctre::phoenix6::SignalLogger::Stop()");
    }
    res.success = true;
    return true;
}

bool HootLoggerDevices::writeStringSrv(ros_control_boilerplate::SignalLoggerString::Request &req, ros_control_boilerplate::SignalLoggerString::Response &res)
{
    if (req.name.empty() || req.value.empty())
    {
        ROS_ERROR_STREAM("Empty name or data passed to writeStringSrv");
        res.success = false;
        return false;
    }
    ROS_INFO_STREAM("Writing string to SignalLogger: " << req.name << " : " << req.value);
    safeCall(ctre::phoenix6::SignalLogger::WriteString(req.name, req.value), "ctre::phoenix6::SignalLogger::WriteString");
    res.success = true;
    return true;
}