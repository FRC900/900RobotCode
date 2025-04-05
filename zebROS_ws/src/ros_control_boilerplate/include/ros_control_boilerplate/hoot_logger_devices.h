#ifndef HOOT_LOGGER_DEVICES_H_INC__
#define HOOT_LOGGER_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"
#include "std_srvs/SetBool.h"
#include "ros_control_boilerplate/SignalLoggerString.h"

class HootLoggerDevices : public Devices
{

public:
    explicit HootLoggerDevices(ros::NodeHandle root_nh);
    HootLoggerDevices(const HootLoggerDevices &) = delete;
    HootLoggerDevices(HootLoggerDevices &&) noexcept = delete;
    ~HootLoggerDevices() override;

    HootLoggerDevices &operator=(const HootLoggerDevices &) = delete;
    HootLoggerDevices &operator=(HootLoggerDevices &&) noexcept = delete;

    void write(const ros::Time& time, const ros::Duration& period, Tracer &tracer) override;

private:
    // Default to true so that a transition from true to false
    // happens on code startup.
    bool prev_robot_enabled_{true};

    // Allos external code to force the SignalLogger to start or stop
    ros::ServiceServer force_enable_service_;
    bool forceEnableSrv(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    // Write an arbitrary string to the SignalLogger
    ros::ServiceServer write_string_service_;
    bool writeStringSrv(ros_control_boilerplate::SignalLoggerString::Request &req, ros_control_boilerplate::SignalLoggerString::Response &res);
};

#endif