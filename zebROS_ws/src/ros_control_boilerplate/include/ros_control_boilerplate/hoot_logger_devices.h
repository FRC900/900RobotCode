#ifndef HOOT_LOGGER_DEVICES_H_INC__
#define HOOT_LOGGER_DEVICES_H_INC__

#include "ros_control_boilerplate/devices.h"

class HootLoggerDevices : public Devices
{

public:
    explicit HootLoggerDevices(const ros::NodeHandle &root_nh);
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
};

#endif