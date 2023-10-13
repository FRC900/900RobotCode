#ifndef SIM_MATCH_DATA_DEVICE_INC__
#define SIM_MATCH_DATA_DEVICE_INC__
// This class is derived from the HW joystick code and adds a subscriber
// which reads from a sensor_msgs/Joy message and writes the values from
// there to the hal joystick sim 

// This is only needed because the version of roscpp on the Jetson doesn't
// work with C++20, and we need C++20 to build wpilib code used in the HW 
// interface.  So split this out and only build it on x86, which works because 
// the version of ROS we build for Ubuntu 22.04 is patched to work correctly
// with C++20
#include <string>

#include <ros/node_handle.h>
#include "frc_msgs/MatchSpecificData.h"
#include "ros_control_boilerplate/match_data_device.h"

class SimMatchDataDevice : public MatchDataDevice
{
public:
    SimMatchDataDevice(ros::NodeHandle &nh);
    SimMatchDataDevice(const SimMatchDataDevice &) = delete;
    SimMatchDataDevice(SimMatchDataDevice &&other) noexcept = delete;
    ~SimMatchDataDevice();

    SimMatchDataDevice &operator=(const SimMatchDataDevice &) = delete;
    SimMatchDataDevice &operator=(SimMatchDataDevice &&) noexcept = delete;

    void read(const ros::Time& time, const ros::Duration& period);

    void simInit(ros::NodeHandle nh);

    std::optional<bool> isEnabled(void) const;
    bool getControlWord(HAL_ControlWord &cw) const;

private:
    mutable std::unique_ptr<std::mutex> mutex_;

    void matchDataCallback(const frc_msgs::MatchSpecificData &match_data);
    ros::Subscriber sim_sub_;
};

#endif