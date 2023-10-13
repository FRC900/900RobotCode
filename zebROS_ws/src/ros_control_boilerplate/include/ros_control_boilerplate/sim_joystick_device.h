#ifndef SIM_JOYSTICK_DEVICE_INC__
#define SIM_JOYSTICK_DEVICE_INC__

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
#include <ros_control_boilerplate/joystick_device.h>
#include <sensor_msgs/Joy.h>

class SimJoystickDevice : public JoystickDevice
{
public:
    SimJoystickDevice(const int joint_index,
                   const std::string &name,
                   const int id,
                   const double read_hz);
    SimJoystickDevice(const SimJoystickDevice &) = delete;
    SimJoystickDevice(SimJoystickDevice &&other) noexcept = delete;
    virtual ~SimJoystickDevice();

    SimJoystickDevice &operator=(const SimJoystickDevice &) = delete;
    SimJoystickDevice &operator=(SimJoystickDevice &&) noexcept = delete;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void simInit(ros::NodeHandle nh, size_t joint_index) override;

private:
    mutable std::unique_ptr<std::mutex> mutex_;
    ros::Subscriber sim_sub_;
    void joystickCallback(const sensor_msgs::JoyConstPtr &msg);
};

#endif