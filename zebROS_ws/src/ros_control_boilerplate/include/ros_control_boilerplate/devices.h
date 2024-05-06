// Base class for all devices classes.  This provides a common interface
// to the hardware interface as well as defaults for required functions.
// Derived classes implementing code specific to a type of device
// will derive from the class and override those functions needed
// to provide support for the device.
// A typical devices class will store a vector holding a device class.
// The devices code will parse joint list param and create device instances for
// all of the defined device of that type.
// The read and write functions (and sim and hw variants) are responsible
// for iterating through each device and calling the appropriate functions
// to set the device state from hardware / write the command buffer out to hardware.
// Each device class instance will hold a buffer for device state and commands,
// as well as the interface glue code needed for controllers to access them.
// It'll also have the details of communicating with hardware, typically
// by creating and storing a device object for that type.
// Individual device types might add more as needed.
#ifndef INC_DEVICES_H__
#define INC_DEVICES_H__

#include <ros/ros.h>
#include "ros_control_boilerplate/tracer.h"

namespace hardware_interface
{
    class InterfaceManager;
}

namespace gazebo::physics
{
    class Model;
}

class Devices
{
public:
    Devices() = default;
    Devices(const Devices &) = delete;
    Devices(Devices &&) noexcept = delete;
    virtual ~Devices() = default;

    Devices& operator= (const Devices &) = delete;
    Devices& operator= (Devices &&) noexcept = delete;

    virtual hardware_interface::InterfaceManager *registerInterface() {return nullptr;}
    // Read and write functions that are shared between hardware and sim interfaces
    virtual void read(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void write(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}

    // Read and write functions which add additional sim features
    virtual void simInit(ros::NodeHandle &/*nh*/) {}
    virtual void simPreRead(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void simPostRead(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void simWrite(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}

    // Read and write functions which add additional sim features
    virtual bool gazeboSimInit(const ros::NodeHandle& /*nh*/, boost::shared_ptr<gazebo::physics::Model> parent_model) { return true;}
    virtual void gazeboSimRead(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void gazeboSimWrite(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/, const bool e_stop_active_) {}

    // Read and write functions which add additional hardware-only features
    virtual void hwInit(ros::NodeHandle &/*nh*/) {}
    virtual void hwRead(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}
    virtual void hwWrite(const ros::Time& /*time*/, const ros::Duration& /*period*/, Tracer& /*tracer*/) {}

    // Used to signal that all controllers are finished loading
    static void signalReady(void) { ready_ = true; }

    // Updates the robot's enable/disable state
    static void setEnabled(const bool enabled) { enabled_ = enabled; }
    static void setHALRobot(const bool hal_robot) { hal_robot_ = hal_robot; }

protected:
    static bool isEnabled(void)  { return enabled_; }
    static bool isHALRobot(void) { return hal_robot_; }
    static bool isReady(void)    { return ready_; }

private:
    static inline bool enabled_{true};
    static inline bool hal_robot_{true};
    static inline bool ready_{false};
};

template<class T>
T* getDevicesOfType(const std::vector<std::unique_ptr<Devices>> &device_list)
{
    for (auto &d: device_list)
    {
        auto di = dynamic_cast<T*>(d.get());
        if (di)
        {
            return di;
        }
    }
    return nullptr;
}

#endif