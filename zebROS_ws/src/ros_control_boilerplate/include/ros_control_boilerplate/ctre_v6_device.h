// Base class for all devices using the CTREv6 API
// Provides utility functions for error checking as well
// as a function used to hook devices up to other CTREv6
// devices (for example, setting up a Falcon500 as an 
// orchestra instrument)
#ifndef CTRE_V6_DEVICE_INC__
#define CTRE_V6_DEVICE_INC__

#include <atomic>
#include <optional>
#include <string>

#include "ros/console.h"
#include "hal/DriverStation.h"

#include "ctre/phoenix6/StatusSignal.hpp"
namespace ctre::phoenix6::hardware
{
    class ParentDevice;
}

class CTREV6Device
{
public:
    CTREV6Device(const std::string &device_type,
                 const std::string &joint_name,
                 const int id);
    CTREV6Device(const CTREV6Device &) = delete;
    CTREV6Device(CTREV6Device &&other) noexcept = delete;
    virtual ~CTREV6Device();

    CTREV6Device &operator=(const CTREV6Device &) = delete;
    CTREV6Device &operator=(CTREV6Device &&) noexcept = delete;

    const std::string& getName(void) const;
    ctre::phoenix6::hardware::ParentDevice *getParentDevice(void) const;

protected:
    bool safeCall(ctre::phoenix::StatusCode status_code, const std::string &method_name) const;
    template <class T>
    std::optional<T> safeRead(const ctre::phoenix6::StatusSignal<T> &status_signal, const std::string &method_name) const
    {
        //status_signal.Refresh();
        if (status_signal.GetStatus() == ctre::phoenix::StatusCode::OK)
        {
            can_error_count_ = 0;
            can_error_sent_ = false;
            return status_signal.GetValue();
        }
        ROS_ERROR_STREAM("Error : " << device_type_ << " " << name_ << " id = " << id_ << " calling " << method_name << " : " << status_signal.GetStatus().GetName() << " (" << status_signal.GetStatus() << ")");
        can_error_count_++;
        if ((can_error_count_ > 1000) && !can_error_sent_)
        {
            HAL_SendError(true, -1, false, (device_type_ + " safeCall - too many CAN bus errors!").c_str(), "", "", true);
            can_error_sent_ = true;
        }
        return std::nullopt;
    }

    int getId(void) const;

    void setParentDevice(ctre::phoenix6::hardware::ParentDevice *parent_device);

private:
    const std::string device_type_;
    const std::string name_;
    const int id_;
    ctre::phoenix6::hardware::ParentDevice *parent_device_{nullptr};
    static inline std::atomic<size_t> can_error_count_{0};
    static inline std::atomic<bool> can_error_sent_{false};
};

#endif
