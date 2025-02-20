#include "frc/PWM.h"
#include "ros/console.h"
#include "ros_control_boilerplate/pwm_device.h"
#include "ros_control_boilerplate/servo_device.h"

ServoDevice::ServoDevice(const int joint_index,
                         const std::string &joint_name,
                         const int servo_channel,
                         const double output_max,
                         const double output_min,
                         const int period_multiplier,
                         const bool invert,
                         const bool local_hardware,
                         const bool local_update)
    : PWMDevice(joint_index,
                joint_name,
                servo_channel,
                output_max,
                0,
                0,
                output_min,
                0,
                period_multiplier,
                invert,
                local_hardware,
                local_update,
                false)
{
    ROS_INFO_STREAM("Loading joint " << joint_index << "=" <<  joint_name <<
                    (local_update ? " local" : " remote") << " update, " <<
                    (local_hardware ? "local" : "remote") << " hardware" <<
                    " as Servo " << servo_channel <<
                    " invert " << invert);
}

ServoDevice::~ServoDevice() = default;

void ServoDevice::writeImpl(const double value)
{
    pwm_->SetPosition(value);
}