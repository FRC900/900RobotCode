#include <cmath>
#include <ros/console.h>

#include "ros_control_boilerplate/get_conversion_factor.h"

double getConversionFactor(const int encoder_ticks_per_rotation,
                           const hardware_interface::FeedbackDevice encoder_feedback,
                           const hardware_interface::TalonMode talon_mode)
{
    if ((talon_mode == hardware_interface::TalonMode_Position) ||
        (talon_mode == hardware_interface::TalonMode_MotionMagic)) // TODO - maybe motion profile as well?
    {
        switch (encoder_feedback)
        {
        case hardware_interface::FeedbackDevice_Uninitialized:
            return 1.;
        case hardware_interface::FeedbackDevice_QuadEncoder:
        case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
            return 2. * M_PI / encoder_ticks_per_rotation;
        case hardware_interface::FeedbackDevice_Analog:
            return 2. * M_PI / 1024.;
        case hardware_interface::FeedbackDevice_Tachometer:
        case hardware_interface::FeedbackDevice_SensorSum:
        case hardware_interface::FeedbackDevice_SensorDifference:
        case hardware_interface::FeedbackDevice_RemoteSensor0:
        case hardware_interface::FeedbackDevice_RemoteSensor1:
        case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
            return 2. * M_PI / encoder_ticks_per_rotation;
        default:
            ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
            return 1.;
        }
    }
    else if (talon_mode == hardware_interface::TalonMode_Velocity)
    {
        switch (encoder_feedback)
        {
        case hardware_interface::FeedbackDevice_Uninitialized:
            return .1;
        case hardware_interface::FeedbackDevice_QuadEncoder:
        case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
            return 2. * M_PI / encoder_ticks_per_rotation / .1;
        case hardware_interface::FeedbackDevice_Analog:
            return 2. * M_PI / 1024. / .1;
        case hardware_interface::FeedbackDevice_Tachometer:
        case hardware_interface::FeedbackDevice_SensorSum:
        case hardware_interface::FeedbackDevice_SensorDifference:
        case hardware_interface::FeedbackDevice_RemoteSensor0:
        case hardware_interface::FeedbackDevice_RemoteSensor1:
        case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
            return 2. * M_PI / encoder_ticks_per_rotation / .1;
        default:
            ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
            return .1;
        }
    }
    else
    {
        // ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
        return 1.;
    }
}