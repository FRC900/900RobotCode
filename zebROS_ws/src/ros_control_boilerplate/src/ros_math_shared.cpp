
#include <ros/console.h>
#include "ros_control_boilerplate/ros_math_shared.hpp"
#include <hal/FRCUsageReporting.h>

void ROSMathShared::ReportErrorV(fmt::string_view format, fmt::format_args args) {
    ROS_ERROR_STREAM(__FILE__ << " " << __LINE__<< " " << __FUNCTION__ << " " << fmt::vformat(format, args));
}

void ROSMathShared::ReportWarningV(fmt::string_view format, fmt::format_args args) {
    ROS_WARN_STREAM(__FILE__ << " " << __LINE__<< " " << __FUNCTION__ << " " << fmt::vformat(format, args));
}

void ROSMathShared::ReportUsage(wpi::math::MathUsageId id, int count)
{
    switch (id)
    {
    case wpi::math::MathUsageId::kKinematics_DifferentialDrive:
        HAL_Report(HALUsageReporting::kResourceType_Kinematics,
                   HALUsageReporting::kKinematics_DifferentialDrive);
        break;
    case wpi::math::MathUsageId::kKinematics_MecanumDrive:
        HAL_Report(HALUsageReporting::kResourceType_Kinematics,
                   HALUsageReporting::kKinematics_MecanumDrive);
        break;
    case wpi::math::MathUsageId::kKinematics_SwerveDrive:
        HAL_Report(HALUsageReporting::kResourceType_Kinematics,
                   HALUsageReporting::kKinematics_SwerveDrive);
        break;
    case wpi::math::MathUsageId::kTrajectory_TrapezoidProfile:
        HAL_Report(HALUsageReporting::kResourceType_TrapezoidProfile, count);
        break;
    case wpi::math::MathUsageId::kFilter_Linear:
        HAL_Report(HALUsageReporting::kResourceType_LinearFilter, count);
        break;
    case wpi::math::MathUsageId::kOdometry_DifferentialDrive:
        HAL_Report(HALUsageReporting::kResourceType_Odometry,
                   HALUsageReporting::kOdometry_DifferentialDrive);
        break;
    case wpi::math::MathUsageId::kOdometry_SwerveDrive:
        HAL_Report(HALUsageReporting::kResourceType_Odometry,
                   HALUsageReporting::kOdometry_SwerveDrive);
        break;
    case wpi::math::MathUsageId::kOdometry_MecanumDrive:
        HAL_Report(HALUsageReporting::kResourceType_Odometry,
                   HALUsageReporting::kOdometry_MecanumDrive);
        break;
    case wpi::math::MathUsageId::kController_PIDController2:
        HAL_Report(HALUsageReporting::kResourceType_PIDController2, count);
        break;
    case wpi::math::MathUsageId::kController_ProfiledPIDController:
        HAL_Report(HALUsageReporting::kResourceType_ProfiledPIDController,
                   count);
        break;
    }
}