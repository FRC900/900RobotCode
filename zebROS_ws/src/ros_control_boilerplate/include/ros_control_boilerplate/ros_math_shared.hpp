#ifndef ROS_MATH_SHARED_INC__
#define ROS_MATH_SHARED_INC__
#include "wpimath/MathShared.h"

class ROSMathShared : public wpi::math::MathShared {
 public:
  void ReportErrorV(fmt::string_view format, fmt::format_args args) override;
  void ReportWarningV(fmt::string_view format, fmt::format_args args) override;
  void ReportUsage(wpi::math::MathUsageId id, int count) override;
};

#endif