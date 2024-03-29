#ifndef GPU_APRILTAG_RESULT_INC__
#define GPU_APRILTAG_RESULT_INC__

#include <opencv2/core.hpp>
#include <opencv2/core/quaternion.hpp>

struct GpuApriltagResult
{
    cv::Point3f position_;
    cv::Quatf orientation_;
    double pose_error_;
    double distortion_factor_;
    double pose_error_ratio_;
    std::array<cv::Point2f, 4> original_corners_;
    std::array<cv::Point2f, 4> undistorted_corners_;
};

#endif