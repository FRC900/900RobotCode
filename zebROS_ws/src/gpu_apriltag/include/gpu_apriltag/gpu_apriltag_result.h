#ifndef GPU_APRILTAG_RESULT_INC__
#define GPU_APRILTAG_RESULT_INC__

#include <opencv2/core.hpp>
// Not available on Jetson version of opencv #include <opencv2/core/quaternion.hpp>

struct GpuApriltagResult
{
    // cv::Point3d position_;
    // cv::Quatd orientation_;
    int id_;
    int hamming_;
    double pose_error_;
    double distortion_factor_;
    double pose_error_ratio_;
    cv::Point2d center_;
    std::array<cv::Point2d, 4> original_corners_;
    std::array<cv::Point2d, 4> undistorted_corners_;
};

#endif