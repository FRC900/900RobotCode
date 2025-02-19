#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif

#include <array>
#include <vector>
#include "gpu_apriltag/gpu_apriltag.h"
#include "gpu_apriltag/gpu_apriltag_impl.h"

namespace frc971_gpu_apriltag
{
    FRC971GpuApriltagDetector::FRC971GpuApriltagDetector(const sensor_msgs::CameraInfo::ConstPtr &camera_info, frc971::apriltag::InputFormat input_format, const int min_white_black_diff)
        : impl_{std::make_unique<FRC971GpuApriltagDetectorImpl>(camera_info, input_format, min_white_black_diff)}
    {
    }
    FRC971GpuApriltagDetector::~FRC971GpuApriltagDetector() = default;

    void FRC971GpuApriltagDetector::Detect(std::vector<GpuApriltagResult> &results,
                                           std::vector<std::array<cv::Point2d, 4>> &rejected_margin_corners,
                                           std::vector<std::array<cv::Point2d, 4>> &rejected_noconverge_corners,
                                           const cv::Mat &color_image)
    {
        impl_->Detect(results, rejected_margin_corners, rejected_noconverge_corners, color_image);
    }
} // namespace
