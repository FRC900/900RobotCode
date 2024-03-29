#include "gpu_apriltag/gpu_apriltag.h"
#include "gpu_apriltag/gpu_apriltag_impl.h"

namespace frc971_gpu_apriltag
{
    FRC971GpuApriltagDetector::FRC971GpuApriltagDetector(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
        : impl_{std::make_unique<FRC971GpuApriltagDetectorImpl>(camera_info)}
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
}