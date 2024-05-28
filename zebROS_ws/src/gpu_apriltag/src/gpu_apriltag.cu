#include "gpu_apriltag/gpu_apriltag.h"
#include "gpu_apriltag/gpu_apriltag_impl.h"

namespace frc971_gpu_apriltag
{
    template <frc971::apriltag::InputFormat INPUT_FORMAT>
    FRC971GpuApriltagDetector<INPUT_FORMAT>::FRC971GpuApriltagDetector(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
        : impl_{std::make_unique<FRC971GpuApriltagDetectorImpl<INPUT_FORMAT>>(camera_info)}
    {
    }
    template <frc971::apriltag::InputFormat INPUT_FORMAT>
    FRC971GpuApriltagDetector<INPUT_FORMAT>::~FRC971GpuApriltagDetector() = default;

    template <frc971::apriltag::InputFormat INPUT_FORMAT>
    void FRC971GpuApriltagDetector<INPUT_FORMAT>::Detect(std::vector<GpuApriltagResult> &results,
                                                              std::vector<std::array<cv::Point2d, 4>> &rejected_margin_corners,
                                                              std::vector<std::array<cv::Point2d, 4>> &rejected_noconverge_corners,
                                                              const cv::Mat &color_image)
    {
        impl_->Detect(results, rejected_margin_corners, rejected_noconverge_corners, color_image);
    }

    template class FRC971GpuApriltagDetector<frc971::apriltag::InputFormat::Mono8>;
    template class FRC971GpuApriltagDetector<frc971::apriltag::InputFormat::BGR8>;
}