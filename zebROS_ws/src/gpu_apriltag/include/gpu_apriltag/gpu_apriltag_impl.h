#ifndef GPU_APRILTAG_IMPL_INC__
#define GPU_APRILTAG_IMPL_INC__
#include <vector>
#include "gpu_apriltag/gpu_apriltag_result.h"

#include <sensor_msgs/CameraInfo.h>
#include "frc971/orin/apriltag.h"
#include "frc971/orin/apriltag_input_format.h"
#include "third_party/apriltag/apriltag_pose.h"
#include "third_party/apriltag/apriltag.h"
namespace frc971_gpu_apriltag
{
class FRC971GpuApriltagDetectorImpl
{
public:
    FRC971GpuApriltagDetectorImpl(const sensor_msgs::CameraInfo::ConstPtr &camera_info,
                                  const frc971::apriltag::InputFormat input_format);
    virtual ~FRC971GpuApriltagDetectorImpl();
    void Detect(std::vector<GpuApriltagResult> &results,
                std::vector<std::array<cv::Point2d, 4>> &rejected_margin_corners,
                std::vector<std::array<cv::Point2d, 4>> &rejected_noconverge_corners,
                const cv::Mat &color_image);

private:
    bool UndistortDetection(apriltag_detection_t *det) const;
    double ComputeDistortionFactor(const std::array<cv::Point2d, 4> &orig_corners,
                                   const std::array<cv::Point2d, 4> &corners) const;

    apriltag_family_t *tag_family_;
    apriltag_detector_t *tag_detector_;

    cv::Mat intrinsics_;
    frc971::apriltag::CameraMatrix distortion_camera_matrix_;
    frc971::apriltag::DistCoeffs distortion_coefficients_;

    frc971::apriltag::GpuDetector gpu_detector_;
    cv::Size image_size_;

    size_t rejections_{0};

    double max_expected_distortion_{0.314}; // TODO - make this a config item
    double min_decision_margin_{50.};
    int32_t pixel_border_{5};
    uint64_t pose_estimation_iterations_{50};
    double tag_size_ = 0.1651; // 6.5 inches
};
}
#endif
