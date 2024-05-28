#ifndef GPU_APRILTAG_INC__
#define GPU_APRILTAG_INC__

#include "frc971/orin/apriltag_input_format.h"
#include "sensor_msgs/CameraInfo.h"
#include "gpu_apriltag/gpu_apriltag_result.h"

namespace frc971_gpu_apriltag
{
template <frc971::apriltag::InputFormat INPUT_FORMAT>
class FRC971GpuApriltagDetectorImpl;

// Create a pure virtual base class that templated classes can inherit from
class FRC971GpuApriltagDetectorBase
{
public:
    FRC971GpuApriltagDetectorBase() = default;
    virtual ~FRC971GpuApriltagDetectorBase() = default;
    virtual void Detect(std::vector<GpuApriltagResult> &results,
                         std::vector<std::array<cv::Point2d, 4>> &rejected_margin_corners,
                         std::vector<std::array<cv::Point2d, 4>> &rejected_noconverge_corners,
                         const cv::Mat &color_image) = 0;
};

template <frc971::apriltag::InputFormat INPUT_FORMAT>
class FRC971GpuApriltagDetector : public FRC971GpuApriltagDetectorBase
{
public:
    FRC971GpuApriltagDetector(const sensor_msgs::CameraInfo::ConstPtr &camera_info);
    ~FRC971GpuApriltagDetector() override;
    void Detect(std::vector<GpuApriltagResult> &results,
                std::vector<std::array<cv::Point2d, 4>> &rejected_margin_corners,
                std::vector<std::array<cv::Point2d, 4>> &rejected_noconverge_corners,
                const cv::Mat &color_image) override; 

private:
    std::unique_ptr<FRC971GpuApriltagDetectorImpl<INPUT_FORMAT>> impl_;
};
}

#endif