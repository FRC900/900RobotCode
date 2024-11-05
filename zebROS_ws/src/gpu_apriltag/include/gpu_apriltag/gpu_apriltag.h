#ifndef GPU_APRILTAG_INC__
#define GPU_APRILTAG_INC__

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif

#include <array>
#include "frc971/orin/apriltag_input_format.h"
#include "sensor_msgs/CameraInfo.h"
#include "gpu_apriltag/gpu_apriltag_result.h"

namespace frc971_gpu_apriltag
{
class FRC971GpuApriltagDetectorImpl;

class FRC971GpuApriltagDetector
{
public:
    FRC971GpuApriltagDetector(const sensor_msgs::CameraInfo::ConstPtr &camera_info,
                              frc971::apriltag::InputFormat input_format);
    ~FRC971GpuApriltagDetector();
    void Detect(std::vector<GpuApriltagResult> &results,
                std::vector<std::array<cv::Point2d, 4>> &rejected_margin_corners,
                std::vector<std::array<cv::Point2d, 4>> &rejected_noconverge_corners,
                const cv::Mat &color_image); 

private:
    std::unique_ptr<FRC971GpuApriltagDetectorImpl> impl_;
};
}

#endif