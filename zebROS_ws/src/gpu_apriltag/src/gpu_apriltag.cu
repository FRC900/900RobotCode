#include <opencv2/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include "frc971/orin/apriltag.h"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/apriltag_pose.h"
#include "third_party/apriltag/tag16h5.h"
#include "third_party/apriltag/tag36h11.h"

namespace frc971_gpu_apriltag
{
frc971::apriltag::CameraMatrix getCameraMatrix(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
      return frc971::apriltag::CameraMatrix{
      .fx = camera_info->K[0],
      .cx = camera_info->K[2],
      .fy = camera_info->K[4],
      .cy = camera_info->K[5]
  };
}

frc971::apriltag::DistCoeffs getDistCoeffs(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
      return frc971::apriltag::DistCoeffs{
      .k1 = camera_info->D[0],
      .k2 = camera_info->D[1],
      .p1 = camera_info->D[2],
      .p2 = camera_info->D[3],
      .k3 = camera_info->D[4],
  };
}

apriltag_detector_t *makeTagDetector(apriltag_family_t *tag_family, const bool debug = false)
{
    auto tag_detector = apriltag_detector_create();

    apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

    tag_detector->nthreads = 6;
    tag_detector->wp = workerpool_create(tag_detector->nthreads);
    tag_detector->qtp.min_white_black_diff = 5;
    tag_detector->debug = debug;

    return tag_detector;
}

class FRC971GpuApriltagDetector
{
public:
    FRC971GpuApriltagDetector(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
        : tag_family_{tag36h11_create()}
        , tag_detector_{makeTagDetector(tag_family_)}
        , intrinsics_{cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info->K.data()))}
        , distortion_camera_matrix_{getCameraMatrix(camera_info)}
        , distortion_coefficients_{getDistCoeffs(camera_info)}
        , gpu_detector_{camera_info->width, camera_info->height, tag_detector_, distortion_camera_matrix_, distortion_coefficients_}
    {

    }
    virtual ~FRC971GpuApriltagDetector()
    {
        apriltag_detector_destroy(tag_detector_);
        free(tag_family_);
    }


private:
    bool UndistortDetection(apriltag_detection_t *det) const
    {
        // Copy the undistorted points into det
        bool converged = true;
        for (size_t i = 0; i < 4; i++)
        {
            double u = det->p[i][0];
            double v = det->p[i][1];

            converged &= frc971::apriltag::GpuDetector::UnDistort(&u, &v, &distortion_camera_matrix_,
                                                                  &distortion_coefficients_);
            det->p[i][0] = u;
            det->p[i][1] = v;
        }
        return converged;
    }

    double ComputeDistortionFactor(
        const std::vector<cv::Point2f> &orig_corners,
        const std::vector<cv::Point2f> &corners)
    {
        CHECK_EQ(orig_corners.size(), 4ul);
        CHECK_EQ(corners.size(), 4ul);

        double avg_distance = 0.0;
        for (size_t i = 0; i < corners.size(); i++)
        {
            avg_distance += cv::norm(orig_corners[i] - corners[i]);
        }
        avg_distance /= corners.size();

        // Normalize avg_distance by dividing by the image diagonal,
        // and then the maximum expected distortion
        double distortion_factor =
            avg_distance /
            cv::norm(cv::Point2d(image_size_.width, image_size_.height));
        return std::min(distortion_factor / max_expected_distortion_, 1.0);
    }

    std::vector<cv::Point2f> MakeCornerVector(
        const apriltag_detection_t *det)
    {
        std::vector<cv::Point2f> corner_points;
        corner_points.emplace_back(det->p[0][0], det->p[0][1]);
        corner_points.emplace_back(det->p[1][0], det->p[1][1]);
        corner_points.emplace_back(det->p[2][0], det->p[2][1]);
        corner_points.emplace_back(det->p[3][0], det->p[3][1]);

        return corner_points;
    }

    void DestroyPose(apriltag_pose_t *pose) const
    {
        matd_destroy(pose->R);
        matd_destroy(pose->t);
    }

    apriltag_family_t *tag_family_;
    apriltag_detector_t *tag_detector_;

    cv::Mat intrinsics_;
    frc971::apriltag::CameraMatrix distortion_camera_matrix_;
    frc971::apriltag::DistCoeffs distortion_coefficients_;

    frc971::apriltag::GpuDetector gpu_detector_;
    cv::Size image_size_;

    double max_expected_distortion_{0.314}; // TODO - make this a config item
};
} // namespace
