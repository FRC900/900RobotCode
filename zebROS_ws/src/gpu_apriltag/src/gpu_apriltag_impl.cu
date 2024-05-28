#include <ros/console.h>
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "gpu_apriltag/gpu_apriltag_impl.h"

#include "third_party/apriltag/tag36h11.h"

namespace frc971_gpu_apriltag
{
static frc971::apriltag::CameraMatrix getCameraMatrix(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
      return frc971::apriltag::CameraMatrix{
      .fx = camera_info->K[0],
      .cx = camera_info->K[2],
      .fy = camera_info->K[4],
      .cy = camera_info->K[5]
  };
}

static frc971::apriltag::DistCoeffs getDistCoeffs(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
      return frc971::apriltag::DistCoeffs{
      .k1 = camera_info->D[0],
      .k2 = camera_info->D[1],
      .p1 = camera_info->D[2],
      .p2 = camera_info->D[3],
      .k3 = camera_info->D[4]
  };
}

static apriltag_detector_t *makeTagDetector(apriltag_family_t *tag_family, const size_t num_threads, const bool debug = false)
{
    auto tag_detector = apriltag_detector_create();

    apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

    tag_detector->nthreads = 2;
    tag_detector->wp = workerpool_create(tag_detector->nthreads);
    tag_detector->qtp.min_white_black_diff = 5;
    tag_detector->debug = debug;

    return tag_detector;
}

static std::array<cv::Point2d, 4> MakeCornerArray(const apriltag_detection_t *det)
{
    std::array<cv::Point2d, 4> corner_points;
    for (size_t i = 0; i < corner_points.size(); i++)
    {
        corner_points[i] = cv::Point2d(det->p[i][0], det->p[i][1]);
    }

    return corner_points;
}

#if 0
static void setPose(GpuApriltagResult &result, const apriltag_pose_t &pose)
{
    result.position_ = cv::Point3d(pose.t->data[0], pose.t->data[1], pose.t->data[2]);
    result.orientation_ = cv::Quatd(pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.R->data[3]);
}

static void DestroyPose(apriltag_pose_t *pose)
{
    matd_destroy(pose->R);
    matd_destroy(pose->t);
}

#endif

template <frc971::apriltag::InputFormat INPUT_FORMAT>
FRC971GpuApriltagDetectorImpl<INPUT_FORMAT>::FRC971GpuApriltagDetectorImpl(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
    : tag_family_{tag36h11_create()}
    , tag_detector_{makeTagDetector(tag_family_, 2, false)}
    , intrinsics_{cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info->K.data()))}
    , distortion_camera_matrix_{getCameraMatrix(camera_info)}
    , distortion_coefficients_{getDistCoeffs(camera_info)}
    , gpu_detector_{camera_info->width, camera_info->height, tag_detector_, distortion_camera_matrix_, distortion_coefficients_}
{
}

template <frc971::apriltag::InputFormat INPUT_FORMAT>
FRC971GpuApriltagDetectorImpl<INPUT_FORMAT>::~FRC971GpuApriltagDetectorImpl()
{
    apriltag_detector_destroy(tag_detector_);
    free(tag_family_);
}

template <frc971::apriltag::InputFormat INPUT_FORMAT>
void FRC971GpuApriltagDetectorImpl<INPUT_FORMAT>::Detect(std::vector<GpuApriltagResult> &results,
                                           std::vector<std::array<cv::Point2d, 4>> &rejected_margin_corners,
                                           std::vector<std::array<cv::Point2d, 4>> &rejected_noconverge_corners,
                                           const cv::Mat &color_image)
{
    gpu_detector_.Detect(color_image.data);
    image_size_ = color_image.size();

    const zarray_t *detections = gpu_detector_.Detections();

    const uint32_t min_x = pixel_border_;
    const uint32_t max_x = color_image.cols - pixel_border_;
    const uint32_t min_y = pixel_border_;
    const uint32_t max_y = color_image.rows - pixel_border_;
    results.clear();
    rejected_margin_corners.clear();
    rejected_noconverge_corners.clear();

    for (int i = 0; i < zarray_size(detections); ++i)
    {
        apriltag_detection_t *gpu_detection;

        zarray_get(detections, i, &gpu_detection);

        const bool valid = gpu_detection->decision_margin > min_decision_margin_;

        if (valid)
        {
            // Reject tags that are too close to the boundary, since they often
            // lead to corrupt matches since part of the tag is cut off
            if (gpu_detection->p[0][0] < min_x || gpu_detection->p[0][0] > max_x ||
                gpu_detection->p[1][0] < min_x || gpu_detection->p[1][0] > max_x ||
                gpu_detection->p[2][0] < min_x || gpu_detection->p[2][0] > max_x ||
                gpu_detection->p[3][0] < min_x || gpu_detection->p[3][0] > max_x ||
                gpu_detection->p[0][1] < min_y || gpu_detection->p[0][1] > max_y ||
                gpu_detection->p[1][1] < min_y || gpu_detection->p[1][1] > max_y ||
                gpu_detection->p[2][1] < min_y || gpu_detection->p[2][1] > max_y ||
                gpu_detection->p[3][1] < min_y || gpu_detection->p[3][1] > max_y)
            {
                ROS_DEBUG_STREAM("Rejecting detection because corner is outside pixel border");

                // Send rejected corner points to foxglove in red
                rejected_margin_corners.emplace_back(MakeCornerArray(gpu_detection));
                rejections_++;
                continue;
            }

            ROS_DEBUG("Found GPU %s tag number %d hamming %d margin %f  %f, %f), (%f, "
                      "%f), (%f, %f), (%f, %f)",
                      valid ? "valid" : "invalid", gpu_detection->id,
                      gpu_detection->hamming, gpu_detection->decision_margin,
                      gpu_detection->p[0][0], gpu_detection->p[0][1],
                      gpu_detection->p[1][0], gpu_detection->p[1][1],
                      gpu_detection->p[2][0], gpu_detection->p[2][1],
                      gpu_detection->p[3][0], gpu_detection->p[3][1]);

            ROS_DEBUG_STREAM("Found tag number " << gpu_detection->id
                                                 << " hamming: " << gpu_detection->hamming
                                                 << " margin: " << gpu_detection->decision_margin);

            auto orig_corner_points = MakeCornerArray(gpu_detection);

            const bool converged = UndistortDetection(gpu_detection);

            if (!converged)
            {
                ROS_DEBUG_STREAM("Rejecting detection because Undistort failed to coverge");

                // Send corner points rejected to to lack of convergence in orange
                rejected_noconverge_corners.emplace_back(MakeCornerArray(gpu_detection));
                rejections_++;
                continue;
            }
            auto corner_points = MakeCornerArray(gpu_detection);

            const double distortion_factor =
                ComputeDistortionFactor(orig_corner_points, corner_points);
#if 0
            // First create an apriltag_detection_info_t struct using your known
            // parameters.
            apriltag_detection_info_t info;
            info.tagsize = tag_size_;

            info.fx = intrinsics_.at<double>(0, 0);
            info.fy = intrinsics_.at<double>(1, 1);
            info.cx = intrinsics_.at<double>(0, 2);
            info.cy = intrinsics_.at<double>(1, 2);


            // We're setting this here to use the undistorted corner points in pose estimation.
            info.det = gpu_detection;

            // const aos::monotonic_clock::time_point before_pose_estimation =
            //     aos::monotonic_clock::now();

            apriltag_pose_t pose_1;
            apriltag_pose_t pose_2;
            double pose_error_1;
            double pose_error_2;
            estimate_tag_pose_orthogonal_iteration(&info, &pose_error_1, &pose_1,
                                                   &pose_error_2, &pose_2,
                                                   pose_estimation_iterations_);

            // const aos::monotonic_clock::time_point after_pose_estimation =
            //     aos::monotonic_clock::now();
            // VLOG(1) << "Took "
            //         << chrono::duration<double>(after_pose_estimation -
            //                                     before_pose_estimation)
            //                .count()
            //         << " seconds for pose estimation";
            ROS_INFO_STREAM("Pose err 1: " << std::setprecision(20) << std::fixed
                                            << pose_error_1 << " " << (pose_error_1 < 1e-6 ? "Good" : "Bad"));
            ROS_INFO_STREAM("Pose err 2: " << std::setprecision(20) << std::fixed
                                            << pose_error_2 << " " << (pose_error_2 < 1e-6 ? "Good" : "Bad"));


            // We get two estimates for poses.
            // Choose the one with the lower pose estimation error
            const bool use_pose_1 = (pose_error_1 < pose_error_2);
            auto best_pose = (use_pose_1 ? pose_1 : pose_2);
            auto secondary_pose = (use_pose_1 ? pose_2 : pose_1);
            const double best_pose_error = (use_pose_1 ? pose_error_1 : pose_error_2);
            const double secondary_pose_error = (use_pose_1 ? pose_error_2 : pose_error_1);

            // CHECK_NE(best_pose_error, std::numeric_limits<double>::infinity())
            //     << "Got no valid pose estimations, this should not be possible.";
            double pose_error_ratio = 0.;
            if (!std::isfinite(best_pose_error))
            {
                pose_error_ratio = best_pose_error / secondary_pose_error;
            }

            // Destroy the secondary pose if we got one
            if (secondary_pose_error != std::numeric_limits<double>::infinity())
            {
                DestroyPose(&secondary_pose);
            }
#endif

            results.emplace_back(GpuApriltagResult{});
            results.back().id_ = gpu_detection->id;
            results.back().hamming_ = gpu_detection->hamming;
            results.back().distortion_factor_ = distortion_factor;
            results.back().original_corners_ = orig_corner_points;
            results.back().undistorted_corners_ = corner_points;
            results.back().center_ = cv::Point2d{gpu_detection->c[0], gpu_detection->c[1]};

#if 0
            results.back().pose_error_ = best_pose_error;
            results.back().pose_error_ratio_ = pose_error_ratio;
            if (std::isfinite(best_pose_error))
            {
                setPose(results.back(), best_pose);
                DestroyPose(&best_pose);
            }
#endif

            ROS_DEBUG_STREAM("Found tag number " << gpu_detection->id
                                                 << " hamming: " << gpu_detection->hamming
                                                 << " margin: " << gpu_detection->decision_margin);
        }
        else
        {
            rejections_++;
        }
    }
}

template <frc971::apriltag::InputFormat INPUT_FORMAT>
bool FRC971GpuApriltagDetectorImpl<INPUT_FORMAT>::UndistortDetection(apriltag_detection_t *det) const
{
    // Copy the undistorted points into det
    bool converged = true;
    for (size_t i = 0; i < 4; i++)
    {
        double u = det->p[i][0];
        double v = det->p[i][1];

        converged &= frc971::apriltag::UnDistort(&u, &v, &distortion_camera_matrix_,
                                                 &distortion_coefficients_);
        det->p[i][0] = u;
        det->p[i][1] = v;
    }
    return converged;
}

template <frc971::apriltag::InputFormat INPUT_FORMAT>
double FRC971GpuApriltagDetectorImpl<INPUT_FORMAT>::ComputeDistortionFactor(
    const std::array<cv::Point2d, 4> &orig_corners,
    const std::array<cv::Point2d, 4> &corners) const
{
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

template class FRC971GpuApriltagDetectorImpl<frc971::apriltag::InputFormat::Mono8>;
template class FRC971GpuApriltagDetectorImpl<frc971::apriltag::InputFormat::BGR8>;

} // namespace