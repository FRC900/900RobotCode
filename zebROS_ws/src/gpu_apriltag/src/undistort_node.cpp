#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include "apriltag_msgs/ApriltagArrayStamped.h"

struct CameraMatrix {
  double fx;
  double cx;
  double fy;
  double cy;
};

struct DistCoeffs {
  double k1;
  double k2;
  double p1;
  double p2;
  double k3;
  double k4;
  double k5;
  double k6;
  // Are we using 5 or 8 parameter model?
  int num_params;
};

static CameraMatrix getCameraMatrix(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
      return CameraMatrix{
      .fx = camera_info->K[0],
      .cx = camera_info->K[2],
      .fy = camera_info->K[4],
      .cy = camera_info->K[5]
  };
}

static DistCoeffs getDistCoeffs(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
      return DistCoeffs{
      .k1 = camera_info->D[0],
      .k2 = camera_info->D[1],
      .p1 = camera_info->D[2],
      .p2 = camera_info->D[3],
      .k3 = camera_info->D[4],
      .k4 = 0.0,
      .k5 = 0.0,
      .k6 = 0.0,
      .num_params = 5
  };
}

// We're undistorting using math found from this github page
// https://yangyushi.github.io/code/2020/03/04/opencv-undistort.html
// Also, see for reference the source code:
// https://github.com/opencv/opencv/blob/master/modules/calib3d/src/undistort.dispatch.cpp
bool UnDistort(double *u, double *v,
               const CameraMatrix *camera_matrix,
               const DistCoeffs *distortion_coefficients)
{
    constexpr int kUndistortIterationThreshold = 100;
    constexpr double kUndistortConvergenceEpsilon = 1e-6;
    bool converged = true;
    const double k1 = distortion_coefficients->k1;
    const double k2 = distortion_coefficients->k2;
    const double p1 = distortion_coefficients->p1;
    const double p2 = distortion_coefficients->p2;
    const double k3 = distortion_coefficients->k3;
    const double k4 = distortion_coefficients->k4;
    const double k5 = distortion_coefficients->k5;
    const double k6 = distortion_coefficients->k6;

    const double fx = camera_matrix->fx;
    const double cx = camera_matrix->cx;
    const double fy = camera_matrix->fy;
    const double cy = camera_matrix->cy;

    const double xPP = (*u - cx) / fx;
    const double yPP = (*v - cy) / fy;

    double xP = xPP;
    double yP = yPP;

    double x0 = xP;
    double y0 = yP;

    double prev_x = 0, prev_y = 0;

    int iterations = 0;
    do
    {
        prev_x = xP;
        prev_y = yP;
        double rSq = xP * xP + yP * yP;

        double radial_distortion =
            (1 + (k1 * rSq) + (k2 * rSq * rSq) + (k3 * rSq * rSq * rSq)) /
            (1 + (k4 * rSq) + (k5 * rSq * rSq) + (k6 * rSq * rSq * rSq));

        double radial_distortion_inv = 1 / radial_distortion;

        double tangential_dx = 2 * p1 * xP * yP + p2 * (rSq + 2 * xP * xP);
        double tangential_dy = p1 * (rSq + 2 * yP * yP) + 2 * p2 * xP * yP;

        xP = (x0 - tangential_dx) * radial_distortion_inv;
        yP = (y0 - tangential_dy) * radial_distortion_inv;

        if (iterations > kUndistortIterationThreshold)
        {
            converged = false;
            break;
        }

        iterations++;
    } while (std::abs(xP - prev_x) > kUndistortConvergenceEpsilon ||
             std::abs(yP - prev_y) > kUndistortConvergenceEpsilon);

    //   if (iterations < kUndistortIterationThreshold) {
    //     VLOG(1) << "Took " << iterations << " iterations to reach convergence.";
    //   } else {
    //     VLOG(1) << "Took " << iterations
    //             << " iterations and didn't reach convergence with "
    //             << " (xP, yP): "
    //             << " (" << xP << ", " << yP << ")"
    //             << " vs. (prev_x, prev_y): "
    //             << " (" << prev_x << ", " << prev_y << ")";
    //   }

    *u = xP * fx + cx;
    *v = yP * fy + cy;

    return converged;
}
int main(int argc, char **argv)
{
    ros::init (argc, argv, "undistort_node");
    ros::NodeHandle nh;

    // Get camera info from ros subscriber
    CameraMatrix distortion_camera_matrix_;
    DistCoeffs distortion_coefficients_;
    bool valid_camera_info = false;
    ros::Subscriber camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info", 1, [&](const sensor_msgs::CameraInfo::ConstPtr &camera_info)
    {
        if (camera_info->height == 0 || camera_info->width == 0)
        {
            ROS_ERROR_STREAM_THROTTLE(1.0, "Camera info not valid - make sure .yaml file is loaded from ~/.ros/camera_info/filename.yaml.");
            return;
        }
        distortion_camera_matrix_ = getCameraMatrix(camera_info);
        distortion_coefficients_ = getDistCoeffs(camera_info);
        valid_camera_info = true;
    });

    // Republish apriltag detections with undistorted coordinates
    ros::Publisher tags_out = nh.advertise<apriltag_msgs::ApriltagArrayStamped>("tags_undistorted", 1);
    ros::Subscriber tags_in = nh.subscribe<apriltag_msgs::ApriltagArrayStamped>("tags", 1, [&](const apriltag_msgs::ApriltagArrayStamped::ConstPtr &apriltag_array)
    {
        if (!valid_camera_info)
        {
            ROS_ERROR_STREAM_THROTTLE(1.0, "Camera info not valid - make sure camera_info topic is correct");
            return;
        }

        apriltag_msgs::ApriltagArrayStamped out_msg = *apriltag_array;
 
        for (auto &tag : out_msg.apriltags)
        {
            for (size_t i = 0; i < 4; ++i)
            {
                UnDistort(&tag.corners[i].x,
                          &tag.corners[i].y,
                          &distortion_camera_matrix_,
                          &distortion_coefficients_);
            }
            UnDistort(&tag.center.x,
                      &tag.center.y,
                      &distortion_camera_matrix_,
                      &distortion_coefficients_);
        }
        tags_out.publish(*apriltag_array);
    });

    ros::spin();
    return 0;
}