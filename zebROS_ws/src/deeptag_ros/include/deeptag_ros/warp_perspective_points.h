#ifndef WARP_PERSPECTIVE_POINTS_INC__
#define WARP_PERSPECTIVE_POINTS_INC__

#include <math.h>                        // for fabs
#include <opencv2/core/hal/interface.h>  // for CV_64FC1
#include <stddef.h>                      // for size_t
#include <array>                         // for array
#include <opencv2/core/mat.hpp>          // for Mat, MatExpr, operator*
#include <opencv2/core/mat.inl.hpp>      // for Mat::at, Mat::operator()
#include <opencv2/core/types.hpp>        // for Point2d, Rect
#include <vector>                        // for vector

#undef  DEBUG
#include "deeptag_ros/debug.h"

std::vector<cv::Point2d> warpPerspectivePts(const cv::Mat &H, const std::vector<cv::Point2d> &points, const double image_scale = 1.);

template <size_t N>
void warpPerspectivePts(const cv::Mat &H, std::array<cv::Point2d, N> &points, const double image_scale = 1.)
{
    if (H.rows <= 0)
    {
        return;
    }
    // H is CV64FC1, so these have to match and be doubles rather than floats
    cv::Mat pts(points.size(), 2, CV_64FC1);
    cv::Mat ptsHom(points.size(), 3, CV_64FC1);
    for (int i = 0; i < static_cast<int>(points.size()); i++)
    {
        pts.at<double>(i, 0) = points[i].x;
        pts.at<double>(i, 1) = points[i].y;
        ptsHom.at<double>(i, 0) = points[i].x;
        ptsHom.at<double>(i, 1) = points[i].y;
        ptsHom.at<double>(i, 2) = 1.0f;
    }
    cv::Mat ptsWarp = (H * ptsHom.t()).t();

    ptsWarp(cv::Rect(0, 0, 2, ptsWarp.rows)).copyTo(pts);
    for (int i = 0; i < pts.rows; i++)
    {
        const auto divisor = (fabs(ptsWarp.at<double>(i,2)) < 1e-20) ? 1e-20 : ptsWarp.at<double>(i, 2);
        pts.at<double>(i, 0) /= divisor;
        pts.at<double>(i, 1) /= divisor;
    }
    for (int i = 0; i < pts.rows; i++)
    {
        points[i] = cv::Point2d{pts.at<double>(i, 0) / image_scale, pts.at<double>(i, 1) / image_scale};
    }
    printPoints("points", points);
}

template <size_t N>
std::array<cv::Point2d, N> warpPerspectivePts(const cv::Mat &H, const std::array<cv::Point2d, N> &points, const double image_scale = 1.)
{
    if (H.rows <= 0)
    {
        return points;
    }
    // H is CV64FC1, so these have to match and be doubles rather than floats
    cv::Mat pts(points.size(), 2, CV_64FC1);
    cv::Mat ptsHom(points.size(), 3, CV_64FC1);
    for (int i = 0; i < static_cast<int>(points.size()); i++)
    {
        pts.at<double>(i, 0) = points[i].x;
        pts.at<double>(i, 1) = points[i].y;
        ptsHom.at<double>(i, 0) = points[i].x;
        ptsHom.at<double>(i, 1) = points[i].y;
        ptsHom.at<double>(i, 2) = 1.0f;
    }
    cv::Mat ptsWarp = (H * ptsHom.t()).t();

    ptsWarp(cv::Rect(0, 0, 2, ptsWarp.rows)).copyTo(pts);
    for (int i = 0; i < pts.rows; i++)
    {
        const auto divisor = (fabs(ptsWarp.at<double>(i,2)) < 1e-20) ? 1e-20 : ptsWarp.at<double>(i, 2);
        pts.at<double>(i, 0) /= divisor;
        pts.at<double>(i, 1) /= divisor;
    }
    std::array<cv::Point2d, N> ret;
    for (int i = 0; i < pts.rows; i++)
    {
        ret[i] = cv::Point2d{pts.at<double>(i, 0) / image_scale, pts.at<double>(i, 1) / image_scale};
    }
    printPoints("ret", ret);
    return ret;
}
#endif