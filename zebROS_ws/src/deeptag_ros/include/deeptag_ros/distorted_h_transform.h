#ifndef DISTORTED_H_TRANSFORM_INC__
#define DISTORTED_H_TRANSFORM_INC__

#include <array>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "warp_perspective_points.h"
#undef  DEBUG
#include "debug.h"

template <size_t N>
std::array<cv::Point2d, N> fromCropToUndistorted(const std::array<cv::Point2d, N> &pointsInCrop,
                                                 const cv::Mat &cameraMatrix,
                                                 const cv::Mat &distCoeffs,
                                                 const cv::Mat &H)
{
    auto pointsInImage = warpPerspectivePts(H.inv(), pointsInCrop);
    printPoints("pointsInImage", pointsInImage);
    if (cameraMatrix.empty() || distCoeffs.empty())
    {
        return pointsInImage;
    }
    std::array<cv::Point2d, N> pointsInImageUndistorted;
    cv::undistortPoints(pointsInImage, pointsInImageUndistorted, cameraMatrix, distCoeffs);
    printPoints("pointsInImageUndistorted", pointsInImageUndistorted);
    return pointsInImageUndistorted;
}

template <size_t N>
std::array<cv::Point2d, N> fromUndistortedToCrop(const std::array<cv::Point2d, N> &pointsUndistorted,
                                                 const cv::Mat &cameraMatrix,
                                                 const cv::Mat &distCoeffs,
                                                 const cv::Mat &H)
{
    if (cameraMatrix.empty() || distCoeffs.empty())
    {
        return warpPerspectivePts(H, pointsUndistorted);
    }

    std::array<cv::Point3d, N> pointsUndistortedH;
    cv::convertPointsToHomogeneous(pointsUndistorted, pointsUndistortedH);

    std::array<cv::Point2d, N> pointsInImageCV;
    cv::Mat zeros(1, 3, CV_64FC1, cv::Scalar(0));
    cv::projectPoints(pointsUndistortedH, zeros, zeros, cameraMatrix, distCoeffs, pointsInImageCV);
    warpPerspectivePts(H, pointsInImageCV);
    return pointsInImageCV;
}

// Given the transform found by matching cptsGt (control points ground truth) to cptsInCrop
// Transform kptsGt (keypoints GT) to keypoints in cropped image space
template <size_t N1, size_t N2>
std::array<cv::Point2d, N2> controlpointsToKeypointsInCropWithH(const std::array<cv::Point2d, N1> &cptsGt,
                                                                const std::array<cv::Point2d, N1> &cptsInCrop,
                                                                const std::array<cv::Point2d, N2> &kptsGt,
                                                                const cv::Mat &cameraMatrix,
                                                                const cv::Mat &distCoeffs,
                                                                const cv::Mat &H)
{
    if (cameraMatrix.empty() || distCoeffs.empty() || H.empty())
    {
        const auto HGt2Undist = cv::findHomography(cptsGt, cptsInCrop);
        return warpPerspectivePts(HGt2Undist, kptsGt);
    }
    auto cptsUndistorted = fromCropToUndistorted(cptsInCrop, cameraMatrix, distCoeffs, H);
    printPoints("cptsGt", cptsGt);
    printPoints("cptsUndistorted", cptsUndistorted);
    const auto HGt2Undist = cv::findHomography(cptsGt, cptsUndistorted);
#ifdef DEBUG
    std::cout << "HGt2Undist = " << HGt2Undist << std::endl;
#endif
    const auto keypointsUndistorted = warpPerspectivePts(HGt2Undist, kptsGt);
    printPoints("keypointsUndistorted", keypointsUndistorted);
    return fromUndistortedToCrop(keypointsUndistorted, cameraMatrix, distCoeffs, H);
}

#endif