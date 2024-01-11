#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

#undef  DEBUG
#include "deeptag_ros/debug.h"
#include "deeptag_ros/pose_estimator.h"
#include "deeptag_ros/warp_perspective_points.h"
template <size_t GRID_SIZE, class UNIT_TAG_TYPE, size_t STEP_ELEM_NUM, bool IS_RANSAC_SOLVEPNP, bool IS_INVERSE_X>
PoseEstimator<GRID_SIZE, UNIT_TAG_TYPE, STEP_ELEM_NUM, IS_RANSAC_SOLVEPNP, IS_INVERSE_X>::PoseEstimator(const cv::Mat &cameraMatrix,
                                                                                                        const cv::Mat &distCoeffs,
                                                                                                        const UNIT_TAG_TYPE &unitTag,
                                                                                                        const double tagRealSizeInMeter)
    : m_cameraMatrix{cameraMatrix}
    , m_distCoeffs{distCoeffs}
    , m_fineGridPointsAnno{unitTag.getFineGridPoints(0, true, STEP_ELEM_NUM)}
    , m_tagRealSizeInMeter{tagRealSizeInMeter}
{
    const auto w = unitTag.getFineGridSize();
    const auto centerXY = unitTag.getCenterPos();

    const cv::Mat H1 = (cv::Mat_<double>(3, 3) << 1., 0., -centerXY.x, 0., 1., -centerXY.y, 0., 0., 1.);
    const cv::Mat H2 = (cv::Mat_<double>(3, 3) << 1. / static_cast<double>(w), 0., 0., 0., 1. / static_cast<double>(w), 0., 0., 0., 1.);

    const auto H = H2 * H1;

    warpPerspectivePts(H, m_fineGridPointsAnno);
    if constexpr (IS_INVERSE_X)
    {
        for (auto &p : m_fineGridPointsAnno)
        {
            p.x *= -1.0;
        }
    }
}

template <size_t GRID_SIZE, class UNIT_TAG_TYPE, size_t STEP_ELEM_NUM, bool IS_RANSAC_SOLVEPNP, bool IS_INVERSE_X>
bool PoseEstimator<GRID_SIZE, UNIT_TAG_TYPE, STEP_ELEM_NUM, IS_RANSAC_SOLVEPNP, IS_INVERSE_X>::fineGridKeypointsToPose(cv::Mat &rVec,
                                                                                                                       cv::Mat &tVec,
                                                                                                                       const std::array<cv::Point2d, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &kptsInImage) const
{
    return keypointsToPose(BoolTag<IS_RANSAC_SOLVEPNP>{}, rVec, tVec, kptsInImage, m_fineGridPointsAnno);
}

template <size_t GRID_SIZE, class UNIT_TAG_TYPE, size_t STEP_ELEM_NUM, bool IS_RANSAC_SOLVEPNP, bool IS_INVERSE_X>
bool PoseEstimator<GRID_SIZE, UNIT_TAG_TYPE, STEP_ELEM_NUM, IS_RANSAC_SOLVEPNP, IS_INVERSE_X>::tagCornerKeypointsToPose(cv::Mat &rVec,
                                                                                                                        cv::Mat &tVec,
                                                                                                                        const std::array<cv::Point2d, 4> &cptsInImage) const
{
    return keypointsToPose(BoolTag<false>{}, rVec, tVec, cptsInImage, m_tagCornersAnno);
}

template <size_t GRID_SIZE, class UNIT_TAG_TYPE, size_t STEP_ELEM_NUM, bool IS_RANSAC_SOLVEPNP, bool IS_INVERSE_X>
template <size_t N, bool USE_RANSAC_SOLVEPNP>
bool PoseEstimator<GRID_SIZE, UNIT_TAG_TYPE, STEP_ELEM_NUM, IS_RANSAC_SOLVEPNP, IS_INVERSE_X>::keypointsToPose(const BoolTag<USE_RANSAC_SOLVEPNP>,
                                                                                                               cv::Mat &rVec,
                                                                                                               cv::Mat &tVec,
                                                                                                               const std::array<cv::Point2d, N> &kptsInImage,
                                                                                                               const std::array<cv::Point2d, N> &tagKptsAnno) const
{
    if (kptsInImage.size() == 0)
    {
        return false;
    }
    std::array<cv::Point3d, N> scaledTagKpts;
    for (size_t i = 0; i < scaledTagKpts.size(); i++)
    {
        scaledTagKpts[i] = cv::Point3d{tagKptsAnno[i].x * m_tagRealSizeInMeter, tagKptsAnno[i].y * m_tagRealSizeInMeter, 0};
    }
    printPoints("scaledTagKpts", scaledTagKpts);
    printPoints("kptsInImage", kptsInImage);

    if constexpr (USE_RANSAC_SOLVEPNP)
    {
        return cv::solvePnPRansac(scaledTagKpts, kptsInImage, m_cameraMatrix, m_distCoeffs, rVec, tVec);
    }
    else
    {
        return cv::solvePnP(scaledTagKpts, kptsInImage, m_cameraMatrix, m_distCoeffs, rVec, tVec);
    }
}

template <size_t GRID_SIZE, class UNIT_TAG_TYPE, size_t STEP_ELEM_NUM, bool IS_RANSAC_SOLVEPNP, bool IS_INVERSE_X>
void PoseEstimator<GRID_SIZE, UNIT_TAG_TYPE, STEP_ELEM_NUM, IS_RANSAC_SOLVEPNP, IS_INVERSE_X>::drawVecs(cv::Mat &image,
                                                                                                        const cv::Mat &rVec,
                                                                                                        const cv::Mat &tVec,
                                                                                                        const bool isInverseZ) const
{
    std::vector<cv::Point3f> axis;
    axis.emplace_back(1, 0, 0);
    axis.emplace_back(0, 1, 0);
    axis.emplace_back(0, 0, isInverseZ ? -1 : 1);

    for (auto &a : axis)
    {
        a *= m_tagRealSizeInMeter / 2.;
    }

    std::vector<cv::Point2f> imgPts;
    cv::projectPoints(axis, rVec, tVec, m_cameraMatrix, m_distCoeffs, imgPts);

    std::vector<cv::Point2f> imgPtsCorner;
    axis.clear();
    axis.emplace_back(0, 0, 0);
    cv::projectPoints(axis, rVec, tVec, m_cameraMatrix, m_distCoeffs, imgPtsCorner);

    cv::line(image, imgPtsCorner[0], imgPts[0], {0, 0, 255}, 3);
    cv::line(image, imgPtsCorner[0], imgPts[1], {0, 255, 0}, 3);
    cv::line(image, imgPtsCorner[0], imgPts[2], {255, 0, 0}, 3);
}

template class PoseEstimator<4, UnitArucoTag<4>, 1, false, true>;
template class PoseEstimator<5, UnitArucoTag<5>, 1, false, true>;
template class PoseEstimator<6, UnitArucoTag<6>, 1, false, true>;