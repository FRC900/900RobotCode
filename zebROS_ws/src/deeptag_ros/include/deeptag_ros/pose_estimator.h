#ifndef POSE_ESTIMATOR_INC__
#define POSE_ESTIMATOR_INC__

#include <array>                   // for array
#include <cstddef>                 // for size_t
#include <opencv2/core/mat.hpp>    // for Mat
#include <opencv2/core/types.hpp>  // for Point2d
template <bool BOOL> struct BoolTag;

#undef  DEBUG
#include "debug.h"

template <size_t GRID_SIZE, class UNIT_TAG_TYPE, size_t STEP_ELEM_NUM, bool IS_RANSAC_SOLVEPNP, bool IS_INVERSE_X = true>
class PoseEstimator
{
    public:
        PoseEstimator(const cv::Mat &cameraMatrix,
                      const cv::Mat &distCoeffs,
                      const UNIT_TAG_TYPE &unitTag,
                      const double tagRealSizeInMeter);
        PoseEstimator(const PoseEstimator &other) = delete;
        PoseEstimator(PoseEstimator &&other) noexcept = delete;

        PoseEstimator &operator=(const PoseEstimator &other) = delete;
        PoseEstimator &operator=(PoseEstimator &&other) noexcept = delete;
        virtual ~PoseEstimator() = default;

        bool fineGridKeypointsToPose(cv::Mat &rVecs, cv::Mat &tVecs, const std::array<cv::Point2d, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &kptsInImage) const;
        bool tagCornerKeypointsToPose(cv::Mat &rVecs, cv::Mat &tVecs, const std::array<cv::Point2d, 4> &cptsInImage) const;
        void drawVecs(cv::Mat &image, const cv::Mat &rVec, const cv::Mat &tVec, const bool isInverseZ = false) const;

    private:
        template <size_t N, bool USE_RANSAC_SOLVEPNP>
        bool keypointsToPose(const BoolTag<USE_RANSAC_SOLVEPNP>,
                            cv::Mat &rVecs,
                            cv::Mat &tVecs,
                            const std::array<cv::Point2d, N> &kptsInImage,
                            const std::array<cv::Point2d, N> &tagKptsAnno) const;

        cv::Mat m_cameraMatrix;
        cv::Mat m_distCoeffs;
        std::array<cv::Point2d, (GRID_SIZE + 2) * (GRID_SIZE + 2)> m_fineGridPointsAnno;
        const std::array<cv::Point2d, 4> m_tagCornersAnno{
            cv::Point2d{-0.5, -0.5},
            cv::Point2d{0.5, -0.5},
            cv::Point2d{0.5, 0.5},
            cv::Point2d{-0.5, 0.5}
        };
        const double m_tagRealSizeInMeter;
};

#include "unit_arucotag.h"
template <size_t GRID_SIZE>
using ArucoPoseEstimator = PoseEstimator<GRID_SIZE, UnitArucoTag<GRID_SIZE>, 1, false, true>;
#endif