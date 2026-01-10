#ifndef UNIT_CHESSBOARD_TAG_INC__
#define UNIT_CHESSBOARD_TAG_INC__

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

#include "deeptag_ros/bool_tag.h"

class UnitChessboardTagBase
{
public:
    UnitChessboardTagBase(void) = default;
    UnitChessboardTagBase(const UnitChessboardTagBase &other) = delete;
    UnitChessboardTagBase(UnitChessboardTagBase &&other) noexcept = delete;

    UnitChessboardTagBase &operator=(const UnitChessboardTagBase &other) = delete;
    UnitChessboardTagBase &operator=(UnitChessboardTagBase &&other) noexcept = delete;

    virtual ~UnitChessboardTagBase() = default;
};


template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
class UnitChessboardTag : public UnitChessboardTagBase
{
public:
    UnitChessboardTag(const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> &elementLabels,
                      const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> &keypointLabels,
                      const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> &maskLabels);
    UnitChessboardTag(const UnitChessboardTag &other) = delete;
    UnitChessboardTag(UnitChessboardTag &&other) noexcept = delete;

    UnitChessboardTag &operator=(const UnitChessboardTag &other) = delete;
    UnitChessboardTag &operator=(UnitChessboardTag &&other) noexcept = delete;
    ~UnitChessboardTag() override = default;
    std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> getKeypoints(const size_t mainIdx = 0,
                                                                          const bool isCenter = false,
                                                                          const size_t stepElemNum = 1,
                                                                          const cv::Mat &H = cv::Mat()) const;
    std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> getKeypointsWithLabels(std::vector<uint8_t> &labels,
                                                                                    const size_t mainIdx = 0,
                                                                                    const bool isCenter = false,
                                                                                    const size_t stepElemNum = 1,
                                                                                    const cv::Mat &H = cv::Mat()) const;

    std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> getFineGridPoints(const size_t mainIdx = 0,
                                                                               const bool isCenter = false,
                                                                               const size_t stepElemNum = 1,
                                                                               const cv::Mat &H = cv::Mat()) const;

    cv::Point2d getCenterPos(const cv::Mat &H = cv::Mat()) const;

    std::array<cv::Point2d, 4> getOrderedCorners(const size_t mainIdx = 0, const cv::Mat &H = cv::Mat()) const;

    static size_t getFineGridSize(void) { return FINE_GRID_SIZE; }
private:
    template <bool WITH_LABELS>
    std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> getXandY(std::vector<uint8_t> *reorderedLabels,
                                                                      const bool isCenter,
                                                                      const size_t mainIdx,
                                                                      const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> *labels,
                                                                      const int maxLabel,
                                                                      const size_t stepElemNum,
                                                                      const cv::Mat &H,
                                                                      const BoolTag<WITH_LABELS>) const;

    std::vector<size_t> getOrderedIdxList(const size_t gridSize,
                                          const size_t mainIdx = 0,
                                          const size_t steElemNum = 1) const;

    const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> m_elementLabels;
    const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> m_keypointLabels;
    const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> m_maskLabels;
};
#endif
