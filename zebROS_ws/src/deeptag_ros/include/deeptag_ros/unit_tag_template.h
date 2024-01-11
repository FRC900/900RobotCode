#ifndef UNIT_TAG_TEMPLATE_INC__
#define UNIT_TAG_TEMPLATE_INC__

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "span.hpp"
#include "vector_types.h"

//#define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

class Stage2KeypointGroup;
struct PointsAndIDs;


template <size_t GRID_SIZE, class UNIT_TAG_CLASS, bool IS_NEED_MAIN_IDX, size_t STEP_ELEM_NUM, size_t KPT_START_IDX>
class UnitTagTemplate
{
public:
    UnitTagTemplate();
    UnitTagTemplate(const UnitTagTemplate &other) = delete;
    UnitTagTemplate(UnitTagTemplate &&other) noexcept = delete;

    UnitTagTemplate &operator=(const UnitTagTemplate &other) = delete;
    UnitTagTemplate &operator=(UnitTagTemplate &&other) noexcept = delete;

    virtual ~UnitTagTemplate() = default;
    // Reorder points based on orientation of tag 
    // input is original fineGridPoints
    // mainIdx is the orientation of the tag, in 90* increments (0-3)
    // the two outputs are the rotated fine grid points (points including tag borders),
    // and rotated keypoints (just the "data" bits of the tag)
    template <typename T>
    void reorderPointsWithMainIdx(std::array<T, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &fineGridPointsRotated,
                                  std::array<T, GRID_SIZE * GRID_SIZE> &keypointsRotated,
                                  const size_t mainIdx,
                                  const std::array<T, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &fineGridPoints) const
    {
        constexpr auto N = GRID_SIZE + 2;
        auto reorderedXYs = m_unitTags.getFineGridPoints(mainIdx, false, STEP_ELEM_NUM);
        for (size_t i = 0; i < fineGridPointsRotated.size(); i++)
        {
            const auto &p = reorderedXYs[i];
            size_t idx = p.y / STEP_ELEM_NUM * N + p.x / STEP_ELEM_NUM;
            fineGridPointsRotated[i] = fineGridPoints[idx];
#ifdef DEBUG
            std::cout << "fineGridPointsRotated[" << i << "] = " << fineGridPointsRotated[i] << std::endl;
#endif
        }

        std::vector<uint8_t> labels;
        reorderedXYs = m_unitTags.getKeypointsWithLabels(labels, mainIdx, false, STEP_ELEM_NUM); // don't need labels
        for (size_t i = 0; i < keypointsRotated.size(); i++)
        {
            const auto &p = reorderedXYs[i + KPT_START_IDX];
            size_t idx = p.y / STEP_ELEM_NUM * N + p.x / STEP_ELEM_NUM;
            keypointsRotated[i] = fineGridPoints[idx];
#ifdef DEBUG
            std::cout << "keypointsRotated[" << i << "] = " << keypointsRotated[i] << std::endl;
#endif
        }
    }
    template <typename T>
    void reorderPointsWithMainIdx(std::vector<T> &fineGridPointsRotated,
                                  std::vector<T> &keypointsRotated,
                                  const size_t mainIdx,
                                  const std::vector<T> &fineGridPoints) const
    {
        constexpr auto N = GRID_SIZE + 2;
        auto reorderedXYs = m_unitTags.getFineGridPoints(mainIdx, false, STEP_ELEM_NUM);
        fineGridPointsRotated.clear();
        for (size_t i = 0; i < fineGridPoints.size(); i++)
        {
            const auto &p = reorderedXYs[i];
            size_t idx = p.y / STEP_ELEM_NUM * N + p.x / STEP_ELEM_NUM;
            fineGridPointsRotated.push_back(fineGridPoints[idx]);
        }

        std::vector<uint8_t> labels;
        reorderedXYs = m_unitTags.getKeypointsWithLabels(labels, mainIdx, false, STEP_ELEM_NUM); // don't need labels
        keypointsRotated.clear();
        for (size_t i = 0; i < labels.size(); i++)
        {
            const auto &p = reorderedXYs[i + KPT_START_IDX];
            size_t idx = p.y / STEP_ELEM_NUM * N + p.x / STEP_ELEM_NUM;
            keypointsRotated.push_back(fineGridPoints[idx]);
        }
    }

    void matchFineGrid(double &matchRatio,
                       std::array<PointsAndIDs, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &fineGridPointsAndIDs,
                       const tcb::span<const Stage2KeypointGroup> &stage2KeypointGroups,
                       const cv::Mat &H,
                       const tcb::span<const float2> &stage2PredCorners,
                       const cv::Mat &cameraMatrix,
                       const cv::Mat &distCoeffs) const;

    std::array<cv::Point2d, 4> updateCornersInImage(const std::array<PointsAndIDs, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &orderedPoints,
                                                    const cv::Mat &HCrop,
                                                    const cv::Mat &cameraMatrix,
                                                    const cv::Mat &distCoeffs) const;

    const UNIT_TAG_CLASS& getUnitTags(void) const { return m_unitTags; };

    static constexpr size_t getGridSize(void) { return GRID_SIZE; }

private:
    const UNIT_TAG_CLASS m_unitTags;
};

#include "unit_arucotag.h"
template <size_t GRID_SIZE>
class UnitTagTemplateArucotag : public UnitTagTemplate<GRID_SIZE, UnitArucoTag<GRID_SIZE>, false, 1, 0>
{
public:
    UnitTagTemplateArucotag(void);
    UnitTagTemplateArucotag(const UnitTagTemplateArucotag &other) = delete;
    UnitTagTemplateArucotag(UnitTagTemplateArucotag &&other) noexcept = delete;

    UnitTagTemplateArucotag &operator=(const UnitTagTemplateArucotag &other) = delete;
    UnitTagTemplateArucotag &operator=(UnitTagTemplateArucotag &&other) noexcept = delete;

    virtual ~UnitTagTemplateArucotag() = default;
};

#endif
