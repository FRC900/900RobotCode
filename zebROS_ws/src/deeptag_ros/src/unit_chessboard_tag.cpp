#include "deeptag_ros/unit_chessboard_tag.h"
#include "deeptag_ros/warp_perspective_points.h"
template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::UnitChessboardTag(const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> &elementLabels,
                                                                                const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> &keypointLabels,
                                                                                const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> &maskLabels)
    : UnitChessboardTagBase()
    , m_elementLabels{elementLabels}
    , m_keypointLabels{keypointLabels}
    , m_maskLabels{maskLabels}
{
#if 0
    std::cout << "m_elementLabels"<< std::endl;
    std::copy(m_elementLabels.cbegin(), m_elementLabels.cend(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;

    std::cout << "m_keypointLabels"<< std::endl;
    std::copy(m_keypointLabels.cbegin(), m_keypointLabels.cend(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;

    std::cout << "m_maskLabels"<< std::endl;
    std::copy(m_maskLabels.cbegin(), m_maskLabels.cend(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;
#endif
}
template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
template <bool WITH_LABELS>
std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::getXandY(std::vector<uint8_t> *reorderedLabels,
                                                                                                                                const bool isCenter,
                                                                                                                                const size_t mainIdx,
                                                                                                                                const std::array<uint8_t, FINE_GRID_SIZE * FINE_GRID_SIZE> *labels,
                                                                                                                                const int maxLabel,
                                                                                                                                const size_t stepElemNum,
                                                                                                                                const cv::Mat &H,
                                                                                                                                const BoolTag<WITH_LABELS>) const
{
    const float offset = isCenter ? 0.5f : 0.0f;

    std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> defaultXYList;
    for (size_t y = 0; y < FINE_GRID_SIZE; y++)
    {
        for (size_t x = 0; x < FINE_GRID_SIZE; x++)
        {
            defaultXYList[y * FINE_GRID_SIZE + x] = cv::Point2d{x + offset, y + offset};
        }
    }
    const auto orderedIdxList = getOrderedIdxList(FINE_GRID_SIZE, 0, stepElemNum);
    const auto orderedIdxListRotated = getOrderedIdxList(FINE_GRID_SIZE, mainIdx, stepElemNum);

    std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> reorderedXYs;
    if constexpr (WITH_LABELS)
    {
        if (reorderedLabels)
        {
            reorderedLabels->clear();
        }
    }
    size_t outIdx = 0;
    for (size_t i = 0; i < orderedIdxList.size(); i++)
    {
        // Filter out points with labels > maxLabel
        // used to get non-border points of the grid
        if constexpr (WITH_LABELS)
        {
            const auto idx = orderedIdxList[i];
            if ((maxLabel >= 0) && labels && (labels->size() > idx))
            {
                const auto thisLabel = (*labels)[idx];
                if (thisLabel > maxLabel)
                {
                    continue;
                }
                else if (reorderedLabels)
                {
                    reorderedLabels->push_back(thisLabel);
                }
            }
        }
        reorderedXYs[outIdx++] = defaultXYList[orderedIdxListRotated[i]];
    }
    if (!H.empty())
    {
        warpPerspectivePts(H, reorderedXYs);
    }
    return reorderedXYs;
}

template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
std::vector<size_t> UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::getOrderedIdxList(const size_t gridSize,
                                                                                                    const size_t mainIdx,
                                                                                                    const size_t stepElemNum) const
{
    std::vector<size_t> orderedIdxList;
    for (size_t i = 0; i < gridSize; i += stepElemNum)
    {
        for (size_t j = 0; j < gridSize; j += stepElemNum)
        {
            if (mainIdx == 0)
            {
                orderedIdxList.push_back(i * gridSize + j);
            }
            else if (mainIdx == 1)
            {
                orderedIdxList.push_back(j * gridSize + (gridSize -i - 1));
            }
            else if (mainIdx == 2)
            {
                orderedIdxList.push_back((gridSize - i - 1) * gridSize + (gridSize - j - 1));
            }
            else if (mainIdx == 3)
            {
                orderedIdxList.push_back((gridSize - j - 1) * gridSize + i);
            }
        }
    }
    return orderedIdxList;
}

template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::getKeypoints(const size_t mainIdx,
                                                                                                                                const bool isCenter,
                                                                                                                                const size_t stepElemNum,
                                                                                                                                const cv::Mat &H) const
{
    return getXandY(nullptr, isCenter, mainIdx, nullptr, MAX_LABEL, stepElemNum, H, BoolTag<false>{});
}

template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::getKeypointsWithLabels(std::vector<uint8_t> &labels,
                                                                                                                                          const size_t mainIdx,
                                                                                                                                          const bool isCenter,
                                                                                                                                          const size_t stepElemNum,
                                                                                                                                          const cv::Mat &H) const
{
    return getXandY(&labels, isCenter, mainIdx, &m_keypointLabels, MAX_LABEL, stepElemNum, H, BoolTag<true>{});
}

template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
std::array<cv::Point2d, FINE_GRID_SIZE * FINE_GRID_SIZE> UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::getFineGridPoints(const size_t mainIdx,
                                                                                                                                     const bool isCenter,
                                                                                                                                     const size_t stepElemNum,
                                                                                                                                     const cv::Mat &H) const
{
    return getXandY(nullptr, isCenter, mainIdx, nullptr, -1, stepElemNum, H, BoolTag<false>{});
}

template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
cv::Point2d UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::getCenterPos(const cv::Mat &H) const
{
    std::array<cv::Point2d, 1> centerPos = {cv::Point2d{FINE_GRID_SIZE / 2., FINE_GRID_SIZE / 2.}};
    if (!H.empty())
    {
        warpPerspectivePts(H, centerPos);
    }
    return centerPos[0];
}

template <size_t FINE_GRID_SIZE, uint8_t MAX_LABEL, uint8_t MAX_ELEM_LABEL>
std::array<cv::Point2d, 4> UnitChessboardTag<FINE_GRID_SIZE, MAX_LABEL, MAX_ELEM_LABEL>::getOrderedCorners(const size_t mainIdx, const cv::Mat &H) const
{
    const std::array<cv::Point2d, 4> corners = {cv::Point2d{0, 0}, cv::Point2d{FINE_GRID_SIZE, 0}, cv::Point2d{FINE_GRID_SIZE, FINE_GRID_SIZE}, cv::Point2d{0, FINE_GRID_SIZE}};
    std::array<cv::Point2d, 4> orderedCorners;

    for (size_t i = 0; i < orderedCorners.size(); i++)
    {
        orderedCorners[(i + mainIdx) % orderedCorners.size()] = corners[i];
    }
    if (!H.empty())
    {
        warpPerspectivePts(H, orderedCorners);
    }

    return orderedCorners;
}
template class UnitChessboardTag<6, 1, 1>; // 16h5
template class UnitChessboardTag<7, 1, 1>; // 25h9
template class UnitChessboardTag<8, 1, 1>; // 36h10/36h11