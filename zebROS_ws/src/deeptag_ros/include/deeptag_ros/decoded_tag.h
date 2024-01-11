#ifndef DECODED_TAG_INC__
#define DECODED_TAG_INC__
#include <array>
#include <cstdint>
#include "opencv2/core.hpp"
#include "points_and_ids.h"

template <size_t GRID_SIZE>
class DecodedTag
{
public:
    bool m_isValid;
    cv::Mat m_HCrop;
    int m_tagId;
    uint64_t m_binaryId;
    std::array<PointsAndIDs, (GRID_SIZE + 2) * (GRID_SIZE + 2)> m_keypointsWithIds;
    // TODO : this is just the points from the previous field copied into a different var
    // see about combining them
    std::array<cv::Point2d, (GRID_SIZE + 2) * (GRID_SIZE + 2)> m_keypointsInImage;
    std::array<cv::Point2d, 4> m_roi;

    int m_mainIdx{0};
};

#endif
