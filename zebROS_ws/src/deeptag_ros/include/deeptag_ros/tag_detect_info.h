#ifndef TAG_DETECT_INFO_INC__
#define TAG_DETECT_INFO_INC__

#include <array>
#include <vector>

#include "vector_types.h"

class TagDetectInfo
{
public:
    TagDetectInfo(void) = default;
    TagDetectInfo(const TagDetectInfo &other) = default;
    TagDetectInfo(TagDetectInfo &&other) noexcept = default;

    TagDetectInfo &operator=(const TagDetectInfo &other) = delete;
    TagDetectInfo &operator=(TagDetectInfo &&other) noexcept = default;

    void print(void) const;

    virtual ~TagDetectInfo() = default;

    float2 m_centerPos;
    int m_tagId;
    float m_centerScore;
    float m_centerLinkScore;
    std::array<cv::Point2d, 4> m_orderedCorners;
    std::vector<double> m_cornerScores;
    std::vector<float2> m_anchors;

    int m_mainIndex;
};
#endif
