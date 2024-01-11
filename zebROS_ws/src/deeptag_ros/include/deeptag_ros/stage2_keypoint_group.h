#ifndef STAGE2_KEYPOINT_GROUP_INC__
#define STAGE2_KEYPOINT_GROUP_INC__

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
#include "cuda_runtime.h"
#include <iostream>

// Used to store merged Stage1Grid similar data as a single
// group rather than deal with duplicates of individual results
class Stage2Keypoint;
class Stage2KeypointGroup
{
public:
    __host__ __device__ Stage2KeypointGroup(void);

    Stage2KeypointGroup(const Stage2KeypointGroup &other) = default;
    Stage2KeypointGroup(Stage2KeypointGroup &&other) noexcept = delete;

    Stage2KeypointGroup &operator=(const Stage2KeypointGroup &other) = delete;
    Stage2KeypointGroup &operator=(Stage2KeypointGroup &&other) noexcept = delete;

    virtual __host__ __device__ ~Stage2KeypointGroup();

    __device__ void reset();
    __device__ void append(const Stage2Keypoint &Stage1Grid);
    __device__ void end(void);

    __device__ __host__ void print(void) const;
    friend __host__ std::ostream &operator<<(std::ostream &os, const Stage2KeypointGroup &kg)
    {
        os << "\tm_label = " << kg.m_label << std::endl;
        os << "\tm_label = " << kg.m_label << std::endl;
        os << "\t\tm_score = " << kg.m_score << std::endl;
        os << "\t\tm_keypoint = " << kg.m_keypoint.x << ", " << kg.m_keypoint.y << std::endl;
        os << "\t\tm_score_sum = " << kg.m_score_sum << std::endl;
        return os;
    }

    float m_score{};
    float m_score_sum{};
    int m_label{};
    float2 m_keypoint{};

private:
};
#endif