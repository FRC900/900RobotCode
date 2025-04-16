#ifndef STAGE2_KEYPOINT_INC__
#define STAGE2_KEYPOINT_INC__
#include <cstdint>
#include "deeptag_ros/grid_prior_value.h"

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif

class Stage2Keypoint
{
public:
    Stage2Keypoint(void);

    Stage2Keypoint(const Stage2Keypoint &other) = delete;
    Stage2Keypoint(Stage2Keypoint &&other) noexcept = delete;

    Stage2Keypoint &operator=(const Stage2Keypoint &other) = delete;
    Stage2Keypoint &operator=(Stage2Keypoint &&other) noexcept = delete;

    virtual ~Stage2Keypoint();

    uint32_t m_origIdx;
    float m_scoreCand;
    float m_scoreSum; // sum of both foreground options
    float2 m_keypointCand;
    int m_label;

    __device__ bool check(const Stage2Keypoint &otherIn, int x, int y, const float sigma, const float min_cos) const;
    __device__ void copyInto(const float *confidences,
                             const float *keypoints,
                             const float *values,
                             const GridPriorValue *gridPrior,
                             const float centerVariance,
                             const float /*scaleVariance*/,
                             const uint32_t idx,
                             const uint32_t len);
};

#endif
