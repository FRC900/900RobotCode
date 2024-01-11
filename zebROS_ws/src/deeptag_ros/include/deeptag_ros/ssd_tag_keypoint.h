#ifndef SSD_TAG_KEYPOINT_INC__
#define SSD_TAG_KEYPOINT_INC__

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif

#include <cstdint>
#include "ssd_box.h"
class SSDTagKeypoint
{
public:
    __device__ void copyInto(const float *confidences,
                             const float *keypoints,
                             const float *values,
                             const SSDBoxCenterForm *gridPrior,
                             const float centerVariance,
                             const float sizeVariance,
                             const uint32_t idx,
                             const uint32_t len);
    __device__ bool check(const SSDTagKeypoint &otherIn, const uint32_t x, const uint32_t y, const float /*sigma*/, const float /*min_cos*/) const;

    static inline constexpr float softBinsMin = 1e-20f;
    uint32_t m_origIndex;
    float m_confidence;
    SSDBoxCenterForm m_unscaledSSDBox; // for debugging?
    SSDBoxCornerForm m_SSDBoxCornerForm;
    float  m_values[10];
    float2 m_centerCand;
    float2 m_anchors[4];
    float  m_centerSigma;
};
#endif
