#ifndef STAGE1_GRID_INC__
#define STAGE1_GRID_INC__
#include <cstdint>

#include "vector_types.h"
#include "grid_prior_value.h"

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif

// Used to hold coalesced outputs from stage 1 grid detection
// These are the ones with confidences higher than the threshold passed
// in to the ConfidenceFilter::detect() call
// They will be fed into the stage 1 corner grouper for NMS
template <uint32_t BATCH_SIZE>
struct Stage1Grid
{
    static inline constexpr float softBinsMin = 1e-20f;
    uint32_t m_origIndex{0};
    float m_confidence;
    float2 m_unscaledKeypoints;
    float2 m_keypoints;
    float2 m_cornerDirection;
    int32_t m_group{0};

    __device__ bool check(const Stage1Grid &otherIn, const uint32_t x, const uint32_t y, const float sigma, const float min_cos) const;
    __device__ void copyInto(const float *confidences,
                             const float *keypoints,
                             const float *values,
                             const GridPriorValue *gridPrior,
                             const float centerVariance,
                             const float /*scaleVariance*/,
                             const uint32_t idx,
                             const uint32_t len);

private:
    __device__ uint2 keypointIndexFromConfidenceIndex(const uint32_t softmaxIndex, const uint32_t softmaxLen) const;
    __device__ void cornerDirectionIndexFromConfidenceIndex(uint32_t indexes[4], const uint32_t softmaxIndex, const uint32_t softmaxLen) const;
};
#endif
