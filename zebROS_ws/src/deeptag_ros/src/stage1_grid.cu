#include <cstdio>
#include "deeptag_ros/stage1_grid.h"

// index of the softmax entry is a linear address space, already post-processed from
//  the network ouput
// This needs to be converted into an index for accessing the batches of
//  keypoint x and y values. The addressing scheme for those:
//  for each batch, all x's come first, then all y's.
//  each batch repeats this 
template <uint32_t BATCH_SIZE>
__device__ uint2 Stage1Grid<BATCH_SIZE>::keypointIndexFromConfidenceIndex(const uint32_t softmaxIndex, const uint32_t softmaxLen) const
{
    const uint32_t batchCount = softmaxLen / BATCH_SIZE;      // number of entries per batch
    const uint32_t batchNum = softmaxIndex / batchCount;      // which batch we're in
    const uint32_t offsetInBatch = softmaxIndex % batchCount; // which entry in the current batch

    uint2 ret;

    // Each batch produces 2 entries (an x and a y coord).  All 
    // batchCount x's come first, so y's index is x's + batchCount
    ret.x = 2 * batchCount * batchNum + offsetInBatch;
    ret.y = ret.x + batchCount;

    return ret;
}

// Similar to above, only this time there are 4 values per result.
template <uint32_t BATCH_SIZE>
__device__ void Stage1Grid<BATCH_SIZE>::cornerDirectionIndexFromConfidenceIndex(uint32_t indexes[4], const uint32_t softmaxIndex, const uint32_t softmaxLen) const
{
    const uint32_t batchCount = softmaxLen / BATCH_SIZE;      // number of entries per batch
    const uint32_t batchNum = softmaxIndex / batchCount;      // which batch we're in
    const uint32_t offsetInBatch = softmaxIndex % batchCount; // which entry in the current batch

    for (uint32_t i = 0; i < 4; i++)
    {
        indexes[i] = 2 * batchCount * batchNum + i * batchCount + offsetInBatch;
    }
}

template <uint32_t BATCH_SIZE>
__device__ void Stage1Grid<BATCH_SIZE>::copyInto(const float *confidences,
                                                 const float *keypoints,
                                                 const float *values,
                                                 const GridPriorValue *gridPrior,
                                                 const float centerVariance,
                                                 const float /*scaleVariance*/,
                                                 const uint32_t idx,
                                                 const uint32_t len)
{
    m_origIndex = idx; // For debugging
    // Just copy confidence over unchanged
    m_confidence = confidences[idx];
    // Unscaled keypoint for debugging
    const auto keypointIdx = keypointIndexFromConfidenceIndex(idx, len);
    m_unscaledKeypoints.x = keypoints[keypointIdx.x];
    m_unscaledKeypoints.y = keypoints[keypointIdx.y];

    // Scale up keypoints so they're screen coords rather than 0-1 normalized
    m_keypoints.x = (keypoints[keypointIdx.x] * centerVariance + gridPrior[idx].m_scale.x) * gridPrior[idx].m_imageSize.x + gridPrior[idx].m_offset.x;
    m_keypoints.y = (keypoints[keypointIdx.y] * centerVariance + gridPrior[idx].m_scale.y) * gridPrior[idx].m_imageSize.y + gridPrior[idx].m_offset.y;

    uint32_t softBinIndexes[4];
    cornerDirectionIndexFromConfidenceIndex(softBinIndexes, idx, len);
    const float softBins[4] = {__saturatef(values[softBinIndexes[0]]),
                               __saturatef(values[softBinIndexes[1]]),
                               __saturatef(values[softBinIndexes[2]]),
                               __saturatef(values[softBinIndexes[3]])};
    // soft_bins_to_vec : convert 4 values to a normalized vector
    float xx = softBins[0] / max(softBins[0] + softBins[2], softBinsMin);
    float yy = softBins[1] / max(softBins[1] + softBins[3], softBinsMin);

    xx = (xx - 0.5f) * 2.f;
    yy = (yy - 0.5f) * 2.f;
    float vecNorm = max(hypotf(xx, yy), softBinsMin);
    m_cornerDirection.x = xx / vecNorm;
    m_cornerDirection.y = yy / vecNorm;
}

__device__ static float norm3d(const float2 &in)
{
    return hypot(in.x, in.y);
}

template <uint32_t BATCH_SIZE>
__device__ bool Stage1Grid<BATCH_SIZE>::check(const Stage1Grid &otherIn, const uint32_t x, const uint32_t y, const float sigma, const float min_cos) const
{
#if 0
    printf("x = %d, y = %d, score = %f, m_keypoint = %f %f, other keypoint = %f %f, sigma = %f, dist = %f\n",
           x, y, m_confidence,
           m_keypoints.x, m_keypoints.y,
           otherIn.m_keypoints.x, otherIn.m_keypoints.y,
           sigma,
           hypotf(m_keypoints.x - otherIn.m_keypoints.x, m_keypoints.y - otherIn.m_keypoints.y));
#endif
    if (hypotf(m_keypoints.x - otherIn.m_keypoints.x, m_keypoints.y - otherIn.m_keypoints.y) > sigma)
    {
        return false;
    }
    // get_cos_dist from heatmap_postprocess.py
    const float2 &thisCorner = m_cornerDirection;
    const float2 &otherCorner = otherIn.m_cornerDirection;
    float dist_cos = thisCorner.x * otherCorner.x + thisCorner.y * otherCorner.y;
    //printf("this = %f %f, other = %f %f, dist_cos = %f\n", thisCorner.x, thisCorner.y, otherCorner.x, otherCorner.y, dist_cos);
    dist_cos /= max(norm3d(thisCorner) * norm3d(otherCorner), 1.e-20f);
    dist_cos = min(max(-1.f, dist_cos), 1.f);
    if (dist_cos < min_cos)
    {
        //printf("\t false: this = %f %f, other = %f %f, dist_cos = %f, min_cos = %f\n", thisCorner.x, thisCorner.y, otherCorner.x, otherCorner.y, dist_cos, min_cos);
        return false;
    }
    return true;
}

template class Stage1Grid<1>;
template class Stage1Grid<4>;
template class Stage1Grid<5>;
template class Stage1Grid<9>;
template class Stage1Grid<10>;