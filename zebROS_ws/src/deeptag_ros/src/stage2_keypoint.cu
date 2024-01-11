#include "deeptag_ros/stage2_keypoint.h"
#include <cstdio>

__device__ void Stage2Keypoint::copyInto(const float *confidences,
                                         const float *keypoints,
                                         const float */*nullptr*/,
                                         const GridPriorValue *gridPrior,
                                         const float centerVariance,
                                         const float /*scaleVariance*/,
                                         const uint32_t idx,
                                         const uint32_t len)
{
    m_origIdx = idx;
    const float c0 = confidences[idx];
    const float c1 = confidences[idx + len];
    m_label = (c0 > c1) ? 0 : 1;
    m_scoreCand = max(c0, c1);
    m_scoreSum = c0 + c1;
    m_keypointCand.x = (keypoints[idx] * centerVariance + gridPrior[idx].m_scale.x) * gridPrior[idx].m_imageSize.x + gridPrior[idx].m_offset.x;
    m_keypointCand.y = (keypoints[idx + len] * centerVariance + gridPrior[idx].m_scale.y) * gridPrior[idx].m_imageSize.y + gridPrior[idx].m_offset.y;
    //printf ("%s : idx = %d, c0 = %f, c1 = %f, m_label = %d, m_scoreCand = %f, m_keypointCand = %f %f\n", __PRETTY_FUNCTION__, idx, c0, c1, m_label, m_scoreCand, m_keypointCand.x, m_keypointCand.y);
}

__device__ bool Stage2Keypoint::check(const Stage2Keypoint &otherIn, int x, int y, const float sigma, const float min_cos) const
{
    if (hypotf(m_keypointCand.x - otherIn.m_keypointCand.x, m_keypointCand.y - otherIn.m_keypointCand.y) > sigma)
    {
        //if (y == 0)
        //printf("false : x = %d, y = %d, kp = %f %f, otherKp = %f %f, signma = %f\n", x, y, m_keypointCand.x, m_keypointCand.y, otherIn.m_keypointCand.x, otherIn.m_keypointCand.y, sigma);
        return false;
    }
    // if (y == 0)
    // printf("true  : x = %d, y = %d, kp = %f %f, otherKp = %f %f, signma = %f\n", x, y, m_keypointCand.x, m_keypointCand.y, otherIn.m_keypointCand.x, otherIn.m_keypointCand.y, sigma);
    return true;
}