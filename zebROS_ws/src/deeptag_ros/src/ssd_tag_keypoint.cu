#include <cstdio>
#include "deeptag_ros/ssd_tag_keypoint.h"

__device__ void SSDTagKeypoint::copyInto(const float *confidences,
                                         const float *locations,
                                         const float *values,
                                         const SSDBoxCenterForm *gridPrior,
                                         const float centerVariance,
                                         const float sizeVariance,
                                         const uint32_t idx,
                                         const uint32_t len)
{
    m_origIndex = idx; // For debugging
    // Just copy confidence over unchanged
    m_confidence = confidences[idx];
    // Unscaled keypoint for debugging
    m_unscaledSSDBox.m_center.x = locations[4 * idx];
    m_unscaledSSDBox.m_center.y = locations[4 * idx + 1];
    m_unscaledSSDBox.m_size.x = locations[4 * idx + 2];
    m_unscaledSSDBox.m_size.y = locations[4 * idx + 3];
    //printf("m_unscaledSSDBox[%d] = (%f %f) (%f %f)\n", idx, m_unscaledSSDBox.m_center.x, m_unscaledSSDBox.m_center.y, m_unscaledSSDBox.m_size.x, m_unscaledSSDBox.m_size.y);

    // Scale up keypoints so they're screen coords rather than 0-1 normalized
    SSDBoxCenterForm ssdBoxCenter;
    ssdBoxCenter.m_center.x = (m_unscaledSSDBox.m_center.x * centerVariance * gridPrior[idx].m_size.x + gridPrior[idx].m_center.x) * gridPrior[idx].m_imageSize.x + gridPrior[idx].m_offset.x;
    ssdBoxCenter.m_center.y = (m_unscaledSSDBox.m_center.y * centerVariance * gridPrior[idx].m_size.y + gridPrior[idx].m_center.y) * gridPrior[idx].m_imageSize.y + gridPrior[idx].m_offset.y;
    ssdBoxCenter.m_size.x   = (expf(m_unscaledSSDBox.m_size.x * sizeVariance) * gridPrior[idx].m_size.x) * gridPrior[idx].m_imageSize.x;
    ssdBoxCenter.m_size.y   = (expf(m_unscaledSSDBox.m_size.y * sizeVariance) * gridPrior[idx].m_size.y) * gridPrior[idx].m_imageSize.y;
    //printf("ssdBoxCenter[%d] = (%f %f) (%f %f)\n", idx, ssdBoxCenter.m_center.x, ssdBoxCenter.m_center.y, ssdBoxCenter.m_size.x, ssdBoxCenter.m_size.y);

    // Convert from center, size to tl, br corner
    m_SSDBoxCornerForm.m_tl.x = ssdBoxCenter.m_center.x - ssdBoxCenter.m_size.x / 2.f;
    m_SSDBoxCornerForm.m_tl.y = ssdBoxCenter.m_center.y - ssdBoxCenter.m_size.y / 2.f;
    m_SSDBoxCornerForm.m_br.x = ssdBoxCenter.m_center.x + ssdBoxCenter.m_size.x / 2.f;
    m_SSDBoxCornerForm.m_br.y = ssdBoxCenter.m_center.y + ssdBoxCenter.m_size.y / 2.f;
    //printf("scale = (%f %f)\n", scale.x, scale.y);
    //printf("m_SSDBoxCornerForm[%d] = (%f %f) (%f %f)\n", idx, m_SSDBoxCornerForm.m_tl.x, m_SSDBoxCornerForm.m_tl.y, m_SSDBoxCornerForm.m_br.x, m_SSDBoxCornerForm.m_br.y);

    for (size_t i = 0; i < 10; i++)
    {
        m_values[i] = values[10 * idx + i];
    }
    for (size_t i = 0; i < 4; i++)
    {
        m_anchors[i].x = m_values[i * 2 + 0] * (m_SSDBoxCornerForm.m_br.x - m_SSDBoxCornerForm.m_tl.x) + (m_SSDBoxCornerForm.m_tl.x + m_SSDBoxCornerForm.m_br.x) / 2.f;
        m_anchors[i].y = m_values[i * 2 + 1] * (m_SSDBoxCornerForm.m_br.y - m_SSDBoxCornerForm.m_tl.y) + (m_SSDBoxCornerForm.m_tl.y + m_SSDBoxCornerForm.m_br.y) / 2.f;
    }
    // Last 2 entries in m_values is the center_with_labels_cand field in python code, keypoints in grouping code
    m_centerCand.x = m_values[4 * 2 + 0] * (m_SSDBoxCornerForm.m_br.x - m_SSDBoxCornerForm.m_tl.x) + (m_SSDBoxCornerForm.m_tl.x + m_SSDBoxCornerForm.m_br.x) / 2.f;
    m_centerCand.y = m_values[4 * 2 + 1] * (m_SSDBoxCornerForm.m_br.y - m_SSDBoxCornerForm.m_tl.y) + (m_SSDBoxCornerForm.m_tl.y + m_SSDBoxCornerForm.m_br.y) / 2.f;
    //printf("m_centerCand[%d] = %f %f\n", idx, m_centerCand.x, m_centerCand.y);
    m_centerSigma = ((m_SSDBoxCornerForm.m_br.x - m_SSDBoxCornerForm.m_tl.x) + (m_SSDBoxCornerForm.m_br.y - m_SSDBoxCornerForm.m_tl.y)) / 6.f;
    //printf("idx = %d, m_centerSigma = %f\n", idx, m_centerSigma);
}

__device__ bool SSDTagKeypoint::check(const SSDTagKeypoint &otherIn, const uint32_t x, const uint32_t y, const float /*sigma*/, const float /*min_cos*/) const
{
    if (hypotf(m_centerCand.x - otherIn.m_centerCand.x, m_centerCand.y - otherIn.m_centerCand.y) > m_centerSigma)
    {
        return false;
    }
    return true;
}
