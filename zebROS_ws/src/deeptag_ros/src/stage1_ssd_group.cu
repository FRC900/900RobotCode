  #include "deeptag_ros/ssd_tag_keypoint.h"
  #include "deeptag_ros/stage1_ssd_group.h"

  #include <cstdio>

__host__ __device__ Stage1SSDGroup::Stage1SSDGroup(void)
{
}

__host__ __device__ Stage1SSDGroup::~Stage1SSDGroup(void)
{
}

__device__ void Stage1SSDGroup::reset()
{
    //printf("Resetting group\n");
    m_score = 0.f;
    m_id = 0;
    m_score_sum = 0.f;
    m_bbox.m_tl.x = 0.f;
    m_bbox.m_tl.y = 0.f;
    m_bbox.m_br.x = 0.f;
    m_bbox.m_br.y = 0.f;
    for (size_t i = 0 ; i < 4; i++)
    {
        m_anchorsInBox[i].x = 0.f;
        m_anchorsInBox[i].y = 0.f;
    }
    m_center.x = 0.f;
    m_center.y = 0.f;
}
__device__ void Stage1SSDGroup::append(const SSDTagKeypoint &ssdTagKeypoint)
{
    // Score is max confidence of all group member's scores
    m_score = max(m_score, ssdTagKeypoint.m_confidence);
    m_score_sum += ssdTagKeypoint.m_confidence;

    // Coords are confidence-weighted averages
    m_bbox.m_tl.x += ssdTagKeypoint.m_SSDBoxCornerForm.m_tl.x * ssdTagKeypoint.m_confidence;
    m_bbox.m_tl.y += ssdTagKeypoint.m_SSDBoxCornerForm.m_tl.y * ssdTagKeypoint.m_confidence;
    m_bbox.m_br.x += ssdTagKeypoint.m_SSDBoxCornerForm.m_br.x * ssdTagKeypoint.m_confidence;
    m_bbox.m_br.y += ssdTagKeypoint.m_SSDBoxCornerForm.m_br.y * ssdTagKeypoint.m_confidence;

    m_center.x += ssdTagKeypoint.m_centerCand.x * ssdTagKeypoint.m_confidence;
    m_center.y += ssdTagKeypoint.m_centerCand.y * ssdTagKeypoint.m_confidence;

    for (size_t i = 0 ; i < 4; i++)
    {
        m_anchorsInBox[i].x += ssdTagKeypoint.m_anchors[i].x * ssdTagKeypoint.m_confidence;
        m_anchorsInBox[i].y += ssdTagKeypoint.m_anchors[i].y * ssdTagKeypoint.m_confidence;
    }

    //printf("Adding corner to group, groupNum = %d\n", groupNum);
    //print();
}
__device__ void Stage1SSDGroup::end()
{
    m_bbox.m_tl.x /= m_score_sum;
    m_bbox.m_tl.y /= m_score_sum;
    m_bbox.m_br.x /= m_score_sum;
    m_bbox.m_br.y /= m_score_sum;
    m_center.x /= m_score_sum;
    m_center.y /= m_score_sum;
    for (size_t i = 0 ; i < 4; i++)
    {
        m_anchorsInBox[i].x /= m_score_sum;
        m_anchorsInBox[i].y /= m_score_sum;
    }
    //printf("Ending group\n");
    //print();
}

__device__ __host__ void Stage1SSDGroup::print(void) const
{
    printf("\tm_score = %f\n", m_score);
    printf("\t\tm_score_sum = %f\n", m_score_sum);
    printf("\t\tm_box = (%f, %f) (%f, %f)\n",
           m_bbox.m_tl.x, m_bbox.m_tl.y,
           m_bbox.m_br.x, m_bbox.m_br.y);
    printf("\t\tm_center = %f %f\n", m_center.x, m_center.y);
    for (int32_t i = 0; i < 4; i++)
    {
        printf("\t\tm_anchorsInBox[%d] = %f %f\n", i, m_anchorsInBox[i].x, m_anchorsInBox[i].y);
    };
}
