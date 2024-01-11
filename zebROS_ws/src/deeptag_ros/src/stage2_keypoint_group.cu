  #include "deeptag_ros/stage2_keypoint_group.h"
  #include "deeptag_ros/stage2_keypoint.h"

  #include <cstdio>

__host__ __device__ Stage2KeypointGroup::Stage2KeypointGroup(void)
{
}

__host__ __device__ Stage2KeypointGroup::~Stage2KeypointGroup(void)
{
}

__device__ void Stage2KeypointGroup::reset()
{
    m_score = 0.f;
    m_score_sum = 0.f;
    m_label = 0;
    m_keypoint.x = 0.f;
    m_keypoint.y = 0.f;
}
__device__ void Stage2KeypointGroup::append(const Stage2Keypoint &stage2Keypoint)
{
    m_label = stage2Keypoint.m_label;
    // Score is max confidence of all group member's scores
    m_score = max(m_score, stage2Keypoint.m_scoreCand);
    m_score_sum += stage2Keypoint.m_scoreCand;

    // Keypoints is weighted sum of input keypoints.
    // Weight is normalized confidence of each input
    m_keypoint.x += stage2Keypoint.m_keypointCand.x * stage2Keypoint.m_scoreCand;
    m_keypoint.y += stage2Keypoint.m_keypointCand.y * stage2Keypoint.m_scoreCand;

    //printf("Adding corner to group, groupNum = %d\n", groupNum);
    //print();
}
__device__ void Stage2KeypointGroup::end()
{
    m_keypoint.x /= m_score_sum;
    m_keypoint.y /= m_score_sum;
    //printf("Ending group\n");
    //print();
}

__device__ __host__ void Stage2KeypointGroup::print(void) const
{
    printf("\tm_label = %d\n", m_label);
    printf("\t\tm_score = %f\n", m_score);
    printf("\t\tm_keypoint = %f %f\n", m_keypoint.x, m_keypoint.y);
    printf("\t\tm_score_sum = %f\n", m_score_sum);
}