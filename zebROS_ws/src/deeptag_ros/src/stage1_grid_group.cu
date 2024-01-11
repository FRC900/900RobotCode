  #include "deeptag_ros/stage1_grid.h"
  #include "deeptag_ros/stage1_grid_group.h"
  #include <cstdint>

  #include <cstdio>

template <uint32_t BATCH_SIZE>
__host__ __device__ Stage1GridGroup<BATCH_SIZE>::Stage1GridGroup(void)
{
    //printf("Calling constructor for Stage1GridGroup\n");
}

template <uint32_t BATCH_SIZE>
__host__ __device__ Stage1GridGroup<BATCH_SIZE>::~Stage1GridGroup(void)
{
}

template <uint32_t BATCH_SIZE>
__device__ void Stage1GridGroup<BATCH_SIZE>::reset()
{
    //printf("Resetting group\n");
    m_score = 0.f;
    m_score_sum = 0.f;
    m_id = 0;
    m_corner.x = 0.f;
    m_corner.y = 0.f;
    m_cornerDirection.x = 0.f;
    m_cornerDirection.y = 0.f;
}

template <uint32_t BATCH_SIZE>
__device__ void Stage1GridGroup<BATCH_SIZE>::append(const Stage1Grid<BATCH_SIZE> &Stage1Grid)
{
    // Score is max confidence of all group member's scores
    m_score = max(m_score, Stage1Grid.m_confidence);
    m_score_sum += Stage1Grid.m_confidence;

    // Keypoints is weighted sum of input keypoints.
    // Weight is normalized confidence of each input
    m_corner.x += Stage1Grid.m_keypoints.x * Stage1Grid.m_confidence;
    m_corner.y += Stage1Grid.m_keypoints.y * Stage1Grid.m_confidence;

    // Direction is similar to keypoints,
    // except turned into a normalized vector at the end of the process
    m_cornerDirection.x += Stage1Grid.m_cornerDirection.x * Stage1Grid.m_confidence;
    m_cornerDirection.y += Stage1Grid.m_cornerDirection.y * Stage1Grid.m_confidence;
    
    //printf("Adding corner to group, groupNum = %d\n", groupNum);
    //print();
}

template <uint32_t BATCH_SIZE>
__device__ void Stage1GridGroup<BATCH_SIZE>::end()
{
    m_corner.x /= m_score_sum;
    m_corner.y /= m_score_sum;
    m_cornerDirection.x /= m_score_sum;
    m_cornerDirection.y /= m_score_sum;
    const float norm = max(hypotf(m_cornerDirection.x, m_cornerDirection.y), 1e-20f);
    m_cornerDirection.x /= norm;
    m_cornerDirection.y /= norm;
    //printf("Ending group\n");
    //print();
}

template <uint32_t BATCH_SIZE>
__device__ __host__ void Stage1GridGroup<BATCH_SIZE>::print(void) const
{
    printf("\tm_score = %f\n", m_score);
    printf("\t\tm_score_sum = %f\n", m_score_sum);
    printf("\t\tm_keypoint = %f %f\n", m_corner.x, m_corner.y);
    printf("\t\tm_cornerDirection = %f %f\n", m_cornerDirection.x, m_cornerDirection.y);
}

template class Stage1GridGroup<1>;
template class Stage1GridGroup<4>;
template class Stage1GridGroup<5>;
template class Stage1GridGroup<9>;
template class Stage1GridGroup<10>;