#ifndef STAGE1_GRID_GROUP_INC__
#define STAGE1_GRID_GROUP_INC__

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif

#include "cuda_runtime.h"
// Used to store merged Stage1Grid similar data as a single
// group rather than deal with duplicates of individual results
template <uint32_t BATCH_SIZE>
struct Stage1Grid;

template <uint32_t BATCH_SIZE>
class Stage1GridGroup
{
public:
    __host__ __device__ Stage1GridGroup(void);

    Stage1GridGroup(const Stage1GridGroup &other) = delete;
    Stage1GridGroup(Stage1GridGroup &&other) noexcept = delete;

    Stage1GridGroup &operator=(const Stage1GridGroup &other) = delete;
    Stage1GridGroup &operator=(Stage1GridGroup &&other) noexcept = delete;

    virtual __host__ __device__ ~Stage1GridGroup();

    __device__ void reset();
    __device__ void append(const Stage1Grid<BATCH_SIZE> &Stage1Grid);
    __device__ void end(void);

    __device__ __host__ void print(void) const;

    float m_score{};
    float m_score_sum{};
    int m_id{};
    float2 m_corner{};
    float2 m_cornerDirection{}; // likely CornerDirection
};
#endif