#ifndef STAGE1_SSD_GROUP_INC__
#define STAGE1_SSD_GROUP_INC__
#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
#include "deeptag_ros/ssd_box.h"

// Used to store merged Stage1Grid similar data as a single
// group rather than deal with duplicates of individual results
class SSDTagKeypoint;
class Stage1SSDGroup
{
public:
    __host__ __device__ Stage1SSDGroup(void);

    Stage1SSDGroup(const Stage1SSDGroup &other) = delete;
    Stage1SSDGroup(Stage1SSDGroup &&other) noexcept = delete;

    Stage1SSDGroup &operator=(const Stage1SSDGroup &other) = delete;
    Stage1SSDGroup &operator=(Stage1SSDGroup &&other) noexcept = delete;

    virtual __host__ __device__ ~Stage1SSDGroup();

    __device__ void reset();
    __device__ void append(const SSDTagKeypoint &ssdTagKeypoint);
    __device__ void end(void);

    __device__ __host__ void print(void) const;

    float m_score{};
    float m_score_sum{};
    int m_id;
    SSDBoxCornerForm m_bbox;
    float2 m_center{};
    float2 m_anchorsInBox[4]; 

private:
};
#endif