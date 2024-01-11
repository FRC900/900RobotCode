#ifndef SSD_BOX_INC__
#define SSD_BOX_INC__

#include "cuda_runtime.h"
struct SSDBoxCenterForm
{
public:
    float2 m_center;
    float2 m_size;
    ushort2 m_imageSize;
    ushort2 m_offset;
};

struct SSDBoxCornerForm
{
public:
    float2 m_tl;
    float2 m_br;
};
#endif