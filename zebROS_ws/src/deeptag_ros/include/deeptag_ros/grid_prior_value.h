#ifndef GRID_PRIOR_VALUE_INC__
#define GRID_PRIOR_VALUE_INC__

struct GridPriorValue
{
    float2 m_scale;
    // TODO : imageSize and offset are the same for all points in a batch.
    // this could be reduced from 2 values per result to 2 values per batch somehow 
    ushort2 m_imageSize;
    ushort2 m_offset;
};
#endif
