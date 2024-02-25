#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/grid_prior.h"
#include <cstdio>
// TODO : lots of memory taken up here when only a few of the results are actually used
//        see about turning this into something which generates the few needed values in place
//        rather than duplicating the python functionality of generating the entire grid's coords
template <uint32_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
__global__ void generateGridPriorsKernel(GridPriorValue *output,
                                         const ushort2 modelInputSize,
                                         const uint32_t stride,
                                         const ushort2 imageSize,
                                         const ushort2 *offsets)
{
	const uint32_t w = blockIdx.x * blockDim.x + threadIdx.x;
	const uint32_t h = blockIdx.y * blockDim.y + threadIdx.y;
    uint32_t outputIdx = (modelInputSize.x / stride) * h + w;

    if ((h >= (modelInputSize.y / stride)) || (w >= (modelInputSize.x / stride)))
    {
        return;
    }
    const float scaleWidth = static_cast<float>(modelInputSize.x) / stride;
    const float scaleHeight = static_cast<float>(modelInputSize.y) / stride;
    float2 p{(w + 0.5f) / scaleWidth, (h + 0.5f) / scaleHeight};
    if constexpr (CLAMP)
    {
        p.x = __saturatef(p.x);
        p.y = __saturatef(p.y);
    }
    // Priors scales are duplicated across all batch instances
    // The starting offset is different per batch, though.
    if constexpr (NUM_TILES > 0)
    {
        for (uint32_t i = 0; i < NUM_TILES; i++)
        {
            output[outputIdx].m_scale = p;
            output[outputIdx].m_imageSize = modelInputSize;
            output[outputIdx].m_offset = offsets[i];
            outputIdx += (modelInputSize.x / stride) * (modelInputSize.y / stride);
            // printf("h = %d, w = %d, outputIdx = %d, scaleWidth = %f, scaleHeight = %f, p = %f %f\n", h, w, outputIdx, scaleWidth, scaleHeight, p.x, p.y);
        }
    }
    if constexpr (USE_SCALED_IMAGE)
    {
        // Final non-tiled input image uses this scale with an offset of 0
        // Add in a scale factor to restore from model input size back to original image coords
        //  the tiled inputs don't need those since they're model-input sized tiles anyway
        output[outputIdx + NUM_TILES * (modelInputSize.x / stride) * (modelInputSize.y / stride)].m_scale = p;
        output[outputIdx + NUM_TILES * (modelInputSize.x / stride) * (modelInputSize.y / stride)].m_imageSize = imageSize;
        output[outputIdx + NUM_TILES * (modelInputSize.x / stride) * (modelInputSize.y / stride)].m_offset = ushort2{0, 0};
    }
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
GridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::GridPrior(void)
{
    if constexpr (NUM_TILES != 0)
    {
        cudaSafeCall(cudaMalloc(&m_dOffsets, sizeof(*m_dOffsets) * NUM_TILES));
        cudaSafeCall(cudaMallocHost(&m_hOffsets, sizeof(*m_hOffsets) * NUM_TILES));
    }
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
GridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::~GridPrior()
{
    if constexpr (NUM_TILES != 0)
    {
        cudaSafeCall(cudaFree(m_dOffsets));
        cudaSafeCall(cudaFreeHost(m_hOffsets));
    }
    cudaSafeCall(cudaFree(m_dGridPriors));
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
bool GridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::generate(const ushort2 &modelInputSize,
                                                             const uint32_t stride,
                                                             const ushort2 &imageSize,
                                                             const std::array<ushort2, NUM_TILES> &tileOffsets,
                                                             cudaStream_t cudaStream)
{
    if ((modelInputSize.x != m_modelInputSize.x) || 
        (modelInputSize.y != m_modelInputSize.y) ||
        (imageSize.x != m_imageSize.x) || 
        (imageSize.y != m_imageSize.y) ||
        (stride != m_stride))
    {
        if constexpr (NUM_TILES != 0)
        {
            for (size_t i = 0; i < NUM_TILES; i++)
            {
                m_hOffsets[i] = tileOffsets[i];
            }
            cudaSafeCall(cudaMemcpyAsync(m_dOffsets, m_hOffsets, sizeof(ushort2) * NUM_TILES, cudaMemcpyHostToDevice, cudaStream));
        }
        cudaSafeCall(cudaFreeAsync(m_dGridPriors, cudaStream));
        cudaSafeCall(cudaMallocAsync(&m_dGridPriors, sizeof(GridPriorValue) * (modelInputSize.y / stride) * (modelInputSize.x / stride) * (NUM_TILES + NUM_SCALED_IMAGES), cudaStream));
        const dim3 blockDim(16, 16);
        const dim3 gridDim(iDivUp(modelInputSize.x / stride, blockDim.x), iDivUp(modelInputSize.y / stride, blockDim.y));
        generateGridPriorsKernel<NUM_TILES, USE_SCALED_IMAGE, CLAMP><<<gridDim, blockDim, 0, cudaStream>>>(m_dGridPriors,
                                                                                                           modelInputSize,
                                                                                                           stride,
                                                                                                           imageSize,
                                                                                                           m_dOffsets);
        cudaSafeCall(cudaGetLastError());
        m_modelInputSize = modelInputSize;
        m_imageSize = imageSize;
        m_stride = stride;
        return true;
    }
    return false;
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
const tcb::span<const GridPriorValue> GridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::getOutput(void) const
{
    return tcb::span<const GridPriorValue>(m_dGridPriors, (m_modelInputSize.x / m_stride) * (m_modelInputSize.y / m_stride) * (NUM_TILES + NUM_SCALED_IMAGES));
}

template class GridPrior<0, true, true>;
template class GridPrior<4, false, true>;
template class GridPrior<4, true, true>;
template class GridPrior<9, false, true>;
template class GridPrior<9, true, true>;

#if 0
// CPU implementation for reference
std::vector<float2> generateDevicePriorsCPU(const size_t modelInputHeight, const size_t modelInputWidth, const size_t stride, const bool clamp = true)
{
    const float scaleHeight = static_cast<float>(modelInputHeight) / stride;
    const float scaleWidth = static_cast<float>(modelInputWidth) / stride;
    //std::cout << "scales " << scaleHeight << " " << scaleWidth << std::endl;

    std::vector<float2> ret;
    for (size_t h = 0; h < (modelInputHeight / stride); h++)
    {
        for (size_t w = 0; w < (modelInputWidth / stride); w++)
        {
            float2 p; 
            p.x = (w + 0.5) / scaleWidth;
            p.y = (h + 0.5) / scaleHeight;
            //std::cout << " h = " << h << " w = " << w << " p.x = " << p.x << " p.y = " << p.y << std::endl;
            if (clamp)
            {
                p.x = std::max(0.f, std::min(1.f, p.x));
                p.y = std::max(0.f, std::min(1.f, p.y));
            }
            //std::cout << " h = " << h << " w = " << w << " p.x = " << p.x << " p.y = " << p.y << std::endl;
            ret.emplace_back(p);
        }
    }
    return ret;
}
#endif