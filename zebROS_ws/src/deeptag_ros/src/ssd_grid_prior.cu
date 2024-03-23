#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/ssd_grid_prior.h"
class SSDSpec
{
public:
    SSDSpec(const size_t featureMapSizeH,
            const size_t featureMapSizeW,
            const size_t shrinkage,
            const size_t boxSizeMin,
            const size_t boxSizeMax,
            const size_t aspectRatioMin,
            const size_t aspectRatioMax)
        : m_featureMapSizeH{featureMapSizeH}
        , m_featureMapSizeW{featureMapSizeW}
        , m_shrinkage{shrinkage}
        , m_boxSizeMin{boxSizeMin}
        , m_boxSizeMax{boxSizeMax}
        , m_aspectRatioMin{aspectRatioMin}
        , m_aspectRatioMax{aspectRatioMax}
    {
    }
    size_t m_featureMapSizeH;
    size_t m_featureMapSizeW;

    size_t m_shrinkage;
    size_t m_boxSizeMin;
    size_t m_boxSizeMax;
    size_t m_aspectRatioMin;
    size_t m_aspectRatioMax;
};
static constexpr uint32_t boxesPerSize = 6;

// TODO : lots of memory taken up here when only a few of the results are actually used
//        see about turning this into something which generates the few needed values in place
//        rather than duplicating the python functionality of generating the entire grid's coords
template <uint32_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
__global__ void generateSSDGridPriorsKernel(SSDBoxCenterForm *output,
                                            const float2 modelInputSize,
                                            const ushort2 imageSize,
                                            const uint32_t featureMapHeight,
                                            const uint32_t featureMapWidth,
                                            const float shrinkage,
                                            const float boxSizeMin, 
                                            const float boxSizeMax,
                                            const float aspectRatioMin, 
                                            const float aspectRatioMax,
                                            const uint32_t outputsPerBatch,
                                            const ushort2 *offsets)
{
	const uint32_t w = blockIdx.x * blockDim.x + threadIdx.x;
	const uint32_t h = blockIdx.y * blockDim.y + threadIdx.y;
    uint32_t outputIdx = (featureMapWidth * h + w) * boxesPerSize;

    if ((h >= featureMapHeight) || (w >= featureMapWidth))
    {
        return;
    }
    const float2 scale = {modelInputSize.x / shrinkage,
                          modelInputSize.y / shrinkage};

    float2 center = {(static_cast<float>(w) + 0.5f) / scale.x,
                     (static_cast<float>(h) + 0.5f) / scale.y};

    if constexpr (CLAMP)
    {
        center.x = __saturatef(center.x);
        center.y = __saturatef(center.y);
    }

    auto appendPrior = [&](float2 size, const uint32_t outputOffset)
    {
        if constexpr (CLAMP)
        {
            size.x = __saturatef(size.x);
            size.y = __saturatef(size.y);
        }
        if constexpr (NUM_TILES > 0)
        {
            for (uint32_t i = 0; i < NUM_TILES; i++)
            {
                // Center and size of ssd box is same regardless of batch size
                // Offset relative to starting image is different per batch
                // TODO - can we encode that using center?
                output[outputIdx + i * outputsPerBatch + outputOffset].m_center = center;
                output[outputIdx + i * outputsPerBatch + outputOffset].m_size = size;
                output[outputIdx + i * outputsPerBatch + outputOffset].m_imageSize = ushort2{static_cast<unsigned short>(modelInputSize.x), static_cast<unsigned short>(modelInputSize.y)};
                output[outputIdx + i * outputsPerBatch + outputOffset].m_offset = offsets[i];
            }
        }
        if constexpr (USE_SCALED_IMAGE)
        {
            output[outputIdx + NUM_TILES * outputsPerBatch + outputOffset].m_center = center;
            output[outputIdx + NUM_TILES * outputsPerBatch + outputOffset].m_size = size;
            output[outputIdx + NUM_TILES * outputsPerBatch + outputOffset].m_imageSize = imageSize;
            output[outputIdx + NUM_TILES * outputsPerBatch + outputOffset].m_offset = ushort2{0, 0};
        }
    };

    // Small sized square box
    appendPrior(float2{boxSizeMin / modelInputSize.x, boxSizeMin / modelInputSize.y}, 0);
    // big sized square box
    const float boxSize = sqrtf(boxSizeMin * boxSizeMax);
    appendPrior(float2{boxSize / modelInputSize.x, boxSize / modelInputSize.y}, 1);

    // change aspect ratio of small sized box
    float ratio = sqrtf(aspectRatioMin);
    appendPrior(float2{(boxSizeMin / modelInputSize.x) * ratio, (boxSizeMin / modelInputSize.y) / ratio}, 2);
    appendPrior(float2{(boxSizeMin / modelInputSize.x) / ratio, (boxSizeMin / modelInputSize.y) * ratio}, 3);

    ratio = sqrtf(aspectRatioMax);
    appendPrior(float2{(boxSizeMin / modelInputSize.x) * ratio, (boxSizeMin / modelInputSize.y) / ratio}, 4);
    appendPrior(float2{(boxSizeMin / modelInputSize.x) / ratio, (boxSizeMin / modelInputSize.y) * ratio}, 5);
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
SSDGridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::SSDGridPrior(void)
{
    if constexpr (NUM_TILES > 0)
    {
        cudaSafeCall(cudaMalloc(&m_dOffsets, sizeof(*m_dOffsets) * NUM_TILES));
        cudaSafeCall(cudaMallocHost(&m_hOffsets, sizeof(*m_hOffsets) * NUM_TILES));
    }
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
SSDGridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::~SSDGridPrior()
{
    if constexpr (NUM_TILES > 0)
    {
        cudaSafeCall(cudaFree(m_dOffsets));
        cudaSafeCall(cudaFreeHost(m_hOffsets));
    }
    cudaSafeCall(cudaFree(m_dSSDGridPriors));
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
bool SSDGridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::generate(const ushort2 &modelInputSize,
                                                                const ushort2 &imageSize,
                                                                const std::array<ushort2, NUM_TILES> &tileOffsets,
                                                                cudaStream_t cudaStream)
{
    if ((modelInputSize.x != m_modelInputSize.x) || (modelInputSize.y != m_modelInputSize.y) ||
        (imageSize.x != m_imageSize.x) || (imageSize.y != m_imageSize.y))
    {
        m_ssdSpecs.clear();
        m_ssdSpecs.emplace_back(std::make_unique<SSDSpec>(80, modelInputSize.x /  8,  8,  32,  64, 2, 3));
        m_ssdSpecs.emplace_back(std::make_unique<SSDSpec>(40, modelInputSize.x / 16, 16,  64, 128, 2, 3));
        m_ssdSpecs.emplace_back(std::make_unique<SSDSpec>(20, modelInputSize.x / 32, 32, 128, 256, 2, 3));
        m_ssdSpecs.emplace_back(std::make_unique<SSDSpec>(10, modelInputSize.x / 64, 64, 256, 512, 2, 3));
        if constexpr (NUM_TILES > 0)
        {
            for (size_t i = 0; i < NUM_TILES; i++)
            {
                m_hOffsets[i] = tileOffsets[i];
            }
            cudaSafeCall(cudaMemcpyAsync(m_dOffsets, m_hOffsets, NUM_TILES * sizeof(ushort2), cudaMemcpyHostToDevice, cudaStream));
        }
        cudaSafeCall(cudaFree(m_dSSDGridPriors));
        m_count = 0;
        for (const auto &s : m_ssdSpecs)
        {
            m_count += s->m_featureMapSizeH * s->m_featureMapSizeW;
        }
        m_count *= boxesPerSize; // 6 boxes per feature map size : min, max, 4 aspect ratios

        cudaSafeCall(cudaMalloc(&m_dSSDGridPriors, sizeof(SSDBoxCenterForm) * m_count * (NUM_TILES + NUM_SCALED_IMAGES)));
        uint32_t offset = 0;
        for (const auto &s :m_ssdSpecs)
        {
            const dim3 blockDim(8, 8);
            const dim3 gridDim(iDivUp(s->m_featureMapSizeW, blockDim.x),
                               iDivUp(s->m_featureMapSizeH, blockDim.y));
            generateSSDGridPriorsKernel<NUM_TILES, USE_SCALED_IMAGE, CLAMP><<<gridDim, blockDim, 0, cudaStream>>>(m_dSSDGridPriors + offset,
                                                                                                                  float2{static_cast<float>(modelInputSize.x), static_cast<float>(modelInputSize.y)},
                                                                                                                  imageSize,
                                                                                                                  s->m_featureMapSizeH,
                                                                                                                  s->m_featureMapSizeW,
                                                                                                                  s->m_shrinkage,
                                                                                                                  s->m_boxSizeMin,
                                                                                                                  s->m_boxSizeMax,
                                                                                                                  s->m_aspectRatioMin,
                                                                                                                  s->m_aspectRatioMax,
                                                                                                                  m_count,
                                                                                                                  m_dOffsets);
            cudaSafeCall(cudaGetLastError());
                                                                                                
            // Keep track of where this set of feature maps needs to go
            // in the global buffer                                                                                     
            offset += (s->m_featureMapSizeH * s->m_featureMapSizeW) * 6;
        }
        m_modelInputSize = modelInputSize;
        m_imageSize = imageSize;
        return true;
    }
    return false;
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
const tcb::span<const SSDBoxCenterForm> SSDGridPrior<NUM_TILES, USE_SCALED_IMAGE, CLAMP>::getOutput(void) const
{
    return tcb::span<SSDBoxCenterForm>(m_dSSDGridPriors, m_count * (NUM_TILES + NUM_SCALED_IMAGES));
}

template class SSDGridPrior<0, true, true>;
template class SSDGridPrior<4, false, true>;
template class SSDGridPrior<4, true, true>;
template class SSDGridPrior<9, false, true>;
template class SSDGridPrior<9, true, true>;