#include <iostream>
#include "deeptag_ros/stage1_grid.h"
#include "deeptag_ros/confidence_filter.h"
#include "deeptag_ros/cuCompactor.cuh"
#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/ssd_box.h"
#include "deeptag_ros/ssd_tag_keypoint.h"

template <class OUTPUT, class GRID_DIM, class PREDICATE>
ConfidenceFilter<OUTPUT, GRID_DIM, PREDICATE>::ConfidenceFilter(const size_t blockSize)
    : m_blockSize(blockSize)
{
    // Output size is calculated as cumulative size up to the last block plus the
    // number of outputs from the final block.
    cudaSafeCall(cudaMallocHost(&m_hOutputCountPtr, sizeof(*m_hOutputCountPtr)));
    cudaSafeCall(cudaEventCreate(&m_outputReadyEvent));
}

template <class OUTPUT, class GRID_DIM, class PREDICATE>
ConfidenceFilter<OUTPUT, GRID_DIM, PREDICATE>::~ConfidenceFilter(void)
{
    cudaSafeCall(cudaFree(m_dOutput));
    cudaSafeCall(cudaFreeHost(m_hOutputCountPtr));
    cudaSafeCall(cudaFree(m_dBlocksCount));
    cudaSafeCall(cudaFree(m_dBlocksOffset));

    cudaSurfaceFormatMode(cudaEventDestroy(m_outputReadyEvent));
}

template <class OUTPUT, class GRID_DIM, class PREDICATE>
bool ConfidenceFilter<OUTPUT, GRID_DIM, PREDICATE>::reallocBuffers(const size_t count, cudaStream_t cudaStream)
{
    if (count > m_count)
    {
        cudaSafeCall(cudaFreeAsync(m_dOutput, cudaStream));
        cudaSafeCall(cudaFreeAsync(m_dBlocksCount, cudaStream));
        cudaSafeCall(cudaFreeAsync(m_dBlocksOffset, cudaStream));
        cudaSafeCall(cudaMallocAsync(&m_dOutput, sizeof(OUTPUT) * count, cudaStream));

        const uint32_t numBlocks = iDivUp(count, m_blockSize);
        cudaSafeCall(cudaMallocAsync(&m_dBlocksCount, sizeof(*m_dBlocksCount) * numBlocks, cudaStream));
        // Add 1 to hold the overall total valid count
        cudaSafeCall(cudaMallocAsync(&m_dBlocksOffset, sizeof(*m_dBlocksOffset) * (numBlocks + 1), cudaStream));

        m_count = count;
        return true;
    }
    return false;
}

template <class OUTPUT, class GRID_DIM, class PREDICATE>
void ConfidenceFilter<OUTPUT, GRID_DIM, PREDICATE>::detect(const std::array<const float *, 3> &inputs,
                                                           const GRID_DIM gridDims,
                                                           const float centerVariance,
                                                           const float sizeVariance,
                                                           const float confidence,
                                                           cudaStream_t cudaStream,
                                                           const bool forceCudaGraphRegen)
{
    m_predicate.m_confidence = confidence;
    const bool buffersResized = reallocBuffers(gridDims.size(), cudaStream);
    if (buffersResized || forceCudaGraphRegen)
    {
        std::cout << "Keypoint detector : generating CUDA graph buffersResize = " <<  buffersResized << " forceCudaGraphRegen = " << forceCudaGraphRegen << std::endl;
        cudaSafeCall(cudaStreamSynchronize(cudaStream));
        cudaSafeCall(cudaStreamBeginCapture(cudaStream, cudaStreamCaptureModeGlobal));
        cuCompactor::compact(inputs,
                             m_dOutput,
                             m_hOutputCountPtr,
                             gridDims.data(),
                             centerVariance,
                             sizeVariance,
                             m_count,
                             m_predicate,
                             m_blockSize,
                             m_dBlocksCount,
                             m_dBlocksOffset,
                             cudaStream);
        cudaSafeCall(cudaStreamEndCapture(cudaStream, &m_cudaGraph));

        cudaSafeCall(cudaGraphInstantiate(&m_cudaGraphInstance, m_cudaGraph, NULL, NULL, 0));
        cudaSafeCall(cudaStreamSynchronize(cudaStream));
    }
    cudaSafeCall(cudaGraphLaunch(m_cudaGraphInstance, cudaStream));
    cudaSafeCall(cudaEventRecord(m_outputReadyEvent, cudaStream));
}

// Have to wait until a queued async D2H memcpy into m_hOutputCountPtr finishes
// before allowing host to get the number of output which passed filtering
template <class OUTPUT, class GRID_DIM, class PREDICATE>
tcb::span<const OUTPUT> ConfidenceFilter<OUTPUT, GRID_DIM, PREDICATE>::getOutput()
{
    cudaSafeCall(cudaEventSynchronize(m_outputReadyEvent));
    return tcb::span<const OUTPUT>(m_dOutput, m_hOutputCountPtr[0]);
}

// Predicate to filter on a single float confidence value
__device__ bool Stage1Predicate::operator()(const float *f, const int index, const int length) const
{
    //printf("f[%u] = %f, m_confidence = %f\n", index, f[index], m_confidence);
    return f[index] > m_confidence;
}

// Predicate to filter on the max of two confidences
__device__ bool Stage2Predicate::operator()(const float *f, const int index, const int length) const
{
    return max(f[index], f[index + length]) > m_confidence;
}

#include "deeptag_ros/grid_prior_value.h"
template class ConfidenceFilter<Stage1Grid<1>, const tcb::span<const GridPriorValue> &, Stage1Predicate>;
template class ConfidenceFilter<Stage1Grid<4>, const tcb::span<const GridPriorValue> &, Stage1Predicate>;
template class ConfidenceFilter<Stage1Grid<5>, const tcb::span<const GridPriorValue> &, Stage1Predicate>;
template class ConfidenceFilter<Stage1Grid<9>, const tcb::span<const GridPriorValue> &, Stage1Predicate>;
template class ConfidenceFilter<Stage1Grid<10>, const tcb::span<const GridPriorValue> &, Stage1Predicate>;
template class ConfidenceFilter<SSDTagKeypoint, const tcb::span<const SSDBoxCenterForm> &, Stage1Predicate>;

#include "deeptag_ros/stage2_keypoint.h"
template class ConfidenceFilter<Stage2Keypoint, const tcb::span<const GridPriorValue> &, Stage2Predicate>;