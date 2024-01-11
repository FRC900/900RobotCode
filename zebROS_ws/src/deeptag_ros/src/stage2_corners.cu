#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/stage2_corners.h"
#include "deeptag_ros/cuda_utils.h"
#include <iostream>

__global__ static void calculateStage2CornerKernel(float2 *output,
                                                   const float *input,
                                                   const GridPriorValue *priors,
                                                   const uint32_t length,
                                                   const float centerVariance)
{
	const uint32_t idx = threadIdx.x;

    if (idx >= length)
    {
        return;
    }
    //printf("idx = %d, input[idx] = %f %f, centerVariance = %f, priors[idx] = %f %f\n", idx, input[idx], input[idx + length], centerVariance, priors[idx].x, priors[idx].y);
    output[idx].x = (input[idx] * centerVariance + priors[idx].m_scale.x) * priors[idx].m_imageSize.x + priors[idx].m_offset.x;
    output[idx].y = (input[idx + length] * centerVariance + priors[idx].m_scale.y) * priors[idx].m_imageSize.x + priors[idx].m_offset.y;
}

Stage2Corners::Stage2Corners(void)
{
    cudaSafeCall(cudaMalloc(&m_dResults, 4 * sizeof(float2)));
    cudaSafeCall(cudaMallocHost(&m_hResults, 4 * sizeof(float2)));
    cudaSafeCall(cudaEventCreate(&m_dataReadyEvent));
}

Stage2Corners::~Stage2Corners()
{
    cudaSafeCall(cudaFree(m_dResults));
    cudaSafeCall(cudaFreeHost(m_hResults));
    cudaSafeCall(cudaEventDestroy(m_dataReadyEvent));
}

void Stage2Corners::compute(const float *input,
                            const tcb::span<const GridPriorValue> &priors,
                            const float centerVariance,
                            cudaStream_t cudaStream)
{
    calculateStage2CornerKernel<<<1, priors.size(), 0, cudaStream>>>(m_dResults, input, priors.data(), priors.size(), centerVariance);
    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaMemcpyAsync(m_hResults, m_dResults, priors.size() * sizeof(float2), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaEventRecord(m_dataReadyEvent, cudaStream));
}

const tcb::span<const float2> Stage2Corners::getDeviceOutput(void)
{
    cudaEventSynchronize(m_dataReadyEvent);
    return tcb::span<const float2>(m_dResults, 4);
}

const tcb::span<const float2> Stage2Corners::getHostOutput(void)
{
    cudaEventSynchronize(m_dataReadyEvent);
    return tcb::span<const float2>(m_hResults, 4);
}
