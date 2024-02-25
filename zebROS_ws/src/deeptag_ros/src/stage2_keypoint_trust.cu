#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/stage2_keypoint_trust.h"
#include <cstdio>
#include <cooperative_groups.h>
#include <iostream>

/*
    This version uses n/2 threads --
    it performs the first level of reduction when reading from global memory.
*/
namespace cg = cooperative_groups;
// Utility class used to avoid linker errors with extern
// unsized shared memory arrays with templated type
template <class T>
struct SharedMemory
{
    __device__ inline operator T *()
    {
        extern __shared__ int __smem[];
        return (T *)__smem;
    }

    __device__ inline operator const T *() const
    {
        extern __shared__ int __smem[];
        return (T *)__smem;
    }
};

__global__ void reduce3(float *g_odata, const Stage2Keypoint *keypoint, unsigned int n)
{
    // Handle to thread block group
    cg::thread_block cta = cg::this_thread_block();
    float *sdata = SharedMemory<float>();

    // perform first level of reduction,
    // reading from global memory, writing to shared memory
    const uint32_t tid = threadIdx.x;
    const uint32_t i = blockIdx.x * (blockDim.x * 2) + threadIdx.x;

    auto mySum = (i < n) ? keypoint[i].m_scoreCand : 0;
    //printf("i = %d, scoreCand = %f\n", i, keypoint[i].m_scoreCand);

    if ((i + blockDim.x) < n)
    {
        mySum += keypoint[i + blockDim.x].m_scoreCand;
    }
    //printf("i = %d, scoreCand = %f\n", i + blockDim.x, keypoint[i + blockDim.x].m_scoreCand);

    sdata[tid] = mySum;
    cg::sync(cta);

    // do reduction in shared mem
    for (unsigned int s = (blockDim.x / 2); s > 0; s >>= 1)
    {
        if (tid < s)
        {
            sdata[tid] = mySum = mySum + sdata[tid + s];
        }

        cg::sync(cta);
    }

    // write result for this block to global mem
    if (tid == 0)
    {
        g_odata[blockIdx.x] = mySum;
    }
    cg::sync(cta);
    // Do final sum across blocks
    if (i == 0)
    {
        for (size_t i = 1; i < gridDim.x; i++)
        {
            g_odata[0] += g_odata[i];
        }
    }
}

Stage2KeypointTrust::Stage2KeypointTrust(void)
{
    cudaSafeCall(cudaMallocHost(&m_hResultPtr, sizeof(float)));
}

Stage2KeypointTrust::~Stage2KeypointTrust()
{
    cudaSafeCall(cudaFreeHost(m_hResultPtr));
}

bool Stage2KeypointTrust::check(const tcb::span<const Stage2Keypoint> &stage2Keypoints, cudaStream_t cudaStream)
{
    if (stage2Keypoints.size() == 0)
    {
        return false;
    }
    // when there is only one warp per block, we need to allocate two warps
    // worth of shared memory so that we don't index shared memory out of bounds
    // Also handle the case where just 1 keypoint is detected - force blocks
    // to be at least 1 in that case
    constexpr int threads = 32;
    const int blocks = std::max(1U, iDivUp(stage2Keypoints.size() / 2, threads));
    if (m_dReduceBufferSize < blocks)
    {
        if (m_dReduceBuffer)
        {
            cudaSafeCall(cudaFreeAsync(m_dReduceBuffer, cudaStream));
        }
        cudaSafeCall(cudaMallocAsync(&m_dReduceBuffer, sizeof(float) * blocks, cudaStream));
        m_dReduceBufferSize = blocks;
    }
    constexpr int smemSize = (threads <= 32) ? 2 * threads * sizeof(float) : threads * sizeof(float);
    // std::cout << "threads = " << threads << " blocks = " << blocks << " m_dReduceBufferSize = " << m_dReduceBufferSize << " smemSize = " << smemSize << std::endl;
    reduce3<<<blocks, threads, smemSize, cudaStream>>>(m_dReduceBuffer, stage2Keypoints.data(), stage2Keypoints.size());
    cudaSafeCall(cudaGetLastError());
    cudaSafeCall(cudaMemcpyAsync(m_hResultPtr, m_dReduceBuffer, sizeof(float), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaStreamSynchronize(cudaStream));
    return (*m_hResultPtr / stage2Keypoints.size()) > 0.6;
}
