// Merge similar sets of INPUTs into a single output, where the 
// output's values are the confidence-weighted combination of
// the various input fields
//
// Kind of like non-max suppression, but also better(?) since it
// merges detections into an averaged midpoint rather than just
// rejecting the non-max inference results

#include <cstdio>
#include <iostream>
#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/suppress_and_average_keypoints.h"

template <class INPUT, class OUTPUT>
__global__ void computeGroupMembership(OUTPUT *groups,
                                       uint32_t *groupCount,
                                       bool *scratch,
                                       const INPUT *input,
                                       const uint32_t inputCount,
                                       const float sigma,
                                       const float min_cos)
{
	const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
	const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if ((x >= inputCount) || (y >= inputCount))
    {
        return;
    }
    // Only need one diagonal of the matrix calculated
    // It is symmetrical, but also the other diagonal isn't ever referenced
    #if 1
    if (y < x)
    {
        return;
    }
    #endif

    bool result = true;
    //printf ("x = %d, y = %d, index = %d\n ", x, y, x * inputCount + y);
    if (x != y)
    {
        result = input[x].check(input[y], x, y, sigma, min_cos);
    }

    //printf ("x = %d, y = %d, index = %d, result = %d, sigma = %f, min_cos = %f\n", x, y, x * inputCount + y, result, sigma, min_cos);
    scratch[x * inputCount + y] = result;

    __syncthreads();

    if ((x == 0) && (y == 0))
    {
        // Keep track of which indexes have already been added
        // to a group. The next one not added is the index of
        // a start of a new group.
        bool *used = &scratch[inputCount * inputCount];
        for (int32_t i = 0; i < inputCount; i++)
        {
            used[i] = false;
        }
        *groupCount = 0;
        for (int32_t i = 0; i < inputCount; i++)
        {
            // Starting with the next first unused index
            // group together all of the indexes that are 
            // in the same group as that first index
            if (!used[i])
            {
                groups[*groupCount].reset();
                for (int32_t j = i; j < inputCount; j++)
                {
                    if (scratch[i * inputCount + j])
                    {
                        groups[*groupCount].append(input[j]);
                        used[j] = true;
                    }
                }
                groups[*groupCount].end();
                *groupCount += 1;
            }
        }
    }
}

template <class INPUT, class OUTPUT>
SuppressAndAverageKeypoints<INPUT, OUTPUT>::SuppressAndAverageKeypoints()
{
    cudaSafeCall(cudaMallocHost(&m_hOutputLengthPtr, sizeof(*m_hOutputLengthPtr)));
    cudaSafeCall(cudaMalloc(&m_dOutputLengthPtr, sizeof(*m_dOutputLengthPtr)));
    cudaSafeCall(cudaEventCreate(&m_outputReadyEvent));
}

template <class INPUT, class OUTPUT>
SuppressAndAverageKeypoints<INPUT, OUTPUT>::~SuppressAndAverageKeypoints()
{
    cudaSafeCall(cudaFreeHost(m_hOutputLengthPtr));
    cudaSafeCall(cudaFree(m_dOutput));
    cudaSafeCall(cudaFree(m_dGroupMatrix));
}

template <class INPUT, class OUTPUT>
void SuppressAndAverageKeypoints<INPUT, OUTPUT>::compute(const tcb::span<const INPUT> &input,
                                                         const float sigma,
                                                         const float min_cos,
                                                         cudaStream_t cudaStream)
{
    constexpr int32_t blockSize = 8;
    if (input.size() > m_inputCount)
    {
        cudaSafeCall(cudaFreeAsync(m_dOutput, cudaStream));
        cudaSafeCall(cudaFreeAsync(m_dGroupMatrix, cudaStream));
        cudaSafeCall(cudaFreeHost(m_hOutput));
        cudaSafeCall(cudaMallocAsync(&m_dOutput, sizeof(OUTPUT) * input.size(), cudaStream));
        // This wil not call the contstructor for each OUTPUT entry,
        // but we don't really care - the data only gets set from
        // a memcpy from the device anyway
        cudaSafeCall(cudaMallocHost(&m_hOutput, sizeof(OUTPUT) * input.size()));
        // Allocate an nxn grid to mark same/not-same group from input i to input j.
        // Add a final row to use in combining groups together
        cudaSafeCall(cudaMallocAsync(&m_dGroupMatrix, sizeof(bool) * (input.size() + 1) * input.size(), cudaStream));
        m_inputCount = input.size();

    }
    if (input.size() > 0)
    {
        const dim3 blockDim(blockSize, blockSize);
        const dim3 gridDim(iDivUp(input.size(), blockDim.x), iDivUp(input.size(), blockDim.y));
        computeGroupMembership<INPUT, OUTPUT><<<gridDim, blockDim, 0, cudaStream>>>(m_dOutput, m_dOutputLengthPtr, m_dGroupMatrix, input.data(), input.size(), sigma, min_cos);
        cudaSafeCall(cudaGetLastError());
        cudaSafeCall(cudaMemcpyAsync(m_hOutputLengthPtr, m_dOutputLengthPtr, sizeof(*m_hOutputLengthPtr), cudaMemcpyDeviceToHost, cudaStream));
        // inputCount is small enough that it seems quicker to unconditionally queue this up here
        // rather than wait for output count to get back to the host and then queue up a shorter sized copy
        cudaSafeCall(cudaMemcpyAsync(m_hOutput, m_dOutput, m_inputCount * sizeof(OUTPUT), cudaMemcpyDeviceToHost, cudaStream));
    }
    else
    {
        *m_hOutputLengthPtr = 0;
    }
    cudaSafeCall(cudaEventRecord(m_outputReadyEvent, cudaStream));

#if 0
    bool *hGroupMatrix = new bool[inputCount * inputCount];
    cudaSafeCall(cudaMemcpyAsync(hGroupMatrix, m_dGroupMatrix, sizeof(bool) * inputCount * inputCount, cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaStreamSynchronize(cudaStream));
    for (int r = 0; r < inputCount; r++)
    {
        for (int c = 0; c < inputCount; c++)
        {
            std::cout << hGroupMatrix[r * inputCount + c] << " ";
        }
        std::cout << std::endl;
    }
#endif
}


template <class INPUT, class OUTPUT>
const tcb::span<const OUTPUT> SuppressAndAverageKeypoints<INPUT, OUTPUT>::getOutput()
{
    cudaSafeCall(cudaEventSynchronize(m_outputReadyEvent));

    return tcb::span<const OUTPUT>(m_hOutput, *m_hOutputLengthPtr);
}

#include "deeptag_ros/stage1_grid.h"
#include "deeptag_ros/ssd_tag_keypoint.h"
#include "deeptag_ros/stage1_grid_group.h"
#include "deeptag_ros/stage1_ssd_group.h"
#include "deeptag_ros/stage2_keypoint.h"
#include "deeptag_ros/stage2_keypoint_group.h"
template class SuppressAndAverageKeypoints<Stage1Grid<1>, Stage1GridGroup<1>>;
template class SuppressAndAverageKeypoints<Stage1Grid<4>, Stage1GridGroup<4>>;
template class SuppressAndAverageKeypoints<Stage1Grid<5>, Stage1GridGroup<5>>;
template class SuppressAndAverageKeypoints<Stage1Grid<9>, Stage1GridGroup<9>>;
template class SuppressAndAverageKeypoints<Stage1Grid<10>, Stage1GridGroup<10>>;
template class SuppressAndAverageKeypoints<SSDTagKeypoint, Stage1SSDGroup>;
template class SuppressAndAverageKeypoints<Stage2Keypoint, Stage2KeypointGroup>;
