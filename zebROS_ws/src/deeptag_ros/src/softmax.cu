#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/softmax.h"
#include <iostream>

// Take raw confidence values for the two binary classifiers 
// and turn them into softmax scores for the non-background class.
// This converts arbitrary values into a 0-1 confidence score where
// the sum of the 2 confidences per entry sums to 1.
//
// The output is aligned differently for the grid vs. ssd outputs.
// For the grid, it is all background values followed by all object values.
// That means the access pattern for background, foreground pairs requires
// reads from [x], [x + count], where count is the number of pairs per batch.
// SSD results are sequential, with background1, foreground1 adjacent followed
// by background2, foreground2, etc.
// Each batch result is separate, that is, the first batch starts this pattern at
// entry 0, the second at 2 * number of value pairs, and so on.

// Since this is a binary classifier, we just need to keep the foreground score
// The bg score is 1-foreground, but we don't really use it. Later tests in the code
// just check that the foreground score is greater than some threshold.

template <uint32_t BATCH_SIZE>
__global__ static void calculateSoftmaxKernel(float *output,
                                              const float *input,
                                              const uint32_t idx_mult,
                                              const uint32_t idx_add,
                                              const uint32_t countPerBatch)
{
    const uint32_t w = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t h = blockIdx.y * blockDim.y + threadIdx.y;
    const uint32_t idx = blockDim.x * gridDim.x * h + w;

    if (idx >= countPerBatch)
    {
        return;
    }

    for (uint32_t i = 0; i < BATCH_SIZE; i++)
    {
        const float in1 = input[2 * i * countPerBatch + idx_mult * idx];
        const float in2 = input[2 * i * countPerBatch + idx_mult * idx + idx_add];
#if 0
        if constexpr (BATCH_SIZE > 1)
            printf("i, %u, countPerBatch, %u, idx_mult, %u, idx_add, %u, idx, %u, i1, %u, i2, %u, in1, %f, in2, %f\n",
                   i, countPerBatch, idx_mult, idx_add, idx,
                   2 * i * countPerBatch + idx_mult * idx,
                   2 * i * countPerBatch  + idx_mult * idx + idx_add, in1, in2);
#endif

        // Softmax is
        //  e^x[i] / sum(e^x[j])_over_all_js, normalized so that the sum
        //  of the soft max values is 1.
        // Note - net outputs are set up such that sum of e^i == 1 already.
        const float ein1 = expf(in1);
        const float ein2 = expf(in2);

        const float unnorm2 = ein2 / (ein1 + ein2);

#if 0
        const float unnorm1 = ein1 / (ein1 + ein2);
        const float weight = unnorm1 + unnorm2;
        printf("softmax idx, %d, in1, %f, in2, %f, ein1, %f, ein2, %f, unnorm1, %f, unnorm2, %f, weight, %f\n",
                idx, in1, in2, ein1, ein2, unnorm1, unnorm2, weight);
#endif

        // idx is this thread's offset into that particular
        // set of batch outputs
        //printf("countPerBatch, %u, output, %u, %f\n", countPerBatch, idx, unnorm2);
        output[i * countPerBatch + idx] = unnorm2; // / weight;
    }
}

template <uint32_t BATCH_SIZE>
Softmax<BATCH_SIZE>::Softmax(void)
{
}

template <uint32_t BATCH_SIZE>
Softmax<BATCH_SIZE>::~Softmax()
{
    cudaSafeCall(cudaFree(m_dResults));
}

template <uint32_t BATCH_SIZE>
void Softmax<BATCH_SIZE>::allocateResults(const uint32_t count)
{
    if (m_resultCount != count)
    {
        cudaSafeCall(cudaFree(m_dResults));
        cudaSafeCall(cudaMalloc(&m_dResults, count * sizeof(float)));
        m_resultCount = count;
    }
}

template <uint32_t BATCH_SIZE>
void Softmax<BATCH_SIZE>::computeSoftmax(const float *input, const uint32_t idx_mult, const uint32_t idx_add, const uint32_t count, cudaStream_t cudaStream)
{
    allocateResults(count);
    const dim3 blockDim(16, 16);
    const auto countPerBatch = count / BATCH_SIZE;
    const uint32_t squareSize = sqrt(countPerBatch);
    // TODO - this occasionally gets 1 extra gridDim value
    const dim3 gridDim(iDivUp(squareSize, blockDim.x), iDivUp(iDivUp(countPerBatch, squareSize), blockDim.y));
    // std::cout << "gridDim = " << gridDim.x  << " " << gridDim.y << " " << gridDim.z << std::endl;
    calculateSoftmaxKernel<BATCH_SIZE><<<gridDim, blockDim, 0, cudaStream>>>(m_dResults, input, idx_mult, idx_add > 1 ? idx_add / BATCH_SIZE : idx_add, countPerBatch);
    cudaSafeCall(cudaGetLastError());
}

template <uint32_t BATCH_SIZE>
const tcb::span<const float> Softmax<BATCH_SIZE>::getOutput(void) const
{
    return tcb::span<const float>(m_dResults, m_resultCount);
}

template class Softmax<1>;
template class Softmax<4>;
template class Softmax<5>;
template class Softmax<9>;
template class Softmax<10>;