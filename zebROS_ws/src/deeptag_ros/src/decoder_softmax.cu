#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/decoder_softmax.h"
#include <iostream>

__global__ static void calculateDecoderSoftmaxKernel(float *output, const float *input, const int32_t length)
{
	const uint32_t w = blockIdx.x * blockDim.x + threadIdx.x;
	const uint32_t h = blockIdx.y * blockDim.y + threadIdx.y;
    const uint32_t idx =  blockDim.x * gridDim.x * h + w;

    if (idx >= length)
    {
        return;
    }
    // Inputs are in three blocks, all the background values
    // followed by all the foreground 0 then 1 values
    const float in1 = input[idx];
    const float in2 = input[idx + length];
    const float in3 = input[idx + length * 2];

    // softmax is
    //  e^x[i] / sum(e^x[j])_over_all_js, normalized so that the sum
    //  of the soft max values is 1.
    // Note - net outputs are set up such that sum of e^i == 1 already.
    const float ein1 = expf(in1);
    const float ein2 = expf(in2);
    const float ein3 = expf(in3);

    const float unnorm1 = ein1 / (ein1 + ein2 + ein3);
    const float unnorm2 = ein2 / (ein1 + ein2 + ein3);
    const float unnorm3 = ein3 / (ein1 + ein2 + ein3);

    const float weight = unnorm1 + unnorm2 + unnorm3;

#if 0
    printf("softmax idx, %d, in1, %f, in2, %f, in3, %f, ein1, %f, ein2, %f, ein3, %f, unnorm1, %f, unnorm2, %f, unnorm3, %f, weight, %f\n",
           idx, in1, in2, in3, ein1, ein2, ein3, unnorm1, unnorm2, unnorm3, weight);
#endif

    //output[idx].results[0] = unnorm1 / weight;
    output[idx] = unnorm2 / weight;
    output[idx + length] = unnorm3 / weight;
}

DecoderSoftmax::DecoderSoftmax(void)
{
}

DecoderSoftmax::~DecoderSoftmax()
{
    cudaSafeCall(cudaFree(m_dResults));
}


void DecoderSoftmax::allocateResults(const uint32_t count)
{
    if (m_resultLength < count)
    {
        cudaSafeCall(cudaMalloc(&m_dResults, count * sizeof(float) * 2));
        m_resultLength = count;
    }
}

void DecoderSoftmax::compute(const float *input, const uint32_t count, cudaStream_t cudaStream)
{
    allocateResults(count);
    const dim3 blockDim(16, 16);
    const uint32_t squareSize = ceil(sqrt(count));
    const dim3 gridDim(iDivUp(squareSize, blockDim.x), iDivUp(iDivUp(count, squareSize), blockDim.y));
    //printf("===========================================\n");
    calculateDecoderSoftmaxKernel<<<gridDim, blockDim, 0, cudaStream>>>(m_dResults, input, count);
    cudaSafeCall(cudaGetLastError());
}

const tcb::span<const float> DecoderSoftmax::getOutput(void) const
{
    return tcb::span<const float>(m_dResults, m_resultLength);
}