#ifndef DECODER_SOFTMAX_
#define DECODER_SOFTMAX_

#include <cuda_runtime.h>
#include "decoder_softmax_results.h"
#include "span.hpp"


// Class to handle DecoderSoftmax ops.
// Input is result from model, all bg scores (h x w) followed by all fg scores
// Calculate the pairwise softmax of each grid location, store it in m_dResults
class DecoderSoftmax
{
public:
    DecoderSoftmax(void);

    DecoderSoftmax(const DecoderSoftmax &other) = delete;
    DecoderSoftmax(DecoderSoftmax &&other) noexcept = delete;

    DecoderSoftmax &operator=(const DecoderSoftmax &other) = delete;
    DecoderSoftmax &operator=(DecoderSoftmax &&other) noexcept = delete;

    virtual ~DecoderSoftmax();
    void compute(const float *input, const uint32_t count, cudaStream_t cudaStream);
    const tcb::span<const float> getOutput(void) const;

private:
    void allocateResults(const uint32_t count);
    float *m_dResults{nullptr};
    uint32_t m_resultLength{0};
};
#endif