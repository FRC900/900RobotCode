#ifndef SOFTMAX_INC__
#define SOFTMAX_INC__

#include <stdint.h>        // for int32_t
#include "driver_types.h"  // for cudaStream_t
#include "span.hpp"        // for span

// Class to handle Softmax ops.
// Input is result from model, all bg scores (h x w) followed by all fg scores for grid
// For ssd, fg and bg scores are index i, i+1
// Calculate the pairwise softmax of each grid location, store it in m_dResults
template <uint32_t BATCH_SIZE>
class Softmax
{
public:
    Softmax(void);

    Softmax(const Softmax &other) = delete;
    Softmax(Softmax &&other) noexcept = delete;

    Softmax &operator=(const Softmax &other) = delete;
    Softmax &operator=(Softmax &&other) noexcept = delete;

    virtual ~Softmax();
    void computeSoftmax(const float *input, const uint32_t idx_mult, const uint32_t idx_add, const uint32_t length, cudaStream_t cudaStream);
    const tcb::span<const float> getOutput(void) const;

private:
    void allocateResults(const uint32_t count);
    float *m_dResults{nullptr};
    uint32_t m_resultCount{0};
};
#endif