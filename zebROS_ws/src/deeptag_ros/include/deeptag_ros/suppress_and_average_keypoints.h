#ifndef SUPPRESS_AND_AVERAGE_KEYPOINTS_H__
#define SUPPRESS_AND_AVERAGE_KEYPOINTS_H__

#include "span.hpp"

template <class INPUT, class OUTPUT>
class SuppressAndAverageKeypoints
{
public:
    SuppressAndAverageKeypoints();

    SuppressAndAverageKeypoints(const SuppressAndAverageKeypoints &other) = delete;
    SuppressAndAverageKeypoints(SuppressAndAverageKeypoints &&other) noexcept = delete;

    SuppressAndAverageKeypoints &operator=(const SuppressAndAverageKeypoints &other) = delete;
    SuppressAndAverageKeypoints &operator=(SuppressAndAverageKeypoints &&other) noexcept = delete;

    virtual ~SuppressAndAverageKeypoints();
    void compute(const tcb::span<const INPUT> &input,
                 const float sigma,
                 const float min_cos,
                 cudaStream_t cudaStream);
    const tcb::span<const OUTPUT> getOutput();

private:
    OUTPUT   *m_dOutput{nullptr};
    OUTPUT   *m_hOutput{nullptr};
    bool     *m_dGroupMatrix{nullptr};
    uint32_t  m_inputCount{0};
    uint32_t *m_dOutputLengthPtr{nullptr};
    uint32_t *m_hOutputLengthPtr{nullptr};
    cudaEvent_t m_outputReadyEvent;
};

#endif
