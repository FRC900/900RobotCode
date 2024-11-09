#ifndef STAGE2_CORNERS_INC__
#define STAGE2_CORNERS_INC__

#include "cuda_runtime.h"
#include "deeptag_ros/grid_prior_value.h"
#include "deeptag_ros/span.hpp"

class Stage2Corners
{
public:
    Stage2Corners(void);

    Stage2Corners(const Stage2Corners &other) = delete;
    Stage2Corners(Stage2Corners &&other) noexcept = delete;

    Stage2Corners &operator=(const Stage2Corners &other) = delete;
    Stage2Corners &operator=(Stage2Corners &&other) noexcept = delete;

    virtual ~Stage2Corners();
    void compute(const float *input, const tcb::span<const GridPriorValue> &priors, const float centerVariance, cudaStream_t cudaStream);
    const tcb::span<const float2> getDeviceOutput(void);
    const tcb::span<const float2> getHostOutput(void);

private:
    void allocateResults(const size_t length);
    float2 *m_dResults{nullptr};
    float2 *m_hResults{nullptr};
    cudaEvent_t m_dataReadyEvent;
};
#endif