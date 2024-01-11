#ifndef STAGE2_KEYPOINT_TRUST_INC__
#define STAGE2_KEYPOINT_TRUST_INC__
#include <cuda_runtime.h>
#include "stage2_keypoint.h"
#include "span.hpp"

// Given a list of keypoints, check that enough of them
// have a high enough confidence that we trust that the group
// of them is actually from a detection of a real tag.
// This is used to filter out cases where the 1st stage
// tag detection gives a bogus RoI fed into the 2nd stage
class Stage2KeypointTrust
{
public:
    Stage2KeypointTrust();

    Stage2KeypointTrust(const Stage2KeypointTrust &other) = delete;
    Stage2KeypointTrust(Stage2KeypointTrust &&other) noexcept = delete;

    Stage2KeypointTrust &operator=(const Stage2KeypointTrust &other) = delete;
    Stage2KeypointTrust &operator=(Stage2KeypointTrust &&other) noexcept = delete;

    virtual ~Stage2KeypointTrust();
    bool check(const tcb::span<const Stage2Keypoint> &stage2Keypoints, cudaStream_t cudaStream);
private:
    float *m_dReduceBuffer{nullptr};
    float *m_hResultPtr{nullptr};
    size_t m_dReduceBufferSize{0};
};
#endif