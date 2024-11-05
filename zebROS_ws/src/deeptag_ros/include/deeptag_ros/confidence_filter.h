#ifndef CONFIDENCE_FILTER_INC__
#define CONFIDENCE_FILTER_INC__

#include "span.hpp"

#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif

// This takes the raw outputs of inference, filters out values
// with confidence that is too low, and saves the remaining
// results in a more friendly structure (compared to flat arrays
// of floats coming out of the model)
// Part of the "saves the remaining results" is converting from
// model-relative normalized coordinates into input-image relative
// pixel coordinates.  This is done using the scale and shift values
// in the GRID_DIM object particular to each type of filtering.

template <class OUTPUT, class GRID_DIM, class PREDICATE>
class ConfidenceFilter
{
public:
    explicit ConfidenceFilter(const size_t blockSize);

    ConfidenceFilter(const ConfidenceFilter &other) = delete;
    ConfidenceFilter(ConfidenceFilter &&other) noexcept = delete;

    ConfidenceFilter &operator=(const ConfidenceFilter &other) = delete;
    ConfidenceFilter &operator=(ConfidenceFilter &&other) noexcept = delete;

    virtual ~ConfidenceFilter();

    // Takes array of input T's, where T has a .confidence field.
    // Returns a compacted array of Ts with the T's with confidence 
    // higher than the function arg confidence
    void detect(const std::array<const float *, 3> &inputs,
                GRID_DIM gridDims,
                const float centerVariance,
                const float scaleVariance,
                const float confidence,
                cudaStream_t cudaStream, 
                const bool forceCudaGraphRegen);

    tcb::span<const OUTPUT> getOutput();

private:
    bool reallocBuffers(const size_t length, cudaStream_t cudaStream);
    OUTPUT *m_dOutput{nullptr};
    cudaEvent_t m_outputReadyEvent;
    cudaGraph_t m_cudaGraph;
    cudaGraphExec_t m_cudaGraphInstance;
    uint32_t *m_hOutputCountPtr{nullptr};
    size_t m_count{0};
    size_t m_blockSize{0};

    // Scratch space needed for cuCompactor. Reallocated only when needed because
    // of changes in image size (i.e. hopefully never for an input video stream)
    uint32_t *m_dBlocksCount{};
    uint32_t *m_dBlocksOffset{};

    // functor used to test confidence of predictions
    PREDICATE m_predicate{0};
};

// Predicate to filter on a single float confidence value
class Stage1Predicate
{
public:
    explicit Stage1Predicate(const float confidence)
        : m_confidence{confidence}
    {}
    __device__ bool operator()(const float *f, const int index, const int length) const;
    __device__ const char *getName() const { return "Stage1"; }
    float m_confidence;
};

// Predicate to filter on the best of 2 non-background scores
class Stage2Predicate
{
public:
    explicit Stage2Predicate(const float confidence)
        : m_confidence{confidence}
    {}
    __device__ bool operator()(const float *f, const int index, const int length) const;
    __device__ const char *getName() const { return "Stage2"; }
    float m_confidence;
};

#endif