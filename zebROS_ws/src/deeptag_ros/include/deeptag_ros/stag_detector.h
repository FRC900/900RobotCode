#ifndef STAG_DETECTOR_INC__
#define STAG_DETECTOR_INC__

#include <stddef.h>                          // for size_t
#include <stdint.h>                          // for uint32_t
#include <array>                             // for array
#include <memory>                            // for unique_ptr:w
#include <string>                            // for string
#include <vector>                            // for vector
#include "confidence_filter.h"               // for Stage1Predicate, Confide...
#include "driver_types.h"                    // for CUevent_st, cudaEvent_t
#include "grid_prior.h"                      // for GridPrior
#include "softmax.h"                         // for Softmax
#include "span.hpp"                          // for span
#include "ssd_grid_prior.h"                  // for SSDGridPrior
#include "suppress_and_average_keypoints.h"  // for SuppressAndAverageKeypoints
#include "tag_detect_info.h"                 // for TagDetectInfo
#include "vector_types.h"                    // for ushort2
template <size_t NUM_TILES, bool USE_SCALED_IMAGE> class DetectionEngine;
class GpuImageWrapper;
class SSDTagKeypoint;  // lines 16-16
class Stage1SSDGroup;  // lines 18-18
class Timings;  // lines 20-20
namespace cv { class Mat; }
struct GridPriorValue;
struct SSDBoxCenterForm;  // lines 22-22
template <uint32_t BATCH_SIZE> class Stage1GridGroup;  // lines 17-17
template <uint32_t BATCH_SIZE> struct Stage1Grid;  // lines 21-21

template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
class STagDetector
{
public:
    explicit STagDetector(Timings &timing);
    STagDetector(const STagDetector &other) = delete;
    STagDetector(STagDetector &&other) noexcept = delete;

    STagDetector &operator=(const STagDetector &other) = delete;
    STagDetector &operator=(STagDetector &&other) noexcept = delete;

    virtual ~STagDetector();

    void initEngine(const std::string &modelPath, const std::string &onnxModelFilename);
    std::vector<TagDetectInfo> detectTags(const cv::Mat &cpuImg);

    const std::vector<std::vector<GpuImageWrapper>> &getDetectInputs(void) const;

    cudaStream_t getCudaStream(void);

    void   setCornerMinCenterScore(const double cornerMinCenterScore);
    void   setSSDMinCenterScore(const double cornerMinCenterScore);
    void   setGridGrouperSigma(const int gridGrouperSigma);
    void   setSSDGrouperSigma(const int ssdGrouperSigma);
    double getCornerMinCenterScore(void) const;
    double getSSDMinCenterScore(void) const;
    int    getGridGrouperSigma(void) const;
    int    getSSDGrouperSigma(void) const;

private:
    ushort2 generateInputs(const cv::Mat &cpuImg);
    void generatePriors(const ushort2 &imgSize);
    void runSoftmax();
    void runConfidenceFilter(const ushort2 &imgSize);
    void runGroupers(const ushort2 &imgSize);
    cudaStream_t m_ssdCudaStream;
    cudaEvent_t  m_engineDoneCudaEvent;
    cudaEvent_t  m_ssdDoneCudaEvent;

    static constexpr size_t NUM_SCALED_IMAGES{USE_SCALED_IMAGE ? 1 : 0};
    Timings &m_timing;
    std::unique_ptr<DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>> m_detectEngine;
    std::vector<std::vector<GpuImageWrapper>> m_detectInputs;
    GridPrior<NUM_TILES, USE_SCALED_IMAGE, true> m_gridPrior;
    Softmax<NUM_TILES + NUM_SCALED_IMAGES> m_gridSoftmax;
    ConfidenceFilter<Stage1Grid<NUM_TILES + NUM_SCALED_IMAGES>, const tcb::span<const GridPriorValue> &, Stage1Predicate> m_cornerConfidenceFilter{1024};
    SuppressAndAverageKeypoints<Stage1Grid<NUM_TILES + NUM_SCALED_IMAGES>, Stage1GridGroup<NUM_TILES + NUM_SCALED_IMAGES>> m_gridGrouper;

    SSDGridPrior<NUM_TILES, USE_SCALED_IMAGE, true> m_ssdGridPrior;
    Softmax<NUM_TILES + NUM_SCALED_IMAGES> m_ssdSoftmax;
    ConfidenceFilter<SSDTagKeypoint, const tcb::span<const SSDBoxCenterForm> &, Stage1Predicate> m_ssdConfidenceFilter{1024};
    SuppressAndAverageKeypoints<SSDTagKeypoint, Stage1SSDGroup> m_ssdGrouper;

    bool m_regenCudaGraph{true};

    double m_cornerMinCenterScore{0.2f};
    double m_ssdMinCenterScore{0.08f};
    int    m_gridGrouperSigma{8};
    int    m_ssdGrouperSigma{8};
};

#endif
