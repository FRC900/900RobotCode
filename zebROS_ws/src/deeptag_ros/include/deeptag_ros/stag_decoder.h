#ifndef STAG_DECODER_INC__
#define STAG_DECODER_INC__

#include <stddef.h>                          // for size_t
#include <array>                             // for array
#include <memory>                            // for unique_ptr
#include <opencv2/core/mat.hpp>              // for Mat
#include <opencv2/core/types.hpp>            // for Point2d
#include <string>                            // for string
#include <vector>                            // for vector
#include "deeptag_ros/decoded_tag.h"                     // for DecodedTag
#include "deeptag_ros/decoder_engine.h"                  // for DecoderEngine
#include "deeptag_ros/decoder_softmax.h"                 // for DecoderSoftmax
#include "deeptag_ros/grid_prior.h"                      // for GridPrior
#include "deeptag_ros/confidence_filter.h"               // for ConfidenceFilter
#include "deeptag_ros/span.hpp"                          // for span
#include "deeptag_ros/stage2_corners.h"                  // for Stage2Corners
#include "deeptag_ros/stage2_keypoint_trust.h"           // for Stage2KeypointTrust
#include "deeptag_ros/suppress_and_average_keypoints.h"  // for SuppressAndAverageKeypoints

class GpuImageWrapper;
struct GridPriorValue;
class Stage2Keypoint;
class Stage2KeypointGroup;
class Timings;
template <size_t GRID_SIZE> struct PointsAndIDs;

template <class MARKER_DICT, size_t GRID_SIZE>
class STagDecoder
{
public:
    STagDecoder(const MARKER_DICT &markerDict, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, Timings &timing);
    STagDecoder(const STagDecoder &other) = delete;
    STagDecoder(STagDecoder &&other) noexcept = delete;

    STagDecoder &operator=(const STagDecoder &other) = delete;
    STagDecoder &operator=(STagDecoder &&other) noexcept = delete;

    void initEngine(const std::string &modelPath, const std::string &onnxModelFilename);

    std::vector<std::array<DecodedTag<GRID_SIZE>, 2>> detectTags(const std::vector<std::vector<GpuImageWrapper>> &detectInputs,
                                                                 const std::vector<std::array<cv::Point2d, 4>> &rois);

    virtual ~STagDecoder() = default;

    void   setMinGridMatchRatio(const double minGridMatchRatio);
    double getMinGridMatchRatio(void) const;
    ushort2 getModelSize(void) const;
    cudaStream_t getCudaStream(void);

private:
    void runInference(std::vector<std::vector<Stage2KeypointGroup>> &stage2KeypointGroupss,
                      std::vector<std::array<float2, 4>> &stage2Corners,
                      const std::vector<std::vector<GpuImageWrapper>> &detectInputs,
                      const tcb::span<const std::array<cv::Point2d, 4>> &rois);
    void fillEmptyIds(PointsAndIDs<GRID_SIZE + 2> &orderedFineGridPointsIds,
                      const tcb::span<const Stage2KeypointGroup> &fineGridPointsWithIdsCandidates) const;

    const MARKER_DICT &m_markerDict;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
    Timings &m_timing;
    std::unique_ptr<DecoderEngine> m_decodeEngine;
    GridPrior<0, true, true> m_stage2CornerPrior;
    GridPrior<0, true, true> m_stage2GridPrior;
    DecoderSoftmax m_stage2DecoderSoftmax;

    ConfidenceFilter<Stage2Keypoint, const tcb::span<const GridPriorValue> &, Stage2Predicate> m_confidenceFilter{1024};
    Stage2KeypointTrust m_keypointTrust;
    SuppressAndAverageKeypoints<Stage2Keypoint, Stage2KeypointGroup> m_keypointGrouper;

    Stage2Corners m_corners;

    // TODO - not sure how configurable this needs to be
    static constexpr size_t m_maxBatchSize = 4;
    double m_minGridMatchRatio = 0.4;
};

#include "marker_dict.h"
template <size_t GRID_SIZE>
using ArucoSTagDecoder = STagDecoder<ArucoMarkerDict<GRID_SIZE>, GRID_SIZE>;
#endif
