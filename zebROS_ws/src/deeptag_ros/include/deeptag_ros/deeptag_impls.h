#ifndef DEEPTAG_IMPLS_INC__
#define DEEPTAG_IMPLS_INC__

#include "cuda_event_timing.h"
#include "deeptag.h"
#include "detection_engine.h"
#include "marker_dict.h"
#include "pose_estimator.h"
#include "stag_decoder.h"
#include "stag_detector.h"

class DeepTagImplBase
{
    public:
        explicit DeepTagImplBase() = default;
        DeepTagImplBase(const DeepTagImplBase &other) = delete;
        DeepTagImplBase(DeepTagImplBase &&other) noexcept = delete;

        DeepTagImplBase &operator=(const DeepTagImplBase &other) = delete;
        DeepTagImplBase &operator=(DeepTagImplBase &&other) noexcept = delete;
        virtual ~DeepTagImplBase() = default;

        virtual std::vector<DeepTagResult> runInference(const cv::Mat &cpuImg) = 0;
        virtual void visualize(cv::Mat &image, const std::vector<DeepTagResult> &results) const = 0;

        void setTimingsEnabled(const bool enabled);

    protected:
        Timings &getTimings(void);
    private:
        Timings m_timings{};
};


// Impl for various types of detectors
template<size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
class DeepTagImpl : public DeepTagImplBase
{
    public:
        explicit DeepTagImpl(const DeepTagType dictionaryType,
                             const cv::Mat &cameraMatrix,
                             const cv::Mat &distCoeffs,
                             const double tagRealSizeInMeter,
                             const std::string &detectOnnxModelPath,
                             const std::string &decodeOnnxModelPath);
        DeepTagImpl(const DeepTagImpl &other) = delete;
        DeepTagImpl(DeepTagImpl &&other) noexcept = delete;

        DeepTagImpl &operator=(const DeepTagImpl &other) = delete;
        DeepTagImpl &operator=(DeepTagImpl &&other) noexcept = delete;
        ~DeepTagImpl() override = default;

        std::vector<DeepTagResult> runInference(const cv::Mat &cpuImg) override;
        void visualize(cv::Mat &image, const std::vector<DeepTagResult> &results) const override;
    private:
        STagDetector<NUM_TILES, USE_SCALED_IMAGE> m_sTagDetector;
        ArucoMarkerDict<MARKER_GRID_SIZE> m_arucoMarkerDict;
        ArucoSTagDecoder<MARKER_GRID_SIZE> m_sTagDecoder;
        ArucoPoseEstimator<MARKER_GRID_SIZE> m_poseEstimator;

        float m_nmsConfidenceThreshold{0.5f};
        float m_nmsNMSThreshold{0.4f};

        using TrackbarAction = std::function<void(int)>;
        TrackbarAction m_cornerMinCenterScoreAction = [this](int pos) { m_sTagDetector.setCornerMinCenterScore(static_cast<float>(pos) / 100.f); };
        TrackbarAction m_ssdMinCenterScoreAction = [this](int pos) { m_sTagDetector.setSSDMinCenterScore(static_cast<float>(pos) / 100.f); };
        TrackbarAction m_gridGrouperSigmaAction = [this](int pos) { m_sTagDetector.setGridGrouperSigma(pos); };
        TrackbarAction m_ssdGrouperSigmaAction = [this](int pos) { m_sTagDetector.setSSDGrouperSigma(pos); };
};

#endif
