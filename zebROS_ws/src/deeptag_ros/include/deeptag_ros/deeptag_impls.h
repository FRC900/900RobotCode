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
        virtual void visualizeStage1Grid(cv::Mat &image) = 0;
        virtual void visualizeStage1SSD(cv::Mat &image) = 0;
        virtual void saveInputImage(void) = 0;
        virtual void visualizeStage2(cv::Mat &image, const std::vector<DeepTagResult> &results) const = 0;

        void setTimingsEnabled(const bool enabled);

        virtual void   setCornerMinCenterScore(const double cornerMinCenterScore) = 0;
        virtual void   setSSDMinCenterScore(const double ssdMinCenterScore) = 0;
        virtual void   setGridGrouperSigma(const int gridGrouperSigma) = 0;
        virtual void   setSSDGrouperSigma(const int ssdGrouperSigma) = 0;
        virtual double getCornerMinCenterScore(void) const = 0;
        virtual double getSSDMinCenterScore(void) const = 0;
        virtual int    getGridGrouperSigma(void) const = 0;
        virtual int    getSSDGrouperSigma(void) const = 0;

        virtual void   setNMSConfidenceThreshold(const double nmsConfidenceThreshold) = 0;
        virtual void   setNMSNMSThreshold(const double nmsNMSThreshold) = 0;
        virtual double getNMSConfidenceThreshold(void) const = 0;
        virtual double getNMSNMSThreshold(void) const = 0;

        virtual void setMinGridMatchRatio(const double minGridMatchRatio) = 0;
        virtual double getMinGridMatchRatio(void) const = 0;
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
                             const std::string &modelPath,
                             const std::string &detectOnnxModelFileName,
                             const std::string &decodeOnnxModelFileName);
        DeepTagImpl(const DeepTagImpl &other) = delete;
        DeepTagImpl(DeepTagImpl &&other) noexcept = delete;

        DeepTagImpl &operator=(const DeepTagImpl &other) = delete;
        DeepTagImpl &operator=(DeepTagImpl &&other) noexcept = delete;
        ~DeepTagImpl() override = default;

        std::vector<DeepTagResult> runInference(const cv::Mat &cpuImg) override;
        void visualize(cv::Mat &image, const std::vector<DeepTagResult> &results) const override;
        void visualizeStage1Grid(cv::Mat &image) override;
        void visualizeStage1SSD(cv::Mat &image) override;
        void visualizeStage2(cv::Mat &image, const std::vector<DeepTagResult> &results) const override;
        void saveInputImage(void) override;

        void   setCornerMinCenterScore(const double cornerMinCenterScore) override;
        void   setSSDMinCenterScore(const double ssdMinCenterScore) override;
        void   setGridGrouperSigma(const int gridGrouperSigma) override;
        void   setSSDGrouperSigma(const int ssdGrouperSigma) override;
        double getCornerMinCenterScore(void) const override;
        double getSSDMinCenterScore(void) const override;
        int    getGridGrouperSigma(void) const override;
        int    getSSDGrouperSigma(void) const override;

        void   setNMSConfidenceThreshold(const double nmsConfidenceThreshold) override;
        void   setNMSNMSThreshold(const double nmsNMSThreshold) override;
        double getNMSConfidenceThreshold(void) const override;
        double getNMSNMSThreshold(void) const override;

        void   setMinGridMatchRatio(const double minGridMatchRatio) override;
        double getMinGridMatchRatio(void) const override;
    private:
        STagDetector<NUM_TILES, USE_SCALED_IMAGE> m_sTagDetector;
        ArucoMarkerDict<MARKER_GRID_SIZE> m_arucoMarkerDict;
        ArucoSTagDecoder<MARKER_GRID_SIZE> m_sTagDecoder;
        ArucoPoseEstimator<MARKER_GRID_SIZE> m_poseEstimator;

        double m_nmsConfidenceThreshold{0.5};
        double m_nmsNMSThreshold{0.4};
};

#endif
