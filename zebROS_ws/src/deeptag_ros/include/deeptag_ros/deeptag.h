#ifndef DEEPTAG_INC__
#define DEEPTAG_INC__

#include <iostream>
#include <iterator>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>    // for Mat

// This is the top level interface to deeptag detection code
// The class itself is a thin wrapper around DeepTagImpl.  The
// constructor is a sorta-factory approach to converting runtime
// options for the detector into code which creates the correct
// templated version of the impl class, setting batch size and 
// tag family options.
// The rest of the code is just wrappers to call pImpl->foo()
class DeepTagImplBase;

enum class DeepTagType
{
    APRILTAG_16H5,
    APRILTAG_25H9,
    APRILTAG_36H10,
    APRILTAG_36H11
};

struct DeepTagResult
{
    DeepTagResult(void)
        : m_tVec(3, 1, CV_64FC3)
        , m_rVec(3, 1, CV_64FC3)
    {
    }
    // Corners of tags in the input image coord space ...
    std::array<cv::Point2d, 4> m_stage1Corners{}; // detected from stage 1
    std::array<cv::Point2d, 4> m_stage2Corners{}; // detected using stage 2 refinement of tag keypoints
    cv::Point2d m_center{}; // Center of the tag in input image coord space
    uint16_t m_tagId{std::numeric_limits<uint16_t>::max()};
    cv::Mat m_tVec;
    cv::Mat m_rVec;
    double m_centerScore;
    std::array<double, 4> m_cornerScores;
    // Keypoint location, in stage 2 RoI coordinate system
    std::array<std::vector<cv::Point2d>, 2> m_keypoints{};
    std::array<std::vector<int>, 2> m_keypointIds{};
    std::array<std::vector<double>, 2> m_keypointScores{};

    friend std::ostream &operator<<(std::ostream &os, const DeepTagResult &result)
    {
        os << "Tag ID " << result.m_tagId << std::endl;
        os << "\tstage1Corners = " << result.m_stage1Corners[0] << " "<< result.m_stage1Corners[1] << " "<< result.m_stage1Corners[2] << " "<< result.m_stage1Corners[3] << std::endl;
        os << "\tstage2Corners = " << result.m_stage2Corners[0] << " "<< result.m_stage2Corners[1] << " "<< result.m_stage2Corners[2] << " "<< result.m_stage2Corners[3] << std::endl;
        os << "\tcenter = " << result.m_center << std::endl;
        os << "\ttranslation = " << result.m_tVec.t() << std::endl;
        os << "\trotation = " << result.m_rVec.t() << std::endl;
        os << "\tcenterScore = " << result.m_centerScore << std::endl;
        os << "\tcornerScores = ";
        std::ranges::copy(result.m_cornerScores, std::ostream_iterator<double>(os, " "));
        os << std::endl << "\tkeypointScores[0] = ";
        std::ranges::copy(result.m_keypointScores[0], std::ostream_iterator<double>(os, " "));
        os << std::endl << "\tkeypointScores[1] = ";
        std::ranges::copy(result.m_keypointScores[1], std::ostream_iterator<double>(os, " "));
        os << std::endl;
        return os;
    }
};

class DeepTag
{
    public:
        explicit DeepTag(const cv::Size &cpuImg,
                         const bool tiledDetection,
                         const bool useScaledImage,
                         const DeepTagType dictionaryType,
                         const cv::Mat &cameraMatrix,
                         const cv::Mat &distCoeffs,
                         const double tagRealSizeInMeter,
                         const std::string &modelPath,
                         const std::string &detectOnnxModelFileName,
                         const std::string &decodeOnnxModelFileName);
        DeepTag(const DeepTag &other) = delete;
        DeepTag(DeepTag &&other) noexcept = delete;

        DeepTag &operator=(const DeepTag &other) = delete;
        DeepTag &operator=(DeepTag &&other) noexcept = delete;
        virtual ~DeepTag();

        std::vector<DeepTagResult> runInference(const cv::Mat &cpuImg);
        void visualize(cv::Mat &image, const std::vector<DeepTagResult> &results) const;
        void visualizeStage1Grid(cv::Mat &image);
        void visualizeStage1SSD(cv::Mat &image);
        void visualizeStage2(cv::Mat &image, const std::vector<DeepTagResult> &results) const;
        void saveInputImage(void);
        void setTimingsEnabled(const bool enabled);

        void   setCornerMinCenterScore(const double cornerMinCenterScore);
        void   setSSDMinCenterScore(const double ssdMinCenterScore);
        void   setGridGrouperSigma(const int gridGrouperSigma);
        void   setSSDGrouperSigma(const int ssdGrouperSigma);
        double getCornerMinCenterScore(void) const;
        double getSSDMinCenterScore(void) const;
        int    getGridGrouperSigma(void) const;
        int    getSSDGrouperSigma(void) const;

        void   setNMSConfidenceThreshold(const double nms_confidence_threshold);
        void   setNMSNMSThreshold(const double nms_nms_threshold);
        double getNMSConfidenceThreshold(void) const;
        double getNMSNMSThreshold(void) const;

        void setMinGridMatchRatio(const double minGridMatchRatio);
        double getMinGridMatchRatio(void) const;
    private:
        std::unique_ptr<DeepTagImplBase> m_pImpl;
};

#endif