#include <numeric>
#define DEBUG
#include "deeptag_ros/debug.h"
#include "deeptag_ros/deeptag_impls.h"
#define DEBUG
#ifdef DEBUG
#include <opencv2/opencv.hpp>
#endif

void DeepTagImplBase::setTimingsEnabled(const bool enabled)
{
    m_timings.setEnabled(enabled);
}

Timings &DeepTagImplBase::getTimings(void)
{
    return m_timings;
}

static cv::aruco::PREDEFINED_DICTIONARY_NAME deepTagTypeToCVName(const DeepTagType type)
{
    switch (type)
    {
    case DeepTagType::APRILTAG_16H5:
        return cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_16h5;
    case DeepTagType::APRILTAG_25H9:
        return cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_25h9;
    case DeepTagType::APRILTAG_36H10:
        return cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h10;
    case DeepTagType::APRILTAG_36H11:
        return cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11;
    default:
        throw std::runtime_error("Invalid tag type");
    }
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::DeepTagImpl(const DeepTagType dictionaryType,
                                                                        const cv::Mat &cameraMatrix,
                                                                        const cv::Mat &distCoeffs,
                                                                        const double tagRealSizeInMeter,
                                                                        const std::string &modelPath,
                                                                        const std::string &detectOnnxModelFilename,
                                                                        const std::string &decodeOnnxModelFilename)
    : DeepTagImplBase{}
    , m_sTagDetector{getTimings()}
    , m_arucoMarkerDict{deepTagTypeToCVName(dictionaryType)}
    , m_sTagDecoder{m_arucoMarkerDict, cameraMatrix, distCoeffs, getTimings()}
    , m_poseEstimator{cameraMatrix, distCoeffs, m_arucoMarkerDict.getUnitTagTemplate().getUnitTags(), tagRealSizeInMeter}
{
    m_sTagDetector.initEngine(modelPath, detectOnnxModelFilename);
    m_sTagDecoder.initEngine(modelPath, decodeOnnxModelFilename);
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
std::vector<DeepTagResult> DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::runInference(const cv::Mat &cpuImg)
{
    ScopedEventTiming e2eTiming(getTimings(), "e2e", m_sTagDetector.getCudaStream());
    const auto tagDetectInfo = m_sTagDetector.detectTags(cpuImg);

    std::vector<std::array<cv::Point2d, 4>> rois{};
    for (auto &t : tagDetectInfo)
    {
        rois.push_back(t.m_orderedCorners);
    }
    const auto decodedTags = m_sTagDecoder.detectTags(m_sTagDetector.getDetectInputs(), rois);

    getTimings().start("pose", m_sTagDetector.getCudaStream());
    std::vector<DeepTagResult> results;
    for (size_t i = 0; i < decodedTags.size(); i++)
    {
        if (decodedTags[i][1].m_isValid && decodedTags[i][1].m_tagId != -1)
        {
            results.emplace_back();
            auto &result = results.back();
            result.m_stage1Corners = tagDetectInfo[i].m_orderedCorners;
            result.m_stage2Corners = decodedTags[i][1].m_roi;
            result.m_center = cv::Point2d{tagDetectInfo[i].m_centerPos.x, tagDetectInfo[i].m_centerPos.y};
            result.m_tagId = decodedTags[i][1].m_tagId;
            result.m_centerScore = tagDetectInfo[i].m_centerScore;
            for (size_t j = 0; j < 4; j++)
            {
                result.m_cornerScores[j] = tagDetectInfo[i].m_cornerScores[j];
            }
            for (size_t j = 0; j < decodedTags[i].size(); j++)
            {
                for (const auto &kp : decodedTags[i][j].m_keypointsWithIds)
                {
                    result.m_keypoints[j].push_back(kp.m_point);
                    result.m_keypointIds[j].push_back(kp.m_id);
                    result.m_keypointScores[j].push_back(kp.m_score);
                }
            }
            if (m_poseEstimator.fineGridKeypointsToPose(result.m_rVec, result.m_tVec, decodedTags[i][1].m_keypointsInImage))
            {
#ifdef DEBUG
                // std::cout << "result.m_rVec = " << result.m_rVec.t() << " m_tVec = " << result.m_tVec.t() << std::endl;
#endif
            }
        }
    }
    getTimings().end("pose");

    getTimings().start("nms", m_sTagDetector.getCudaStream());
    std::vector<cv::RotatedRect> boundingBoxes;
    std::vector<float> scores;
    for (const auto &r : results)
    {
        boundingBoxes.push_back(cv::minAreaRect(std::array<cv::Point2f, 4> {r.m_stage2Corners[0], r.m_stage2Corners[1], r.m_stage2Corners[2], r.m_stage2Corners[3]}));
        scores.push_back(static_cast<float>(r.m_centerScore +
                                            std::reduce(r.m_cornerScores.cbegin(), r.m_cornerScores.cend()) +
                                            std::reduce(r.m_keypointScores[1].cbegin(), r.m_keypointScores[1].cend())));
    }
    std::vector<int> keepIndexes;
    cv::dnn::NMSBoxes(boundingBoxes, scores, m_nmsConfidenceThreshold, m_nmsNMSThreshold, keepIndexes);

    decltype(results) nmsResults;

    for (const auto keepIndex : keepIndexes)
    {
        nmsResults.push_back(results[keepIndex]);
    }
    getTimings().end("nms");

    return nmsResults;
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::visualize(cv::Mat &image, const std::vector<DeepTagResult> &results) const
{
    for (const auto &result : results)
    {
        m_poseEstimator.drawVecs(image, result.m_rVec, result.m_tVec);
        const auto &roi = result.m_stage2Corners;

        for (size_t r = 0; r < roi.size(); r++)
        {
            cv::line(image, cv::Point2d(roi[r].x, roi[r].y), cv::Point2d(roi[(r + 1) % roi.size()].x, roi[(r + 1) % roi.size()].y), cv::Scalar(0, 128, 128), 2);
        }
        std::stringstream s;
        s << result.m_tagId;
        cv::putText(image, s.str().c_str(), cv::Point2d(roi[0].x, roi[0].y - 5), 0, 0.75, cv::Scalar(0, 128, 0), 2);
    }
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::visualizeStage1Grid(cv::Mat &image)
{
    m_sTagDetector.visualizeGrid(image);
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::visualizeStage1SSD(cv::Mat &image)
{
    m_sTagDetector.visualizeSSD(image);
}

static cv::Mat getTag(const cv::Mat &image, const size_t outputHW, const std::array<cv::Point2d, 4> &corners)
{
    cv::Mat outputRoi(4, 2, CV_64FC1);
    constexpr double borderRatio = 0.15; // TODO : read from stage 2 to-be-added function
    const double borderWidth = outputHW * borderRatio;
    outputRoi.at<double>(0,0) =            borderWidth - 0.5;
    outputRoi.at<double>(0,1) =            borderWidth - 0.5;
    outputRoi.at<double>(1,0) = outputHW - borderWidth - 0.5;
    outputRoi.at<double>(1,1) =            borderWidth - 0.5;
    outputRoi.at<double>(2,0) = outputHW - borderWidth - 0.5;
    outputRoi.at<double>(2,1) = outputHW - borderWidth - 0.5;
    outputRoi.at<double>(3,0) =            borderWidth - 0.5;
    outputRoi.at<double>(3,1) = outputHW - borderWidth - 0.5;

    cv::Mat inputRoi(4, 2, CV_64FC1);
    for (int i = 0; i < 4; i++)
    {
        inputRoi.at<double>(i, 0) = corners[i].x;
        inputRoi.at<double>(i, 1) = corners[i].y;
    }

    const cv::Mat H = cv::findHomography(inputRoi, outputRoi);
    cv::Mat tag;
    warpPerspective(image, tag, H, cv::Size(outputHW, outputHW));
    return tag;
}

static void writeStage2Debug(cv::Mat &image,
                             const std::vector<cv::Point2d> &keypoints,
                             const std::vector<int> &keypointIds,
                             const uint16_t tagId)
{
    for (size_t kp = 0; kp < keypoints.size(); kp++)
    {
        cv::circle(image,
                   cv::Point2d(keypoints[kp].x, keypoints[kp].y),
                   3,
                   (keypointIds[kp] < 0) ?
                        cv::Scalar(255, 0, 0) : (keypointIds[kp] == 0) ? cv::Scalar(0, 0, 255)
                                                                       : cv::Scalar(0, 255, 0));
    }
    std::stringstream s;
    s << tagId;
    cv::putText(image, s.str(), cv::Point(5, 35), 0, 1.5, cv::Scalar(0, 255, 255), 2);
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::visualizeStage2(cv::Mat &image, const std::vector<DeepTagResult> &result) const
{
    size_t outputHW = m_sTagDecoder.getModelSize().x;
    cv::Mat output(outputHW * 2,     // rows
                   outputHW * std::max(result.size(), static_cast<size_t>(1)), // cols
                   CV_8UC3,
                   cv::Scalar(255, 255, 255));
    if (result.empty())
    {
        image = output;
        return;
    }
    cv::Mat tag;
    for (size_t i = 0; i < result.size(); i++)
    {
        tag = getTag(image, outputHW, result[i].m_stage1Corners);
        writeStage2Debug(tag, result[i].m_keypoints[0], result[i].m_keypointIds[0], result[i].m_tagId);
        tag.copyTo(output(cv::Rect(i * outputHW, 0, outputHW, outputHW)));

        tag = getTag(image, outputHW, result[i].m_stage2Corners);
        writeStage2Debug(tag, result[i].m_keypoints[1], result[i].m_keypointIds[1], result[i].m_tagId);
        tag.copyTo(output(cv::Rect(i * outputHW, outputHW, outputHW, outputHW)));
    }
    image = output;
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::saveInputImage(void)
{
    m_sTagDetector.saveInputImage();
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::setCornerMinCenterScore(const double cornerMinCenterScore)
{
    m_sTagDetector.setCornerMinCenterScore(cornerMinCenterScore);
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::setSSDMinCenterScore(const double ssdMinCenterScore)
{
    m_sTagDetector.setSSDMinCenterScore(ssdMinCenterScore);
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::setGridGrouperSigma(const int gridGrouperSigma)
{
    m_sTagDetector.setGridGrouperSigma(gridGrouperSigma);
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::setSSDGrouperSigma(const int ssdGrouperSigma)
{
    m_sTagDetector.setSSDGrouperSigma(ssdGrouperSigma);
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
double DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::getCornerMinCenterScore(void) const 
{
    return m_sTagDetector.getCornerMinCenterScore();
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
double DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::getSSDMinCenterScore(void) const 
{
    return m_sTagDetector.getSSDMinCenterScore();
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
int DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::getGridGrouperSigma(void) const 
{
    return m_sTagDetector.getGridGrouperSigma();
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
int DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::getSSDGrouperSigma(void) const
{
    return m_sTagDetector.getSSDGrouperSigma();
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
double DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::getNMSConfidenceThreshold(void) const
{
    return m_nmsConfidenceThreshold;
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
double DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::getNMSNMSThreshold(void) const
{
    return m_nmsNMSThreshold;
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::setNMSConfidenceThreshold(const double nmsConfidenceThreshold)
{
    m_nmsConfidenceThreshold = nmsConfidenceThreshold;
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::setNMSNMSThreshold(const double nmsNMSThreshold)
{
    m_nmsNMSThreshold = nmsNMSThreshold;
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
void DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::setMinGridMatchRatio(const double minGridMatchRatio)
{
    m_sTagDecoder.setMinGridMatchRatio(minGridMatchRatio);
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
double DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::getMinGridMatchRatio(void) const
{
    return m_sTagDecoder.getMinGridMatchRatio();
}

template class DeepTagImpl<0, true, 4>;
template class DeepTagImpl<0, true, 5>;
template class DeepTagImpl<0, true, 6>;
template class DeepTagImpl<4, true, 4>;
template class DeepTagImpl<4, true, 5>;
template class DeepTagImpl<4, true, 6>;
template class DeepTagImpl<9, true, 4>;
template class DeepTagImpl<9, true, 5>;
template class DeepTagImpl<9, true, 6>;
template class DeepTagImpl<4, false, 4>;
template class DeepTagImpl<4, false, 5>;
template class DeepTagImpl<4, false, 6>;
template class DeepTagImpl<9, false, 4>;
template class DeepTagImpl<9, false, 5>;
template class DeepTagImpl<9, false, 6>;