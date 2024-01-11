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
    using enum DeepTagType;
    case APRILTAG_16H5:
        return cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_16h5;
    case APRILTAG_25H9:
        return cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_25h9;
    case APRILTAG_36H10:
        return cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h10;
    case APRILTAG_36H11:
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
                                                                        const std::string &detectOnnxModelPath,
                                                                        const std::string &decodeOnnxModelPath)
    : DeepTagImplBase{}
    , m_sTagDetector{getTimings()}
    , m_arucoMarkerDict{deepTagTypeToCVName(dictionaryType)}
    , m_sTagDecoder{m_arucoMarkerDict, cameraMatrix, distCoeffs, getTimings()}
    , m_poseEstimator{cameraMatrix, distCoeffs, m_arucoMarkerDict.getUnitTagTemplate().getUnitTags(), tagRealSizeInMeter}
{
    m_sTagDetector.initEngine(detectOnnxModelPath);
    m_sTagDecoder.initEngine(decodeOnnxModelPath);

    cv::namedWindow("Trackbars", cv::WINDOW_NORMAL);
    cv::TrackbarCallback trackbarCallback = [](int pos, void *userdata) { (*(TrackbarAction *)userdata)(pos); };
    cv::createTrackbar("CornerMinCenterScore", "Trackbars", nullptr, 100, trackbarCallback, &m_cornerMinCenterScoreAction);
    cv::setTrackbarPos("CornerMinCenterScore", "Trackbars", static_cast<int>(m_sTagDetector.getCornerMinCenterScore() * 100));

    cv::createTrackbar("SSDMinCenterScore", "Trackbars", nullptr, 100, trackbarCallback, &m_ssdMinCenterScoreAction);
    cv::setTrackbarPos("SSDMinCenterScore", "Trackbars", static_cast<int>(m_sTagDetector.getSSDMinCenterScore() * 100));

    cv::createTrackbar("GridGrouperSigma", "Trackbars", nullptr, 200, trackbarCallback, &m_gridGrouperSigmaAction);
    cv::setTrackbarPos("GridGrouperSigma", "Trackbars", m_sTagDetector.getGridGrouperSigma());

    cv::createTrackbar("SSDGrouperSigma", "Trackbars", nullptr, 200, trackbarCallback, &m_ssdGrouperSigmaAction);
    cv::setTrackbarPos("SSDGrouperSigma", "Trackbars", m_sTagDetector.getSSDGrouperSigma());
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE, size_t MARKER_GRID_SIZE>
std::vector<DeepTagResult> DeepTagImpl<NUM_TILES, USE_SCALED_IMAGE, MARKER_GRID_SIZE>::runInference(const cv::Mat &cpuImg)
{
    getTimings().start("e2e", m_sTagDetector.getCudaStream());
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
        if (decodedTags[i].m_isValid && decodedTags[i].m_tagId != -1)
        {
            results.emplace_back();
            auto &result = results.back();
            result.m_stage1Corners = tagDetectInfo[i].m_orderedCorners;
            result.m_stage2Corners = decodedTags[i].m_roi;
            result.m_center = cv::Point2d{tagDetectInfo[i].m_centerPos.x, tagDetectInfo[i].m_centerPos.y};
            result.m_tagId = decodedTags[i].m_tagId;
            result.m_centerScore = tagDetectInfo[i].m_centerScore;
            for (size_t j = 0; j < 4; j++)
            {
                result.m_cornerScores[j] = tagDetectInfo[i].m_cornerScores[j];
            }
            for (const auto &kp : decodedTags[i].m_keypointsWithIds)
            {
                result.m_keypointScores.push_back(kp.m_score);
            }
            if (m_poseEstimator.fineGridKeypointsToPose(result.m_rVec, result.m_tVec, decodedTags[i].m_keypointsInImage))
            {
#ifdef DEBUG
                std::cout << "result.m_rVec = " << result.m_rVec.t() << " m_tVec = " << result.m_tVec.t() << std::endl;
#endif
            }
        }
    }
    getTimings().end("pose", m_sTagDetector.getCudaStream());

    getTimings().start("nms", m_sTagDetector.getCudaStream());
    std::vector<cv::RotatedRect> boundingBoxes;
    std::vector<float> scores;
    for (const auto &r : results)
    {
        boundingBoxes.push_back(cv::minAreaRect(std::array<cv::Point2f, 4> {r.m_stage2Corners[0], r.m_stage2Corners[1], r.m_stage2Corners[2], r.m_stage2Corners[3]}));
        scores.push_back(static_cast<float>(r.m_centerScore +
                                            std::reduce(r.m_cornerScores.cbegin(), r.m_cornerScores.cend()) +
                                            std::reduce(r.m_keypointScores.cbegin(), r.m_keypointScores.cend())));
    }
    std::vector<int> keepIndexes;
    cv::dnn::NMSBoxes(boundingBoxes, scores, m_nmsConfidenceThreshold, m_nmsNMSThreshold, keepIndexes);

    decltype(results) nmsResults;

    for (const auto keepIndex : keepIndexes)
    {
        nmsResults.push_back(results[keepIndex]);
    }
    getTimings().end("nms", m_sTagDetector.getCudaStream());

    getTimings().end("e2e", m_sTagDetector.getCudaStream());
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