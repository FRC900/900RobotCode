#include <opencv2/opencv.hpp>

#include "deeptag_ros/cuda_event_timing.h"
#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/gpu_image_wrapper.h"
#include "deeptag_ros/marker_dict.h"
#include "deeptag_ros/stag_decoder.h"

constexpr size_t MARKER_GRID_SIZE = 6;

static cv::Mat getTag(const cv::Mat &image, const size_t outputHW, const cv::Mat H)
{
    cv::Mat tag;
    warpPerspective(image, tag, H, cv::Size(outputHW, outputHW));
    return tag;
}

template <size_t GRID_SIZE>
static void writeStage2Debug(cv::Mat &image,
                             const PointsAndIDs<GRID_SIZE + 2> &keypointsAndIds,
                             const uint16_t tagId)
{
    for (size_t kp = 0; kp < keypointsAndIds.m_point.size(); kp++)
    {
        const auto id = keypointsAndIds.m_id[kp];
        cv::circle(image,
                   cv::Point2d(keypointsAndIds.m_point[kp].x, keypointsAndIds.m_point[kp].y),
                   3,
                   (id < 0) ?
                        cv::Scalar(255, 0, 0) : (id == 0) ? cv::Scalar(0, 0, 255)
                                                            : cv::Scalar(0, 255, 0));
    }
    std::stringstream s;
    s << tagId;
    cv::putText(image, s.str(), cv::Point(5, 35), 0, 1.5, cv::Scalar(0, 255, 255), 2);
}

template <size_t GRID_SIZE>
void visualizeStage2(cv::Mat &image,
                     const size_t outputHW,
                     const std::vector<std::array<DecodedTag<GRID_SIZE>, 2>> &result)
{
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
    // Arrange tags horizontally, with the first pass of the tag decode on top and second on the bottom
    for (size_t i = 0; i < result.size(); i++)
    {
        tag = getTag(image, outputHW, result[i][0].m_HCrop);
        writeStage2Debug<GRID_SIZE>(tag, result[i][0].m_keypointsWithIds, result[i][0].m_tagId);
        tag.copyTo(output(cv::Rect(i * outputHW, 0, outputHW, outputHW)));

        tag = getTag(image, outputHW, result[i][1].m_HCrop);
        writeStage2Debug<GRID_SIZE>(tag, result[i][1].m_keypointsWithIds, result[i][1].m_tagId);
        tag.copyTo(output(cv::Rect(i * outputHW, outputHW, outputHW, outputHW)));
    }
    image = output;
}
int main(int argc, char **argv)
{
    const auto cameraMatrix = (cv::Mat_<double>(3, 3) << 128., 0.0, 128., 0.0, 128., 128., 0.0, 0.0, 1.0);
    const auto distCoeffs = (cv::Mat_<double>(1, 8) << 0, 0, 0, 0, 0, 0, 0, 0);
    Timings timings{};
    ArucoMarkerDict<MARKER_GRID_SIZE> arucoMarkerDict{cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11};
    ArucoSTagDecoder<MARKER_GRID_SIZE> sTagDecoder{arucoMarkerDict, cameraMatrix, distCoeffs, timings};

    sTagDecoder.initEngine("/home/ubuntu/900RobotCode/zebROS_ws/src/deeptag_ros/models", "apriltag_decoder_mono.onnx");

    const std::string inputImage = argv[1];
    auto cpuImg = cv::imread(inputImage, cv::IMREAD_GRAYSCALE);
    if (cpuImg.empty())
    {
        throw std::runtime_error("Unable to read image at path: " + inputImage);
    }
    std::vector<std::vector<GpuImageWrapper>> detectInputs;
    detectInputs.emplace_back();
    detectInputs[0].emplace_back();
    detectInputs[0][0].upload(cpuImg, sTagDecoder.getCudaStream());
    std::vector<std::array<cv::Point2d, 4>> rois;
    rois.emplace_back();
    rois[0][0] = cv::Point2d{.15 * 256, .15 * 256};
    rois[0][1] = cv::Point2d{.85 * 256, .15 * 256};
    rois[0][2] = cv::Point2d{.85 * 256, .85 * 256};
    rois[0][3] = cv::Point2d{.15 * 256, .85 * 256};
    const auto decodedTags = sTagDecoder.detectTags(detectInputs, rois);
    for (const auto &decodedTag : decodedTags)
    {
        std::cout << "Tag " << decodedTag[1].m_tagId << std::endl;
    }
    cv::Mat stage2DebugImg = cpuImg.clone();
    cv::cvtColor(stage2DebugImg, stage2DebugImg, cv::COLOR_GRAY2BGR);
    visualizeStage2<MARKER_GRID_SIZE>(stage2DebugImg, sTagDecoder.getModelSize().x, decodedTags);
    cv::imshow((inputImage + "_stage_2").c_str(), stage2DebugImg);

    cv::waitKey(0);
    return 0;
}