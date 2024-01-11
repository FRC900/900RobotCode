#include <iostream>
#include <opencv2/core/mat.hpp>      // for Mat
#include <opencv2/core/mat.inl.hpp>  // for _InputOutputArray::_InputOutputA...
#include <opencv2/core/types.hpp>    // for Point2d, Point2f, Point_, Scalar
#include <opencv2/highgui.hpp>       // for imshow
#include <opencv2/imgproc.hpp>       // for circle, line, putText, rectangle
#include "deeptag_ros/stage1_grid.h"
#include "deeptag_ros/cuda_utils.h"
#include "deeptag_ros/cuda_event_timing.h"       // for Timings
#include "deeptag_ros/engine.h"                  // for Options, doesFileExist, Precision
#include "deeptag_ros/gpu_image_wrapper.h"
#include "deeptag_ros/roi_generator.h"           // for TagDetectInfo, centerAndCornersT...
#include "deeptag_ros/ssd_box.h"                 // for SSDBoxCornerForm
#include "deeptag_ros/ssd_tag_keypoint.h"
#include "deeptag_ros/stag_detector.h"
#include "deeptag_ros/stage1_grid_group.h"       // for Stage1GridGroup
#include "deeptag_ros/stage1_ssd_group.h"        // for Stage1SSDGroup
#include "deeptag_ros/tag_detect_info.h"
#define DEBUG
#include "deeptag_ros/debug.h"

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
STagDetector<NUM_TILES, USE_SCALED_IMAGE>::STagDetector(Timings &timing)
    :m_timing{timing}
{
    cudaSafeCall(cudaStreamCreate(&m_ssdCudaStream));
    cudaSafeCall(cudaEventCreate(&m_engineDoneCudaEvent));
    cudaSafeCall(cudaEventCreate(&m_ssdDoneCudaEvent));

    // Hard-code the input size - batch size 1, a single input image
    // This is the case regardless of the NUM_TILES template param ...
    // When NUM_TILES > 1, batches are made up from extracting model-input-sized
    // RoIs from this single input image.
    constexpr size_t detectBatchSize = 1;
    std::vector<GpuImageWrapper> input;
    for (size_t j = 0; j < detectBatchSize; ++j)
    {
        input.emplace_back();
    }
    m_detectInputs.emplace_back(std::move(input));
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
STagDetector<NUM_TILES, USE_SCALED_IMAGE>::~STagDetector()
{
    cudaSafeCall(cudaStreamDestroy(m_ssdCudaStream));
    cudaSafeCall(cudaEventDestroy(m_engineDoneCudaEvent));
    cudaSafeCall(cudaEventDestroy(m_ssdDoneCudaEvent));
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::initEngine(const std::string &enginePath)
{
    if (!Util::doesFileExist(enginePath))
    {
        throw std::runtime_error("Error: Unable to find file at path: " + enginePath);
    }
    // Specify our GPU inference configuration options
    Options detectOptions;
    // Specify what precision to use for inference
    // FP16 is approximately twice as fast as FP32.
    detectOptions.precision = Precision::INT8;
    // If using INT8 precision, must specify path to directory containing calibration data.
    detectOptions.calibrationDataDirectoryPath = "";
    // Use NUM_TILES input-sized images tiles plus one copy of the full image resized
    // The higher resolution tiles should give better detection distance,
    // and the lower resolution full image will help detect very large tags
    // which might be split among multiple tiles
    // If the model does not support dynamic batch size, then the below two parameters must be set to 1.
    // Specify the batch size to optimize for.
    detectOptions.optBatchSize = NUM_TILES + NUM_SCALED_IMAGES;
    // Specify the maximum batch size we plan on running.
    detectOptions.maxBatchSize = NUM_TILES + NUM_SCALED_IMAGES;

    m_detectEngine = std::make_unique<DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>>(detectOptions);

    // Build the onnx model into a TensorRT engine file.
    if (!m_detectEngine->build(enginePath))
    {
        throw std::runtime_error("Unable to build TRT engine.");
    }
    // Load the TensorRT engine file from disk
    if (!m_detectEngine->loadNetwork())
    {
        throw std::runtime_error("Unable to load TRT engine.");
    }
}

// Take a CPU cv::Mat image and use it to prepare
// the inputs for model inference.
// Upload the image to the GPU. Then populate the m_detectInput
// field with that image so it can be passed into 
// model inference.
// Returns a float2 with the x and y dimensions of the input image
template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
ushort2 STagDetector<NUM_TILES, USE_SCALED_IMAGE>::generateInputs(const cv::Mat &cpuImg)
{
    m_timing.start("gpuImage_upload", m_detectEngine->getCudaStream());
    // Upload the image GPU memory
    m_detectInputs[0][0].upload(cpuImg, m_detectEngine->getCudaStream());
    const ushort2 imgSize{static_cast<ushort>(cpuImg.cols), static_cast<ushort>(cpuImg.rows)};
    m_timing.end("gpuImage_upload", m_detectEngine->getCudaStream());

    return imgSize;
}

// grid priors are mappings from normalized (0 - 1) pixel values to actual image pixel coords
// These will be largely no-ops after the first call, they just reallocate when image sizes change.
// Priors are the same for every output batch
template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::generatePriors(const ushort2 &imgSize)
{
    m_timing.start("gridprior_generate", m_detectEngine->getCudaStream());
    const auto detectInputdim = m_detectEngine->getContextInputDims()[0];
    std::array<ushort2, NUM_TILES> tileOffsets;
    m_detectEngine->getTileOffsets(tileOffsets);
    std::cout << __PRETTY_FUNCTION__ << " detectInputDim = " << detectInputdim.d[2] << ", " << detectInputdim.d[3] << std::endl;
    const ushort2 modelInputDims{static_cast<unsigned short>(detectInputdim.d[3]), static_cast<unsigned short>(detectInputdim.d[2])}; // W, H
    if (m_gridPrior.generate(modelInputDims,
                             8,
                             imgSize,
                             tileOffsets,
                             m_detectEngine->getCudaStream()))
    {
        m_regenCudaGraph = true;
    }
    m_timing.end("gridprior_generate", m_detectEngine->getCudaStream());

    m_timing.start("ssdprior_generate", m_ssdCudaStream);
    if (m_ssdGridPrior.generate(modelInputDims,
                                imgSize,
                                tileOffsets,
                                m_ssdCudaStream))
    {
        m_regenCudaGraph = true;
    }
    m_timing.end("ssdprior_generate", m_ssdCudaStream);
}

// Softmax converts confidence values from arbitrary numbers to scores from 0-1
// where the sum for each detection sums to 1.
// Both grid and ssd detectiors are binary classifiers, so we only need to keep
// 1 float value holding the foreground confidence of each detection location
template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::runSoftmax()
{
    std::cout << "m_gridPrior.size() = " << m_gridPrior.getOutput().size() << std::endl;
    std::cout << "m_ssdGridPrior.size() = " << m_ssdGridPrior.getOutput().size() << std::endl;
    m_timing.start("grid_softmax", m_detectEngine->getCudaStream());
    // Here, bg and foreground confidence values are offset by the length of the input,
    // so multiply index by 1 and add len, to read {idx, idx+len}
    const auto stage1GridCount = m_gridPrior.getOutput().size();
    m_gridSoftmax.computeSoftmax(m_detectEngine->getBufferByName("grid_confidences_pred"), 1, stage1GridCount, stage1GridCount, m_detectEngine->getCudaStream());
    m_timing.end("grid_softmax", m_detectEngine->getCudaStream());

    cudaSafeCall(cudaStreamWaitEvent(m_ssdCudaStream, m_engineDoneCudaEvent));
    m_timing.start("ssd_softmax", m_ssdCudaStream);
    // This tensor has x & y confidences back to back. So set mult to 2x and add to 1 to read {2idx+0 and 2idx+1}
    const auto stage1SSDGridCount = m_ssdGridPrior.getOutput().size();
    m_ssdSoftmax.computeSoftmax(m_detectEngine->getBufferByName("ssd_confidences_pred"), 2, 1, stage1SSDGridCount, m_ssdCudaStream);
    m_timing.end("ssd_softmax", m_ssdCudaStream);
}

// These reduce the full set of results down to just a list of those above a min
// confidence score.  For each valid detection, it consolidates the results from
// several different output tensors into a single struct for each detection which
// passes the filter step. It also uses the prior values and image size data
// to scale up from normalized image coords to pixel coords
template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::runConfidenceFilter(const ushort2 &imgSize)
{
    m_timing.start("grid_keypoint", m_detectEngine->getCudaStream());
    m_cornerConfidenceFilter.detect({m_gridSoftmax.getOutput().data(),
                                     m_detectEngine->getBufferByName("grid_locations_pred"),
                                     m_detectEngine->getBufferByName("grid_vals_pred")},
                                    m_gridPrior.getOutput(),
                                    0.1f, // centerVariance
                                    0.0f, // sizeVariance - not used for corners
                                    m_cornerMinCenterScore, // min_center_score
                                    m_detectEngine->getCudaStream(),
                                    m_regenCudaGraph);
    m_timing.end("grid_keypoint", m_detectEngine->getCudaStream());

    m_timing.start("ssd_keypoint", m_ssdCudaStream);
    m_ssdConfidenceFilter.detect({m_ssdSoftmax.getOutput().data(),
                                  m_detectEngine->getBufferByName("ssd_locations_pred"),
                                  m_detectEngine->getBufferByName("ssd_vals_pred")},
                                 m_ssdGridPrior.getOutput(),
                                 0.1f, // centerVariance
                                 0.2f, // sizeVariance
                                 m_ssdMinCenterScore, // min_center_score
                                 m_ssdCudaStream,
                                 m_regenCudaGraph);
    m_timing.end("ssd_keypoint", m_ssdCudaStream);
}

// Next, group objects which are close enough in position into a single
// detection. The position, etc of the grouped value is the confidence-weighted
// average of the values from all of the objects grouped together
template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::runGroupers(const ushort2 &imgSize)
{
    const auto inputDim = m_detectEngine->getContextDim("input");
    const double imageScale = std::max(static_cast<double>(imgSize.x) / inputDim.d[3], static_cast<double>(imgSize.y) / inputDim.d[2]);

    m_timing.start("grid_group", m_detectEngine->getCudaStream());
    printf("m_cornerConfidenceFilter.getOutput().size() = %ld\n", m_cornerConfidenceFilter.getOutput().size());
    m_gridGrouper.compute(m_cornerConfidenceFilter.getOutput(), m_gridGrouperSigma * imageScale, 0.9f, m_detectEngine->getCudaStream());
    m_timing.end("grid_group", m_detectEngine->getCudaStream());

    printf("m_ssdConfidenceFilter.getOutput().size() = %ld\n", m_ssdConfidenceFilter.getOutput().size());
    m_timing.start("ssd_group", m_ssdCudaStream);
    m_ssdGrouper.compute(m_ssdConfidenceFilter.getOutput(), m_ssdGrouperSigma * imageScale, 0.9f, m_ssdCudaStream);
    m_timing.end("ssd_group", m_ssdCudaStream);
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
std::vector<TagDetectInfo> STagDetector<NUM_TILES, USE_SCALED_IMAGE>::detectTags(const cv::Mat &cpuImg)
{
    const auto imgSize = generateInputs(cpuImg);

    std::vector<std::vector<std::vector<float>>> featureVectors;
    m_timing.start("detect_inference", m_detectEngine->getCudaStream());
    bool buffersResized = false;
    if (!m_detectEngine->runInference(m_detectInputs, featureVectors, buffersResized, NUM_TILES + NUM_SCALED_IMAGES))
    {
        throw std::runtime_error("Could not run detect inference.");
    }
    if (buffersResized)
    {
        m_regenCudaGraph = true;
    }
    // Record an event that triggers once inference is complete
    m_timing.end("detect_inference", m_detectEngine->getCudaStream());
    cudaSafeCall(cudaEventRecord(m_engineDoneCudaEvent, m_detectEngine->getCudaStream()));

    // For batch size > 1, we're breaking the input image into model-input sized
    // tiles and running them through the model.  This means that inference results
    // are scaled relative to the size of that input tile rather than the original image
    // A separate field holds the offset of the origin of each tile in the original
    // input image.
    const auto contextInputDim = m_detectEngine->getContextDim("input");
    const ushort2 inputSize{static_cast<ushort>(contextInputDim.d[3]), static_cast<ushort>(contextInputDim.d[2])};
    // Schedule a few independent kernels here.
    // From here, grid and sdd postproc can run in parallel since they're independent.
    // Grid takes longer, so start it first. Have it use the engine's cudaStream. Have
    // a separate stream for the SSD detection, and make that stream explicitly wait until the
    // inference output is ready before proceeding.
    // Since the grid code runs on the same stream as the engine inference, the wait
    // there is automatic.

    generatePriors(imgSize);
    runSoftmax();
    runConfidenceFilter(inputSize);
    runGroupers(imgSize);

    // Copy final results from device back to host
    m_timing.start("grid_finalGetOutput", m_detectEngine->getCudaStream());
    const tcb::span<const Stage1GridGroup<NUM_TILES + NUM_SCALED_IMAGES>> hstage1GridGroup = m_gridGrouper.getOutput();
    m_timing.end("grid_finalGetOutput", m_detectEngine->getCudaStream());

    m_timing.start("ssd_finalGetOutput", m_ssdCudaStream);
    const tcb::span<const Stage1SSDGroup> hstage1SSDGroup = m_ssdGrouper.getOutput();
    cudaSafeCall(cudaEventRecord(m_ssdDoneCudaEvent, m_ssdCudaStream));
    m_timing.end("ssd_finalGetOutput", m_ssdCudaStream);

    // Merge the streams back into one - have grid wait on sdd stream to finish
    // before running the last bit of stage 1 ROI detection
    // Note, this is probably a no-op since getOutput does a stream sync anyway
    // to insure data is available on the host
    m_timing.start("ssd_final_streamwaitevent", m_ssdCudaStream);
    cudaSafeCall(cudaStreamWaitEvent(m_detectEngine->getCudaStream(), m_ssdDoneCudaEvent));
    m_timing.end("ssd_final_streamwaitevent", m_ssdCudaStream);

    m_timing.start("centerAndCornersToTags", m_detectEngine->getCudaStream());
    std::vector<TagDetectInfo> tagDetectInfo;
    std::vector<std::vector<cv::Point2d>> centerToCornerLinks;
    centerAndCornersToTags(tagDetectInfo,
                           centerToCornerLinks,
                           hstage1GridGroup,
                           hstage1SSDGroup);
    m_timing.end("centerAndCornersToTags", m_detectEngine->getCudaStream());

#ifdef DEBUG
    std::cout << "===================================================\n";
    for (const auto &g : hstage1GridGroup)
    {
        g.print();
        std::cout << "---------------------------------------------------\n";
    }
    std::cout << "===================================================\n";
    for (const auto &s : hstage1SSDGroup)
    {
        s.print();
        std::cout << "---------------------------------------------------\n";
    }
    for (const auto &t : tagDetectInfo)
    {
        t.print();
        std::cout << "---------------------------------------------------\n";
    }
    for (const auto &cc : centerToCornerLinks)
    {
        for (const auto &c : cc)
        {
            std::cout << "center to corner = " << c.x << " " << c.y << std::endl;
        }
        std::cout << "---------------------------------------------------\n";
    }
    cv::Mat anchorsDebug = cpuImg.clone();
    //cv::Mat centerCornersDebug = cpuImg.clone();
    cv::Mat orderedCornersImg = cpuImg.clone();
    for (size_t j = 0; j < tagDetectInfo.size(); j++)
    {
        cv::rectangle(anchorsDebug, cv::Rect2d(cv::Point2d(hstage1SSDGroup[j].m_bbox.m_tl.x, hstage1SSDGroup[j].m_bbox.m_tl.y), cv::Point2d(hstage1SSDGroup[j].m_bbox.m_br.x, hstage1SSDGroup[j].m_bbox.m_br.y)), cv::Scalar(128, 128, 0), 2);
        cv::circle(anchorsDebug, cv::Point2d(hstage1SSDGroup[j].m_center.x, hstage1SSDGroup[j].m_center.y), 5, cv::Scalar(0, 0, 128));

        for (int i = 0; i < 4; i++)
        {
            const auto &a = hstage1SSDGroup[j].m_anchorsInBox[i];
            cv::circle(anchorsDebug, cv::Point2d(a.x, a.y), 4, cv::Scalar(128, 0, 128));
        }
        #if 0
        cv::rectangle(centerCornersDebug, cv::Rect2d(cv::Point2d(hstage1SSDGroup[j].m_bbox.m_tl.x, hstage1SSDGroup[j].m_bbox.m_tl.y), cv::Point2d(hstage1SSDGroup[j].m_bbox.m_br.x, hstage1SSDGroup[j].m_bbox.m_br.y)), cv::Scalar(128, 128, 0), 2);
        cv::circle(centerCornersDebug, cv::Point2d(tagDetectInfo[j].m_centerPos.x, tagDetectInfo[j].m_centerPos.y), 5, cv::Scalar(0, 0, 128));
        for (size_t i = 0; i < 4; i++)
        {
            const auto &a = tagDetectInfo[j].m_anchors[i];
            cv::circle(centerCornersDebug, cv::Point2d(a.x, a.y), 4, cv::Scalar(128, 0, 128));
        }
        #endif

        const auto &points = tagDetectInfo[j].m_orderedCorners;
        for (size_t i = 0; i < points.size(); i++)
        {
            cv::line(orderedCornersImg, cv::Point2f(points[i].x, points[i].y), cv::Point2f(points[(i + 1) % points.size()].x, points[(i + 1) % points.size()].y), cv::Scalar(0, 128, 128), 2);
            std::stringstream s;
            s << i;
            cv::putText(orderedCornersImg, s.str().c_str(), cv::Point2f(points[i].x, points[i].y), 0, 0.75, cv::Scalar(128, 128, 0), 1);
        }
    }
    cv::imshow("AnchorsDebugAndBBox", anchorsDebug);
    cv::imshow("OrderedCorners", orderedCornersImg);
    //cv::imshow("CenterCornersAndAnchors", centerCornersDebug);

#endif

    m_regenCudaGraph = false;
    return tagDetectInfo;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
const std::vector<std::vector<GpuImageWrapper>> &STagDetector<NUM_TILES, USE_SCALED_IMAGE>::getDetectInputs(void) const
{
    return m_detectInputs;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
cudaStream_t STagDetector<NUM_TILES, USE_SCALED_IMAGE>::getCudaStream(void)
{
    return m_detectEngine->getCudaStream();
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::setCornerMinCenterScore(const float cornerMinCenterScore)
{
    m_cornerMinCenterScore = std::max(cornerMinCenterScore, 0.01f);
    m_regenCudaGraph = true;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::setSSDMinCenterScore(const float ssdMinCenterScore)
{
    m_ssdMinCenterScore = std::max(ssdMinCenterScore, 0.01f);
    m_regenCudaGraph = true;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::setGridGrouperSigma(const int gridGrouperSigma)
{
    m_gridGrouperSigma = gridGrouperSigma;
    m_regenCudaGraph = true;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
void STagDetector<NUM_TILES, USE_SCALED_IMAGE>::setSSDGrouperSigma(const int ssdGrouperSigma)
{
    m_ssdGrouperSigma = ssdGrouperSigma;
    m_regenCudaGraph = true;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
float STagDetector<NUM_TILES, USE_SCALED_IMAGE>::getCornerMinCenterScore(void) const
{
    return m_cornerMinCenterScore;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
float STagDetector<NUM_TILES, USE_SCALED_IMAGE>::getSSDMinCenterScore(void) const
{
    return m_ssdMinCenterScore;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
int STagDetector<NUM_TILES, USE_SCALED_IMAGE>::getGridGrouperSigma(void) const
{
    return m_gridGrouperSigma;
}

template<size_t NUM_TILES, bool USE_SCALED_IMAGE>
int STagDetector<NUM_TILES, USE_SCALED_IMAGE>::getSSDGrouperSigma(void) const
{
    return m_ssdGrouperSigma;
}

#include "deeptag_ros/detection_engine.h"
template class STagDetector<0, true>;

template class STagDetector<4, false>;
template class STagDetector<4, true>;
template class STagDetector<9, false>;
template class STagDetector<9, true>;