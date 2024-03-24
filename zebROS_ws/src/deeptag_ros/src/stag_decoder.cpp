#include "deeptag_ros/stag_decoder.h"
#include <algorithm>                  // for min
#include <cmath>                      // for hypot
#include <iostream>                   // for operator<<, basic_ostream, endl
#include <opencv2/core/mat.inl.hpp>   // for _InputOutputArray::_InputOutput...
#include <opencv2/highgui.hpp>        // for waitKey, imshow
#include <opencv2/imgproc.hpp>        // for line, circle, putText
#include <stdexcept>                  // for runtime_error
#include "deeptag_ros/cuda_event_timing.h"        // for Timings
#include "deeptag_ros/engine.h"                   // for Options, doesFileExist, Precision
#include "deeptag_ros/points_and_ids.h"           // for PointsAndIDs
#include "deeptag_ros/stage2_keypoint_group.h"    // for Stage2KeypointGroup
#include "deeptag_ros/unit_tag_template.h"        // for UnitTagTemplateArucotag
#include "deeptag_ros/warp_perspective_points.h"  // for warpPerspectivePts

// #define DEBUG
#include "deeptag_ros/debug.h"

template <class MARKER_DICT, size_t GRID_SIZE>
STagDecoder<MARKER_DICT, GRID_SIZE>::STagDecoder(const MARKER_DICT &markerDict, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, Timings &timing)
    : m_markerDict(markerDict)
    , m_cameraMatrix{cameraMatrix}
    , m_distCoeffs{distCoeffs}
    , m_timing{timing}
{
}

template <class MARKER_DICT, size_t GRID_SIZE>
void STagDecoder<MARKER_DICT, GRID_SIZE>::initEngine(const std::string &modelPath, const std::string &onnxModelFilename)
{
    if (const std::string onnxModelPath = modelPath + "/" + onnxModelFilename;
        !Util::doesFileExist(onnxModelPath))
    {
        throw std::runtime_error("Error: Unable to find ONNX model file at path: " + onnxModelPath);
    }
    // Specify our GPU inference configuration options
    Options decodeOptions;
    // Specify what precision to use for inference
    // FP16 is approximately twice as fast as FP32.
    decodeOptions.precision = Precision::FP16;
    // If using INT8 precision, must specify path to directory containing calibration data.
    decodeOptions.calibrationDataDirectoryPath = "/home/ubuntu";
    // If the model does not support dynamic batch size, then the below two parameters must be set to 1.
    // Specify the batch size to optimize for.
    decodeOptions.optBatchSize = 4;
    // Specify the maximum batch size we plan on running.
    decodeOptions.maxBatchSize = m_maxBatchSize;
    m_decodeEngine = std::make_unique<DecoderEngine>(decodeOptions);
    // Build the onnx model into a TensorRT engine file.
    if (!m_decodeEngine->build(modelPath, onnxModelFilename))
    {
        throw std::runtime_error("Unable to build TRT engine.");
    }
    // Load the TensorRT engine file from disk
    if (!m_decodeEngine->loadNetwork())
    {
        throw std::runtime_error("Unable to load TRT engine.");
    }
}

// Run 1 batch of inference.
// Batch size is inferred from rois.size()
// append results to vector of Stage2Keypoint vectors, 1 per input RoI
// and the corresponding corners vector of Stage2Keypoint vectors, 1 per input RoI 
// Note : these accmulate values over each batch of the input, so the caller
// is responsible for clearing them if needed.
// (although TODO : those might be duplicates of data in the KeyPoints)
template <class MARKER_DICT, size_t GRID_SIZE>
void STagDecoder<MARKER_DICT, GRID_SIZE>::runInference(std::vector<std::vector<Stage2KeypointGroup>> &stage2KeypointGroups,
                                                       std::vector<std::vector<float2>> &stage2Corners,
                                                       const std::vector<std::vector<GpuImageWrapper>> &detectInputs,
                                                       const tcb::span<const std::array<cv::Point2d, 4>> &rois)
{
    // Run a batch of results in one inference pass
    m_timing.start("setROIs", m_decodeEngine->getCudaStream());
#ifdef DEBUG
    printPoints("rois[0]", rois[0]);
    std::cout << "rois.size() = " << rois.size() << std::endl;
#endif
    m_decodeEngine->setROIs(rois);
    m_timing.end("setROIs", m_decodeEngine->getCudaStream());

    m_timing.start("decode_runInference", m_decodeEngine->getCudaStream());
    std::vector<std::vector<std::vector<float>>> featureVectors;
    bool buffersResized;
    if (!m_decodeEngine->runInference(detectInputs, featureVectors, buffersResized, rois.size()))
    {
        throw std::runtime_error("Could not run decode inference.");
    }
    m_timing.end("decode_runInference", m_decodeEngine->getCudaStream());

    // Priors are the same for each batch # in the output, generate
    // them just once here
    m_timing.start("stage2_corner_priors", m_decodeEngine->getCudaStream());
    // model input size and image size are the same
    m_stage2CornerPrior.generate(getModelSize(), 128, getModelSize(), {}, m_decodeEngine->getCudaStream());
    m_timing.end("stage2_corner_priors", m_decodeEngine->getCudaStream());

    m_timing.start("stage2_grid_priors", m_decodeEngine->getCudaStream());
    m_stage2GridPrior.generate(getModelSize(), 8, getModelSize(), {}, m_decodeEngine->getCudaStream());
    m_timing.end("stage2_grid_priors", m_decodeEngine->getCudaStream());

    for (size_t roiNum = 0; roiNum < rois.size(); roiNum++)
    {
        m_timing.start("stage2_softmax", m_decodeEngine->getCudaStream());
        m_stage2DecoderSoftmax.compute(m_decodeEngine->getBufferByName("confidences_pred", roiNum),
                                       32 * 32,
                                       m_decodeEngine->getCudaStream());
        m_timing.end("stage2_softmax", m_decodeEngine->getCudaStream());
        
        m_timing.start("stage2_keypoint_detect", m_decodeEngine->getCudaStream());
        m_confidenceFilter.detect({m_stage2DecoderSoftmax.getOutput().data(),
                                   m_decodeEngine->getBufferByName("locations_pred", roiNum),
                                   nullptr /* not used */},
                                  m_stage2GridPrior.getOutput(),
                                  0.05f,             // centerVariance
                                  0.0f,              // sizeVariance - not used for corners
                                  0.6f,              // min confidence // TODO : configurable
                                  m_decodeEngine->getCudaStream(),
                                  buffersResized);
        buffersResized = false; // only need to re-do cuda graphs once per iteration, they're the same until the next infer call at least
        m_timing.end("stage2_keypoint_detect", m_decodeEngine->getCudaStream());

        m_timing.start("stage2_trust", m_decodeEngine->getCudaStream());
        const bool trustFlag = m_keypointTrust.check(m_confidenceFilter.getOutput(), m_decodeEngine->getCudaStream());
        m_timing.end("stage2_trust", m_decodeEngine->getCudaStream());

#ifdef DEBUG
        std::cout << " roiNum = " << roiNum << " trustFlag = " << (int)trustFlag << std::endl;
#endif
        stage2KeypointGroups.emplace_back();
        stage2Corners.emplace_back();
        if (trustFlag)
        {
            m_timing.start("stage2_keypoint_group", m_decodeEngine->getCudaStream());
            m_keypointGrouper.compute(m_confidenceFilter.getOutput(), 12, 0.0, m_decodeEngine->getCudaStream());
            m_timing.end("stage2_keypoint_group", m_decodeEngine->getCudaStream());

            m_timing.start("stage2_corner_locations", m_decodeEngine->getCudaStream());
            m_corners.compute(m_decodeEngine->getBufferByName("corner_locations_pred", roiNum),
                              m_stage2CornerPrior.getOutput(),
                              0.05f,
                              m_decodeEngine->getCudaStream());
            m_timing.end("stage2_corner_locations", m_decodeEngine->getCudaStream());

            m_timing.start("stage2_keypoint_group_out", m_decodeEngine->getCudaStream());
            const tcb::span<const Stage2KeypointGroup> hStage2KeypointGroup = m_keypointGrouper.getOutput();
            for (const auto &k : hStage2KeypointGroup)
            {
                stage2KeypointGroups.back().push_back(k);
            }
            m_timing.end("stage2_keypoint_group_out", m_decodeEngine->getCudaStream());

            m_timing.start("stage2_corners_out", m_decodeEngine->getCudaStream());
            const tcb::span<const float2> hStage2Corners = m_corners.getHostOutput();
            for (const auto &c : hStage2Corners)
            {
                stage2Corners.back().push_back(c);
            }
            m_timing.end("stage2_corners_out", m_decodeEngine->getCudaStream());
        }
    }
}

template <class MARKER_DICT, size_t GRID_SIZE>
std::vector<std::array<DecodedTag<GRID_SIZE>, 2>> STagDecoder<MARKER_DICT, GRID_SIZE>::detectTags(const std::vector<std::vector<GpuImageWrapper>> &detectInputs,
                                                                                   const std::vector<std::array<cv::Point2d, 4>> &rois)
{
    std::vector<std::array<cv::Point2d, 4>> thisRois{rois};
    std::vector<std::vector<Stage2KeypointGroup>> stage2KeypointGroups;
    std::vector<std::vector<float2>> stage2Corners;
    std::vector<std::array<DecodedTag<GRID_SIZE>, 2>> ret;
    for (size_t iter = 0; iter < 2; iter++)
    {
#ifdef DEBUG
        std::cout << "================================================" << std::endl << "iter = " << iter << std::endl;
#endif
        size_t retIdx = 0;
        stage2KeypointGroups.clear();
        stage2Corners.clear();
        const tcb::span<const std::array<cv::Point2d, 4>> thisRoiSpan{thisRois};
        for (size_t batchStart = 0; batchStart < rois.size(); batchStart += m_maxBatchSize)
        {
            const size_t thisBatchSize = std::min(rois.size() - batchStart, m_maxBatchSize);
            auto thisRoiSubspan = thisRoiSpan.subspan(batchStart, thisBatchSize);
            runInference(stage2KeypointGroups, stage2Corners, detectInputs, thisRoiSubspan);

            for (size_t ii = 0; ii < thisRoiSubspan.size(); ii++)
            {
                if (iter == 0)
                {
                    ret.push_back(std::array<DecodedTag<GRID_SIZE>, 2>{});
                    ret[retIdx][0].m_HCrop = m_decodeEngine->getH(ii);
                }
                ret[retIdx][iter].m_isValid = stage2KeypointGroups[retIdx].size() > 0;
#ifdef DEBUG
                std::cout << "iter = " << iter << " ret[" << retIdx << "].m_isValid = " << ret[retIdx].m_isValid << std::endl;
#endif
                if (ret[retIdx][iter].m_isValid)
                {
#ifdef DEBUG
                    std::cout << "MatchFineGrid : ii = " << ii << " retIdx = " << retIdx << std::endl;
#endif
                    m_timing.start("stage2_matchfinegrid", m_decodeEngine->getCudaStream());
                    double matchRatio;
                    constexpr auto FINE_GRID_SIZE = MARKER_DICT::getGridSize() + 2;
                    std::array<PointsAndIDs, FINE_GRID_SIZE * FINE_GRID_SIZE> orderedFineGridPointsIds;
                    m_markerDict.getUnitTagTemplate().matchFineGrid(matchRatio,
                                                                    orderedFineGridPointsIds,
                                                                    stage2KeypointGroups[retIdx],
                                                                    m_decodeEngine->getH(ii),
                                                                    stage2Corners[retIdx],
                                                                    m_cameraMatrix, // cameraMatrix
                                                                    m_distCoeffs);  // distCoeffs
                    m_timing.end("stage2_matchfinegrid", m_decodeEngine->getCudaStream());

#ifdef DEBUG
                    std::cout << "matchRatio = " << matchRatio << " m_minGridMatchRatio " << m_minGridMatchRatio << std::endl;
#endif
                    if (matchRatio > m_minGridMatchRatio)
                    {
                        m_timing.start("stage2_fillemptyids", m_decodeEngine->getCudaStream());
                        //fillEmptyIds(orderedFineGridPointsIds, stage2KeypointGroups[retIdx]);
                        m_timing.end("stage2_fillemptyids", m_decodeEngine->getCudaStream());

                        m_timing.start("stage2_updatecornersinimage", m_decodeEngine->getCudaStream());
                        const auto roiUpdated = m_markerDict.getUnitTagTemplate().updateCornersInImage(orderedFineGridPointsIds,
                                                                                                       m_decodeEngine->getH(ii),
                                                                                                       m_cameraMatrix,
                                                                                                       m_distCoeffs);
                        m_timing.end("stage2_updatecornersinimage", m_decodeEngine->getCudaStream());

                        m_timing.start("stage2_getmainindex", m_decodeEngine->getCudaStream());
                        thisRois[retIdx] = roiUpdated;
                        ret[retIdx][iter].m_roi = roiUpdated;
                        std::array<int, FINE_GRID_SIZE * FINE_GRID_SIZE> tagBits;
                        for (size_t i = 0; i < orderedFineGridPointsIds.size(); i++)
                        {
                            tagBits[i] = orderedFineGridPointsIds[i].m_id;
                        }
                        int hammingDist = 2; // TODO - configurable, dynamic reconfig potential

                        m_markerDict.getMainIdx(ret[retIdx][iter].m_mainIdx, ret[retIdx][iter].m_tagId, ret[retIdx][iter].m_binaryId, tagBits, hammingDist);
#ifdef DEBUG
                        std::cout << "mainIdx = " << ret[retIdx].m_mainIdx << " tagId = " << ret[retIdx].m_tagId << std::endl;
#endif
                        m_timing.end("stage2_getmainindex", m_decodeEngine->getCudaStream());

                        m_timing.start("stage2_reorderpointswithmainidx", m_decodeEngine->getCudaStream());
                        std::array<PointsAndIDs, GRID_SIZE * GRID_SIZE> orderedKptsWithIds;
                        m_markerDict.getUnitTagTemplate().reorderPointsWithMainIdx(ret[retIdx][iter].m_keypointsWithIds, // [re] orderedFineGridPointsIds
                                                                                   orderedKptsWithIds,
                                                                                   ret[retIdx][iter].m_mainIdx,
                                                                                   orderedFineGridPointsIds);
                        for (size_t i = 0; i < ret[retIdx][iter].m_keypointsWithIds.size(); i++)
                        {
                            ret[retIdx][iter].m_keypointsInImage[i] = ret[retIdx][iter].m_keypointsWithIds[i].m_point;
                        }
                        warpPerspectivePts(ret[retIdx][0].m_HCrop.inv(), ret[retIdx][iter].m_keypointsInImage);
                        m_timing.end("stage2_reorderpointswithmainidx", m_decodeEngine->getCudaStream());
#ifdef DEBUG
                        for (const auto &r : orderedFineGridPointsIds)
                        {
                            std::cout << " O : " << r.m_point.x << " " << r.m_point.y << " " << r.m_id << std::endl;
                        }
                        for (const auto &r : ret[retIdx].m_keypointsWithIds)
                        {
                            std::cout << " R : " << r.m_point.x << " " << r.m_point.y << " " << r.m_id << std::endl;
                        }
                        for (const auto &r : orderedKptsWithIds)
                        {
                            std::cout << " K : " << r.m_point.x << " " << r.m_point.y << " " << r.m_id << std::endl;
                        }
                        for (const auto &kg : stage2KeypointGroups[retIdx])
                        {
                            kg.print();
                        }
                        for (const auto &c : stage2Corners[retIdx])
                        {
                            std::cout << c.x << " " << c.y << std::endl;
                        }
                        
                        cv::Mat debugImg = m_decodeEngine->getDebugImage(ii);
                        for (const auto &kg : stage2KeypointGroups[retIdx])
                        {
                            cv::circle(debugImg,
                                       cv::Point2d(kg.m_keypoint.x, kg.m_keypoint.y),
                                       3,
                                       (kg.m_label < 0) ? cv::Scalar(128, 0, 0) : (kg.m_label == 0) ? cv::Scalar(0, 0, 128)
                                                                                                    : cv::Scalar(0, 128, 0));
                        }
                        if (stage2Corners[retIdx].size() >= 4)
                        {
                            const std::array<float2, 4> orderedCorners = {stage2Corners[retIdx][0],
                                                                          stage2Corners[retIdx][1],
                                                                          stage2Corners[retIdx][3],
                                                                          stage2Corners[retIdx][2]};
                            cv::line(debugImg, cv::Point2f(orderedCorners[0].x, orderedCorners[0].y), cv::Point2f(orderedCorners[1].x, orderedCorners[1].y), cv::Scalar(0, 128, 0), 2);
                            cv::line(debugImg, cv::Point2f(orderedCorners[1].x, orderedCorners[1].y), cv::Point2f(orderedCorners[2].x, orderedCorners[2].y), cv::Scalar(0, 128, 0), 2);
                            cv::line(debugImg, cv::Point2f(orderedCorners[2].x, orderedCorners[2].y), cv::Point2f(orderedCorners[3].x, orderedCorners[3].y), cv::Scalar(0, 128, 0), 2);
                            cv::line(debugImg, cv::Point2f(orderedCorners[3].x, orderedCorners[3].y), cv::Point2f(orderedCorners[0].x, orderedCorners[0].y), cv::Scalar(0, 128, 0), 2);
                        }
                        std::stringstream s;
                        s << ret[retIdx].m_tagId;
                        cv::putText(debugImg, s.str(), cv::Point(5, 35), 0, 1.5, cv::Scalar(0, 255, 255), 2);
                        s.str("");
                        s << "Debug_Iter" << iter << "Idx" << retIdx;
                        cv::imshow(s.str().c_str(), debugImg);
#endif
                    }
                    else
                    {
                        ret[retIdx][iter].m_isValid = false;
                    }
                }
                retIdx += 1;
            }
        }
    }
#ifdef DEBUG
    cv::waitKey(20);
#endif
    return ret;
}

template <class MARKER_DICT, size_t GRID_SIZE>
void STagDecoder<MARKER_DICT, GRID_SIZE>::fillEmptyIds(std::array<PointsAndIDs, (GRID_SIZE + 2) * (GRID_SIZE + 2)> &orderedFineGridPointsIds,
                                                       const tcb::span<const Stage2KeypointGroup> &fineGridPointsWithIdsCandidates) const
{
    for (auto &kpt1 : orderedFineGridPointsIds)
    {
        if (kpt1.m_id == -1)
        {
            double minDist = 1000.;
            for (const auto &kpt2 : fineGridPointsWithIdsCandidates)
            {
                const auto dist = hypot(kpt1.m_point.x - kpt2.m_keypoint.x, kpt1.m_point.y - kpt2.m_keypoint.y);
                if (dist < minDist)
                {
                    minDist = dist;
                    kpt1.m_id = kpt2.m_label;
                }
            }
        }
    }
}

template <class MARKER_DICT, size_t GRID_SIZE>
void STagDecoder<MARKER_DICT, GRID_SIZE>::setMinGridMatchRatio(const double minGridMatchRatio)
{
    m_minGridMatchRatio = minGridMatchRatio;
}
template <class MARKER_DICT, size_t GRID_SIZE>
double STagDecoder<MARKER_DICT, GRID_SIZE>::getMinGridMatchRatio(void) const
{
    return m_minGridMatchRatio;
} 

template <class MARKER_DICT, size_t GRID_SIZE>
ushort2 STagDecoder<MARKER_DICT, GRID_SIZE>::getModelSize(void) const
{
    auto inputDim = m_decodeEngine->getInputDims()[0];
    return ushort2{inputDim.d[2], inputDim.d[3]};
}

#include "deeptag_ros/marker_dict.h"
template class STagDecoder<ArucoMarkerDict<4>, 4>;
template class STagDecoder<ArucoMarkerDict<5>, 5>;
template class STagDecoder<ArucoMarkerDict<6>, 6>;