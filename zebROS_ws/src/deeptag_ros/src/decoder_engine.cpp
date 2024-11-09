#include "deeptag_ros/decoder_engine.h"
#include <cuda_runtime.h>                  // for cudaFreeHost, cudaMallocHost
#include <opencv2/core/hal/interface.h>    // for CV_64FC1
#include <algorithm>                       // for min
#include <opencv2/calib3d.hpp>             // for findHomography
#include <opencv2/core/mat.inl.hpp>        // for Mat::at, _InputArray::_Input...
#include "deeptag_ros/cuda_utils.h"        // for cudaSafeCall
#include "deeptag_ros/gpu_image_wrapper.h" // for GpuImageWrapper
#include "deeptag_ros/image_format.h"      // for imageFormat
#include "deeptag_ros/span.hpp"            // for span
#include "vector_types.h"                  // for float2

// #define DEBUG
#ifdef DEBUG
#include <iostream>                        // for operator<<, endl, basic_ostream
#include <opencv2/core/cvstd.inl.hpp>      // for operatoro<
#include <opencv2/highgui.hpp>             // for imshow, waitKey
static void showDebugImage(cv::Mat &hR, cv::Mat &hG, cv::Mat &hB, const std::string &windowName)
{
    std::vector<cv::Mat> channels;
    channels.push_back(hR);
    channels.push_back(hG);
    channels.push_back(hB);
    cv::Mat fin_img;
    cv::merge(channels, fin_img);

    cv::imshow(windowName, fin_img);
    cv::waitKey(0);
}

static void readDebugImageChannel(const std::string &fileName, cv::Mat &mat)
{
    std::ifstream is(fileName);
    std::string line;
    std::getline(is, line); // skip header

    do
    {
        std::getline(is, line);
        std::stringstream ss(line);

        int row;
        ss >> row;
        int col = 0;
        float val;
        char comma;
        ss >> comma;
        while (ss >> val)
        {
            mat.at<float>(row, col) = val;
            (reinterpret_cast<float *>(mat.data))[row * mat.cols + col] = val;
            // printf("row = %d, col = %d, val = %f\n", row, col, val);
            col++;
            ss >> comma;
        }
    } while (is.good());
}
#endif

DecoderEngineCalibrator::DecoderEngineCalibrator(int32_t batchSize, int32_t inputW, int32_t inputH, cudaStream_t stream, const std::string &calibDataDirPath, const std::string &calibTableName, const std::string &inputBlobName,
                                                 bool readCache)
    : Int8EntropyCalibrator2(batchSize, inputW, inputH, stream, calibDataDirPath, calibTableName, inputBlobName, readCache)
{
}

void DecoderEngineCalibrator::blobFromGpuImageWrappers(const std::vector<GpuImageWrapper> &batchInput)
{
    // Need a function which just flattens batchInput without any other processing
#if 0
    std::vector<std::vector<double2>> rois; // TODO
    rois.emplace_back(double2{static_cast<double>(m_inputH), static_cast<double>(m_inputW)});
    blobFromGpuMatsSSD(batchInput[0], rois, m_deviceInput, m_cudaStream);
#endif
}

DecoderEngine::DecoderEngine(const Options &options)
    : Engine(options)
{
    setUseCudaGraph(false);
    // Set this up as pinned memory so async copy of it is actually async
    m_hH.resize(m_options.maxBatchSize);
    for (int32_t i = 0; i < m_options.maxBatchSize; i++)
    {
        cudaSafeCall(cudaMallocHost(&(m_hH[i]), 9 * sizeof(float)));
    }
}

DecoderEngine::~DecoderEngine()
{
    for (auto &h : m_hH)
    {
        cudaSafeCall(cudaFreeHost(h));
    }
    for (auto &preprocStream : m_preprocCudaStreams)
    {
        cudaSafeCall(cudaStreamDestroy(preprocStream));
    }
    for (auto &preprocEvent : m_preprocCudaEvents)
    {
        cudaSafeCall(cudaEventDestroy(preprocEvent));
    }
}

bool DecoderEngine::loadNetwork()
{
    if (!Engine::loadNetwork())
    {
        return false;
    }

    m_decoderPreprocess.resize(m_options.maxBatchSize);
    m_preprocCudaStreams.resize(m_options.maxBatchSize);
    m_preprocCudaEvents.resize(m_options.maxBatchSize);
    for (int32_t i = 0; i < m_options.maxBatchSize; i++)
    {
        cudaSafeCall(cudaStreamCreate(&m_preprocCudaStreams[i]));
        cudaSafeCall(cudaEventCreate(&m_preprocCudaEvents[i]));
    }
    return true;
}

// Identify which regions of the original image in stage 1
// to extract and decode in stage 2.  This is kind of a 
// hack to allow the base class runInference to be run as
// is. Code sets the RoIs to use with this method, calls
// runInference, then gets the H matrix for each RoI
// by calling getH below.
void DecoderEngine::setROIs(const tcb::span<const std::array<cv::Point2d, 4>> &rois)
{
    m_rois.clear();
    for (const auto &r : rois)
    {
        m_rois.push_back(r);
    }
    m_Hs.clear();
}

cv::Mat DecoderEngine::getH(const size_t idx) const
{
    return m_Hs[idx];
}

// Input image is stage 1 GPU input image, before being preprocessed, resized, whatever
// Output blobs are crops from those images, mapping from RoI in the input image to a 3,256,256 crop
// hopefully holding a valid tag
// Note, this can potentially be broken up into multiple batches (if there are more RoIs than
// max batch size) so we'll need state to start from the correct RoI index
// each pass
void DecoderEngine::blobFromGpuImageWrappers(const std::vector<GpuImageWrapper> &batchInput, size_t inputIdx)
{
#ifdef DEBUG
    static int callNum = 0;
#endif
    const size_t outputHW = batchInput[0].cols(); // This assumes a square image
    const size_t imgSize = outputHW * outputHW * batchInput[0].channels();
    const size_t thisBatchSize = std::min(m_rois.size(), static_cast<size_t>(m_options.maxBatchSize));
    //std::cout << "thisBatchSize = " << thisBatchSize << std::endl;
    // Get crop images ordered corners
    // Map the input tag to fixed positions in the output
    // image, leaving some room for a border
    cv::Mat outputRoi(4, 2, CV_64FC1);
    constexpr double borderRatio = 0.15;
    double borderWidth = outputHW * borderRatio;
    outputRoi.at<double>(0,0) =            borderWidth - 0.5;
    outputRoi.at<double>(0,1) =            borderWidth - 0.5;
    outputRoi.at<double>(1,0) = outputHW - borderWidth - 0.5;
    outputRoi.at<double>(1,1) =            borderWidth - 0.5;
    outputRoi.at<double>(2,0) = outputHW - borderWidth - 0.5;
    outputRoi.at<double>(2,1) = outputHW - borderWidth - 0.5;
    outputRoi.at<double>(3,0) =            borderWidth - 0.5;
    outputRoi.at<double>(3,1) = outputHW - borderWidth - 0.5;

    cv::Mat inputRoi(4, 2, CV_64FC1);
    for (size_t batchIdx = 0; batchIdx < thisBatchSize; batchIdx++)
    {
        // Create a mapping from the input roi (tag corners) in
        // the input image to a fixed position in the output image
        // Assign to a location in the output image with a border
        // to catch the tag corners even if the initial detection
        // is off by a bit.
        for (int i = 0; i < 4; i++)
        {
            inputRoi.at<double>(i, 0) = m_rois[batchIdx][i].x;
            inputRoi.at<double>(i, 1) = m_rois[batchIdx][i].y;
        }

        // Note - we run output->input since this is how the preproc kernel works,
        // but that's acutally the inverse of what the rest of the code
        // needs for H matrix values. So compute H needed for the 
        // kernel extraction and save the inverse of that for the
        // rest of the code to use later on.
        const cv::Mat H = cv::findHomography(outputRoi, inputRoi);
        m_Hs.push_back(H.inv());
#if 0
        std::cout << "input ROI = " << std::endl << "\t" << inputRoi << std::endl;
        std::cout << "output ROI = " << std::endl << "\t" << outputRoi << std::endl;
        std::cout << "H = " << std::endl << "\t" << H << std::endl;
        std::cout << "H.inv() = " << std::endl << "\t" << H.inv() << std::endl;
#endif
        // Use a separate H matrix for each batch entry since they
        // might not be copied to the device before the next
        // iteration writes over the same memory
        for (size_t i = 0; i < 9; i++)
        {
            m_hH[batchIdx][i] = static_cast<float>(H.at<double>(i / 3, i % 3));
        }
        if (batchInput[0].channels() == 1)
        {
            cudaSafeCall(m_decoderPreprocess[batchIdx].decoderPreprocessGray(m_hH[batchIdx],
                                                                             batchInput[0].getDataPtr(),
                                                                             imageFormat::IMAGE_MONO8,
                                                                             batchInput[0].cols(),
                                                                             batchInput[0].rows(),
                                                                             static_cast<float *>(m_buffers[inputIdx]) + batchIdx * imgSize,
                                                                             outputHW,
                                                                             outputHW,
                                                                             float2{0., 1.},
                                                                             m_preprocCudaStreams[batchIdx]));
        }
        else
        {
            cudaSafeCall(m_decoderPreprocess[batchIdx].decoderPreprocessRGB(m_hH[batchIdx],
                                                                            batchInput[0].getDataPtr(),
                                                                            imageFormat::IMAGE_RGB8,
                                                                            batchInput[0].cols(),
                                                                            batchInput[0].rows(),
                                                                            static_cast<float *>(m_buffers[inputIdx]) + batchIdx * imgSize,
                                                                            outputHW,
                                                                            outputHW,
                                                                            float2{0., 1.},
                                                                            m_preprocCudaStreams[batchIdx]));
        }
        cudaSafeCall(cudaEventRecord(m_preprocCudaEvents[batchIdx], m_preprocCudaStreams[batchIdx]));
#ifdef DEBUG
        cv::Mat m = getDebugImage(batchIdx);
        std::stringstream s;
        s << "C" << callNum << "B" << batchIdx;
        cv::imshow(s.str().c_str(), m);
        // cv::imwrite(s.str() + ".png", m);
#endif
    }
    for (size_t batchIdx = 0; batchIdx < thisBatchSize; batchIdx++)
    {
        cudaSafeCall(cudaStreamWaitEvent(getCudaStream(), m_preprocCudaEvents[batchIdx]));
    }

#ifdef DEBUG
    callNum += 1;
    const size_t channelStride = outputHW * outputHW;
#if 0
    std::cout << " imgSize = " << imgSize << std::endl;
    cv::Mat hR(outputHW, outputHW, CV_32FC1);
    cv::Mat hG(outputHW, outputHW, CV_32FC1);
    cv::Mat hB(outputHW, outputHW, CV_32FC1);
    cudaSafeCall(cudaMemcpyAsync(hR.data, static_cast<float *>(m_buffers[inputIdx]) + 0 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, getCudaStream()));
    cudaSafeCall(cudaMemcpyAsync(hG.data, static_cast<float *>(m_buffers[inputIdx]) + 1 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, getCudaStream()));
    cudaSafeCall(cudaMemcpyAsync(hB.data, static_cast<float *>(m_buffers[inputIdx]) + 2 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, getCudaStream()));
    cudaSafeCall(cudaStreamSynchronize(getCudaStream()));

#if 0
    for (size_t r = 0; r < 10; r++)
    {
        for (size_t c = 0; c < 10; c++)
        {
            std::cout << "\t" << hR.at<float>(r, c) << " " << hG.at<float>(r, c) << " " << hB.at<float>(r, c) << std::endl;
        }
        std::cout << std::endl;
    }

#endif
    showDebugImage(hR, hG, hB, "cudaSSDPreprocess");
#endif

#if 0
    cv::Mat h0(outputHW, outputHW, CV_32FC1);
    cv::Mat h1(outputHW, outputHW, CV_32FC1);
    cv::Mat h2(outputHW, outputHW, CV_32FC1);
    readDebugImageChannel("/home/ubuntu/deeptag-pytorch/image_torch0.csv", h0);
    readDebugImageChannel("/home/ubuntu/deeptag-pytorch/image_torch1.csv", h1);
    readDebugImageChannel("/home/ubuntu/deeptag-pytorch/image_torch2.csv", h2);

    cudaSafeCall(cudaMemcpyAsync(static_cast<float *>(destBuffer) + 0 * channelStride, h0.data, channelStride * sizeof(float), cudaMemcpyHostToDevice, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(static_cast<float *>(destBuffer) + 1 * channelStride, h2.data, channelStride * sizeof(float), cudaMemcpyHostToDevice, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(static_cast<float *>(destBuffer) + 2 * channelStride, h1.data, channelStride * sizeof(float), cudaMemcpyHostToDevice, cudaStream));
    showDebugImage(h0, h1, h2, "python");
    cudaSafeCall(cudaStreamSynchronize(cudaStream));
#endif
#endif
}

nvinfer1::Dims DecoderEngine::inputDimsFromInputImage(const GpuImageWrapper &gpuImg, const nvinfer1::Dims &modelInputDims)
{
    // Decoder is fixed at 3, 256, 256
    return nvinfer1::Dims{4,
                          {modelInputDims.d[0],
                          static_cast<int32_t>(gpuImg.channels()),
                          256,
                          256}};
}
