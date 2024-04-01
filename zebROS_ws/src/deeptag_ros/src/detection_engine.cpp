#include "deeptag_ros/detection_engine.h"
#include <algorithm>           // for min
#include "deeptag_ros/cuda_image_tile.h"
#include "deeptag_ros/cuda_ssd_preprocess.h" // for cudaSSDPreprocessRGB
#include "deeptag_ros/gpu_image_wrapper.h"   // for GpuImageWrapper
#include "deeptag_ros/image_format.h"        // for imageFormat
#include "vector_types.h"                    // for float2
#include "deeptag_ros/cuda_utils.h"

#ifdef DEBUG
#include <fstream>                  // for operator>>, basic_istream, ifstream
#include <opencv2/core.hpp>         // for merge
#include <opencv2/core/mat.hpp>     // for Mat
#include <opencv2/core/mat.inl.hpp> // for _InputArray::_InputArray, _Input...
#include <opencv2/highgui.hpp>      // for imshow, waitKey
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
            //(reinterpret_cast<float *>(mat.data))[row * mat.cols + col] = val;
            // printf("row = %d, col = %d, val = %f\n", row, col, val);
            col++;
            ss >> comma;
        }
    } while (is.good());
}
#endif

DetectionEngineCalibrator::DetectionEngineCalibrator(int32_t batchSize, int32_t inputW, int32_t inputH, cudaStream_t stream, const std::string &calibDataDirPath, const std::string &calibTableName, const std::string &inputBlobName,
                                                     bool readCache)
    : Int8EntropyCalibrator2(batchSize, inputW, inputH, stream, calibDataDirPath, calibTableName, inputBlobName, readCache)
{
}

void DetectionEngineCalibrator::blobFromGpuImageWrappers(const std::vector<GpuImageWrapper> &batchInput)
{
    // Input is a vector of bgr images, in [H][W][C] format
    // Output should be a flat buffer in [Batch][C][H][W] format
    //   Channel order stays the same, see below
    //   Pixel values changed from 0..255 to 0.0 .. 1.0
    const ushort2 batchInputSize{static_cast<ushort>(batchInput[0].cols()), 
                                 static_cast<ushort>(batchInput[0].rows())};
    const size_t imgSize = m_inputH * m_inputW * 3;
    for (size_t i = 0; i < batchInput.size(); i++)
    {
        // Note, this isn't how SSD actually preprocesses data
        // we're passing in a BGR image as if it were an RGB one
        // ... this is a hack to prevent the channels from
        // being swapped. This matches the python implementation
        // which might be a bug, but since we're using their
        // trained model we're stuck with their channel input order
        cudaSafeCall(cudaSSDPreprocessRGB(batchInput[i].getDataPtr(),
                                          IMAGE_RGB8,
                                          batchInputSize,
                                          static_cast<float *>(m_deviceInput) + i * imgSize,
                                          ushort2{static_cast<unsigned short>(m_inputW),
                                                  static_cast<unsigned short>(m_inputH)},
                                          float2{0., 1.},
                                          m_cudaStream));
    }
#ifdef DEBUG
    const size_t channelStride = m_inputW * m_inputH;
#if 0
    std::cout << " imgSize = " << imgSize << std::endl;
    cv::Mat hR(m_inputH, m_inputW, CV_32FC1);
    cv::Mat hG(m_inputH, m_inputW, CV_32FC1);
    cv::Mat hB(m_inputH, m_inputW, CV_32FC1);
    cudaSafeCall(cudaMemcpyAsync(hR.data, static_cast<float *>(destBuffer) + 0 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(hG.data, static_cast<float *>(destBuffer) + 1 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(hB.data, static_cast<float *>(destBuffer) + 2 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaStreamSynchronize(cudaStream));

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
    cv::Mat h0(m_inputH, m_inputW, CV_32FC1);
    cv::Mat h1(m_inputH, m_inputW, CV_32FC1);
    cv::Mat h2(m_inputH, m_inputW, CV_32FC1);
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

template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>::DetectionEngine(const Options &options)
    : Engine(options)
{
    setUseCudaGraph(true);

    for (auto &preprocStream : m_preprocCudaStreams)
    {
        cudaSafeCall(cudaStreamCreate(&preprocStream));
    }
    for (auto &preprocEvent : m_preprocCudaEvents)
    {
        cudaSafeCall(cudaEventCreate(&preprocEvent));
    }
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>::~DetectionEngine() 
{
    for (auto &preprocStream : m_preprocCudaStreams)
    {
        cudaSafeCall(cudaStreamDestroy(preprocStream));
    }
    for (auto &preprocEvent : m_preprocCudaEvents)
    {
        cudaSafeCall(cudaEventDestroy(preprocEvent));
    }
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
void DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>::blobFromGpuImageWrappers(const std::vector<GpuImageWrapper> &batchInput, size_t inputIdx)
{
    const auto inputDim = inputDimsFromInputImage(batchInput[0], m_inputDims[inputIdx]);
    const ushort2 inputSize{static_cast<unsigned short>(inputDim.d[3]), static_cast<unsigned short>(inputDim.d[2])};
    blobFromGpuMats(batchInput, inputSize, m_buffers[inputIdx], getCudaStream());
}

template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
nvinfer1::Dims DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>::inputDimsFromInputImage(const GpuImageWrapper &gpuImg, const nvinfer1::Dims &modelInputDims)
{
    constexpr int stride = 64;
    auto imageH = static_cast<double>(gpuImg.rows());
    auto imageW = static_cast<double>(gpuImg.cols());
    // std::cout << "imageH = " << imageH << " imageW = " << imageW << std::endl;
    // std::cout << "modelInputDims = "<< modelInputDims.d[0] << " " << modelInputDims.d[1] << " " << modelInputDims.d[2] << " " << modelInputDims.d[3] << std::endl;

    // Input image needs to be resized to fit in the fixed model input height (d[2])
    const double detectScale = std::max(static_cast<double>(modelInputDims.d[2]), std::min(static_cast<double>(modelInputDims.d[2]), std::min(imageH, imageW))) / std::min(imageH, imageW);
    // std::cout << "detectScale = " << detectScale << std::endl;

    // Resize so the image is the correct size
    imageH *= detectScale;
    imageW *= detectScale;

    // Now round the sizes down to the model stride size
    // Preserve input channel size and batch size
    return nvinfer1::Dims{4,
                          {modelInputDims.d[0],
                           modelInputDims.d[1],
                           (static_cast<int>(imageH) / stride) * stride,
                           (static_cast<int>(imageW) / stride) * stride}};
}
template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
void DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>::getTileOffsets(std::array<ushort2, NUM_TILES> &offsets) const
{
    if constexpr (NUM_TILES > 0)
    {
        offsets = m_tileOffsets;
    }
}

// Input is a vector of bgr images, in [H][W][C] format
// Output should be a flat buffer in [Batch][C][H][W] format
//   Channel order stays the same, see below
//   Pixel values changed from 0..255 to 0.0 .. 1.0
// The batch inputs should be sampled from the starting coords 
// laid out in the inputTile vector.
template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
void DetectionEngine<NUM_TILES, USE_SCALED_IMAGE>::blobFromGpuMats(const std::vector<GpuImageWrapper> &batchInput,
                                                                   const ushort2 &outputSize,
                                                                   void *destBuffer,
                                                                   cudaStream_t cudaStream)
{
    const size_t imgSize = static_cast<size_t>(outputSize.x) * outputSize.y * 3;
    const ushort2 batchInputSize{static_cast<ushort>(batchInput[0].cols()), 
                                 static_cast<ushort>(batchInput[0].rows())};

    if constexpr (NUM_TILES > 0)
    {
        if ((batchInputSize.x < outputSize.x) || (batchInputSize.y < outputSize.y))
        {
            for (auto &tileOffset : m_tileOffsets)
            {
                tileOffset = ushort2{0, 0};
            }
        }
        else
        {
            if constexpr (NUM_TILES == 4)
            {
                m_tileOffsets[0] = ushort2{0, 0};
                m_tileOffsets[1] = ushort2{static_cast<unsigned short>(batchInputSize.x - outputSize.x), 0};
                m_tileOffsets[2] = ushort2{0, static_cast<unsigned short>(batchInputSize.y - outputSize.y)};
                m_tileOffsets[3] = ushort2{static_cast<unsigned short>(batchInputSize.x - outputSize.x), static_cast<unsigned short>(batchInputSize.y - outputSize.y)};
            }
            if constexpr (NUM_TILES == 9)
            {
                const std::array<ushort, 3> xs{0, static_cast<unsigned short>(batchInputSize.x / 2 - outputSize.x / 2), static_cast<unsigned short>(batchInputSize.x - outputSize.x)};
                const std::array<ushort, 3> ys{0, static_cast<unsigned short>(batchInputSize.y / 2 - outputSize.y / 2), static_cast<unsigned short>(batchInputSize.y - outputSize.y)};
                size_t idx = 0;
                for (auto &y : ys)
                {
                    for (auto &x : xs)
                    {
                        m_tileOffsets[idx++] = ushort2{x, y};
                    }
                }
            }
        }
        // Run these in parallel.  Each tile has a separate cuda stream
        // Create an event when each preproc is finished. Have the main stream wait
        // on those events to trigger (meaning the previous cuda work is finished)
        // before proceeding.
        for (size_t tile = 0; tile < NUM_TILES; tile++)
        {
            cudaSafeCall(cudaImageTileRGB(batchInput[0].getDataPtr(),
                                          IMAGE_RGB8,
                                          batchInputSize,
                                          static_cast<float *>(destBuffer) + tile * imgSize,
                                          outputSize,
                                          m_tileOffsets[tile],
                                          float2{0., 1.},
                                          m_preprocCudaStreams[tile]));
            cudaSafeCall(cudaEventRecord(m_preprocCudaEvents[tile], m_preprocCudaStreams[tile]));
        }
    }
    if constexpr (USE_SCALED_IMAGE)
    {
        // Note, this isn't how SSD actually preprocesses data
        // we're passing in a BGR image as if it were an RGB one
        // ... this is a hack to prevent the channels from
        // being swapped. This matches the python implementation
        // which might be a bug, but since we're using their
        // trained model we're stuck with their channel input order
        // Run this in the main detection stream
        cudaSafeCall(cudaSSDPreprocessRGB(batchInput[0].getDataPtr(),
                                          IMAGE_RGB8,
                                          batchInputSize,
                                          static_cast<float *>(destBuffer) + NUM_TILES * imgSize,
                                          outputSize,
                                          float2{0., 1.},
                                          getCudaStream()));
    }
    // Sync up all of the streams used for creating tiled GPU inputs
    for (size_t tile = 0; tile < NUM_TILES; tile++)
    {
        cudaSafeCall(cudaStreamWaitEvent(getCudaStream(), m_preprocCudaEvents[tile]));
    }
#ifdef DEBUG
    for (size_t tile = 0; tile < (NUM_TILES + NUM_SCALED_IMAGES); tile++)
    {
        static int callNum = 0;
        cv::Mat m = getDebugImage(tile);
        std::stringstream s;
        s << "C" << callNum << "I" << tile;
        cv::imshow(s.str().c_str(), m);
        cv::imwrite(s.str() + ".png", m);
    }
    cv::waitKey(0);
#endif
#if 0
    const size_t channelStride = outputWidth * outputHeight;
    std::cout << " imgSize = " << imgSize << std::endl;
    cv::Mat hR(outputHeight, outputWidth, CV_32FC1);
    cv::Mat hG(outputHeight, outputWidth, CV_32FC1);
    cv::Mat hB(outputHeight, outputWidth, CV_32FC1);
    cudaSafeCall(cudaMemcpyAsync(hR.data, static_cast<float *>(destBuffer) + 0 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(hG.data, static_cast<float *>(destBuffer) + 1 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(hB.data, static_cast<float *>(destBuffer) + 2 * channelStride, channelStride * sizeof(float), cudaMemcpyDeviceToHost, cudaStream));
    cudaSafeCall(cudaStreamSynchronize(cudaStream));

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
    cv::Mat h0(outputHeight, outputWidth, CV_32FC1);
    cv::Mat h1(outputHeight, outputWidth, CV_32FC1);
    cv::Mat h2(outputHeight, outputWidth, CV_32FC1);
    readDebugImageChannel("/home/ubuntu/deeptag-pytorch/image_torch0.csv", h0);
    readDebugImageChannel("/home/ubuntu/deeptag-pytorch/image_torch1.csv", h1);
    readDebugImageChannel("/home/ubuntu/deeptag-pytorch/image_torch2.csv", h2);

    cudaSafeCall(cudaMemcpyAsync(static_cast<float *>(destBuffer) + 0 * channelStride, h0.data, channelStride * sizeof(float), cudaMemcpyHostToDevice, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(static_cast<float *>(destBuffer) + 1 * channelStride, h2.data, channelStride * sizeof(float), cudaMemcpyHostToDevice, cudaStream));
    cudaSafeCall(cudaMemcpyAsync(static_cast<float *>(destBuffer) + 2 * channelStride, h1.data, channelStride * sizeof(float), cudaMemcpyHostToDevice, cudaStream));
    showDebugImage(h0, h1, h2, "python");
    cudaSafeCall(cudaStreamSynchronize(cudaStream));
#endif
}

template class DetectionEngine<0, true>;
template class DetectionEngine<4, false>;
template class DetectionEngine<4, true>;
template class DetectionEngine<9, false>;
template class DetectionEngine<9, true>;