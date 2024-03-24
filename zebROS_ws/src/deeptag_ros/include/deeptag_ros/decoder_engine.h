#ifndef DECODER_ENGINE_INC__
#define DECODER_ENGINE_INC__

#include "span.hpp"

#include "decoder_preprocess.h"
#include "engine.h"

class DecoderEngineCalibrator : public Int8EntropyCalibrator2
{
public:
    DecoderEngineCalibrator(int32_t batchSize,
                            int32_t inputW,
                            int32_t inputH,
                            cudaStream_t stream,
                            const std::string &calibDataDirPath,
                            const std::string &calibTableName,
                            const std::string &inputBlobName,
                            bool readCache = true);
    DecoderEngineCalibrator(const DecoderEngineCalibrator &other) = delete;
    DecoderEngineCalibrator(DecoderEngineCalibrator &&other) noexcept = delete;

    DecoderEngineCalibrator &operator=(const DecoderEngineCalibrator &other) = delete;
    DecoderEngineCalibrator &operator=(DecoderEngineCalibrator &&other) noexcept = delete;
    ~DecoderEngineCalibrator() override = default;
private:
    void blobFromGpuImageWrappers(const std::vector<GpuImageWrapper>& batchInput) override;
};

class DecoderEngine : public Engine<DecoderEngineCalibrator>
{
public:
    explicit DecoderEngine(const Options& options);

    DecoderEngine(const DecoderEngine &other) = delete;
    DecoderEngine(DecoderEngine &&other) noexcept = delete;

    DecoderEngine &operator=(const DecoderEngine &other) = delete;
    DecoderEngine &operator=(DecoderEngine &&other) noexcept = delete;

    ~DecoderEngine() override;

    bool loadNetwork() override;

    void setROIs(const tcb::span<const std::array<cv::Point2d, 4>> &rois);
    cv::Mat getH(const size_t idx) const;

private:
    nvinfer1::Dims inputDimsFromInputImage(const GpuImageWrapper& batchInput, const nvinfer1::Dims &modelInputDims) override;
    void blobFromGpuImageWrappers(const std::vector<GpuImageWrapper>& batchInput, size_t bufferIdx) override;

    std::vector<std::array<cv::Point2d, 4>> m_rois; // regions of interest of the stage 1 image to decode as tags in this batch

    // Decoder preprocess has internal state which needs to be
    // kept separate per batch input when running preproc in parallel
    std::vector<DecoderPreprocess> m_decoderPreprocess;
    std::vector<cv::Mat> m_Hs;
    // Holding space to set up async H2D memcpys of H matrix
    // to device. Need one 9-entry slot (3x3 matrix) per batch
    // size. We can't reuse between batches becuase this H2D
    // copy is async, so we need to know each array content 
    // is preserved until the async copy completes some time
    // in the indefinite future.
    std::vector<float *>m_hH{};

    // Batch image preprocessing can run in parallel, so allocate a stream
    // per max batch value. Also use an event per batch entry to let
    // the main stream sync back up when all are completed
    std::vector<cudaStream_t> m_preprocCudaStreams;
    std::vector<cudaEvent_t> m_preprocCudaEvents;
};

#endif
