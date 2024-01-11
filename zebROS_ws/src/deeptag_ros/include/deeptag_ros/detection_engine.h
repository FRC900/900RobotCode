#ifndef DETECTION_ENGINE_INC__
#define DETECTION_ENGINE_INC__

#include "engine.h"
class DetectionEngineCalibrator : public Int8EntropyCalibrator2
{
public:
    DetectionEngineCalibrator(int32_t batchSize,
                              int32_t inputW,
                              int32_t inputH,
                              cudaStream_t stream,
                              const std::string &calibDataDirPath,
                              const std::string &calibTableName,
                              const std::string &inputBlobName,
                              bool readCache = true);
    DetectionEngineCalibrator(const DetectionEngineCalibrator &other) = delete;
    DetectionEngineCalibrator(DetectionEngineCalibrator &&other) noexcept = delete;

    DetectionEngineCalibrator &operator=(const DetectionEngineCalibrator &other) = delete;
    DetectionEngineCalibrator &operator=(DetectionEngineCalibrator &&other) noexcept = delete;
    ~DetectionEngineCalibrator() override = default;

private:
    void blobFromGpuImageWrappers(const std::vector<GpuImageWrapper>& batchInput) override;
};

template <size_t NUM_TILES, bool USE_SCALED_IMAGE>
class DetectionEngine : public Engine<DetectionEngineCalibrator>
{
public:
    explicit DetectionEngine(const Options& options);

    DetectionEngine(const DetectionEngine &other) = delete;
    DetectionEngine(DetectionEngine &&other) noexcept = delete;

    DetectionEngine &operator=(const DetectionEngine &other) = delete;
    DetectionEngine &operator=(DetectionEngine &&other) noexcept = delete;

    ~DetectionEngine() override;

    void getTileOffsets(std::array<ushort2, NUM_TILES> &offsets) const;

private:
    nvinfer1::Dims inputDimsFromInputImage(const GpuImageWrapper& batchInput, const nvinfer1::Dims &modelInputDims) override;
    void blobFromGpuImageWrappers(const std::vector<GpuImageWrapper>& batchInput, size_t bufferIdx) override;
    void blobFromGpuMats(const std::vector<GpuImageWrapper> &batchInput,
                         const ushort2 &outputSize,
                         void *destBuffer,
                         cudaStream_t cudaStream);

    static constexpr size_t NUM_SCALED_IMAGES{USE_SCALED_IMAGE ? 1 : 0};
    std::array<ushort2, NUM_TILES> m_tileOffsets;
    std::array<cudaStream_t, NUM_TILES> m_preprocCudaStreams;
    std::array<cudaEvent_t, NUM_TILES> m_preprocCudaEvents;
};

#endif
