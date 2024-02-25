#ifndef ENGINE_H_INC__
#define ENGINE_H_INC__

#include <cstddef>                 // for size_t
#include <cstdint>                 // for int32_t, uint32_t
#include <fstream>                 // for ifstream
#include <map>                     // for map
#include <memory>                  // for unique_ptr
#include <string>                  // for string
#include <vector>                  // for vector

#include <opencv2/core/mat.hpp>    // for Mat

#include "NvInfer.h"               // for IInt8EntropyCalibrator2
#include "NvInferRuntime.h"        // for ICudaEngine, IExecutionContext
#include "NvInferRuntimeCommon.h"  // for Dims, ILogger
#include "driver_types.h"          // for cudaStream_t, CUstream_st, CUgraph...

#include "cuda_event_timing.h"     // for Timings
class GpuImageWrapper;

// Utility methods
namespace Util {
    inline bool doesFileExist(const std::string& filepath) {
        std::ifstream f(filepath.c_str());
        return f.good();
    }

    std::vector<std::string> getFilesInDirectory(const std::string& dirPath);
}

// Precision used for GPU inference
enum class Precision {
    // Full precision floating point value
    FP32,
    // Half prevision floating point value
    FP16,
    // Int8 quantization.
    // Has reduced dynamic range, may result in slight loss in accuracy.
    // If INT8 is selected, must provide path to calibration dataset directory.
    INT8,
};

// Options for the network
struct Options {
    // Precision to use for GPU inference.
    Precision precision = Precision::FP16;
    // If INT8 precision is selected, must provide path to calibration dataset directory.
    std::string calibrationDataDirectoryPath;
    // The batch size to be used when computing calibration data for INT8 inference.
    // Should be set to as large a batch number as your GPU will support.
    int32_t calibrationBatchSize = 128;
    // The batch size which should be optimized for.
    int32_t optBatchSize = 1;
    // Maximum allowable batch size
    int32_t maxBatchSize = 16;
    // GPU device index
    int deviceIndex = 0;
};

// Class used for int8 calibration
class Int8EntropyCalibrator2 : public nvinfer1::IInt8EntropyCalibrator2 {
public:
    Int8EntropyCalibrator2(int32_t batchSize, int32_t inputW, int32_t inputH, cudaStream_t stream, const std::string& calibDataDirPath, const std::string& calibTableName, const std::string& inputBlobName,
                           bool readCache = true);
    Int8EntropyCalibrator2(const Int8EntropyCalibrator2 &other) = delete;
    Int8EntropyCalibrator2(Int8EntropyCalibrator2 &&other) noexcept = delete;

    Int8EntropyCalibrator2 &operator=(const Int8EntropyCalibrator2 &other) = delete;
    Int8EntropyCalibrator2 &operator=(Int8EntropyCalibrator2 &&other) noexcept = delete;
    ~Int8EntropyCalibrator2() override;
    // Abstract base class methods which must be implemented
    int32_t getBatchSize () const noexcept override;
    bool getBatch (void *bindings[], char const *names[], int32_t nbBindings) noexcept override;
    void const * readCalibrationCache (std::size_t &length) noexcept override;
    void writeCalibrationCache (void const *ptr, std::size_t length) noexcept override;
protected:
    const int32_t m_inputW;
    const int32_t m_inputH;
    void* m_deviceInput{nullptr};
    cudaStream_t m_cudaStream;
private:
    virtual void blobFromGpuImageWrappers(const std::vector<GpuImageWrapper>& batchInput) = 0;
    const int32_t m_batchSize;
    int32_t m_imgIdx{0};
    std::vector<std::string> m_imgPaths;
    const std::string m_calibTableName;
    const std::string m_inputBlobName;
    const bool m_readCache;
    std::vector<char> m_calibCache;
};

// Class to extend TensorRT logger
class Logger : public nvinfer1::ILogger {
    void log (Severity severity, const char* msg) noexcept override;
};

template <class CALIBRATOR>
class Engine {
public:
    explicit Engine(const Options& options);

    Engine(const Engine &other) = delete;
    Engine(Engine &&other) noexcept = delete;

    Engine &operator=(const Engine &other) = delete;
    Engine &operator=(Engine &&other) noexcept = delete;

    virtual ~Engine();
    // Build the network
    bool build(const std::string &modelPath, const std::string &onnxModelFilename);
    // Load and prepare the network for inference
    virtual bool loadNetwork();
    // Run inference.
    // Input format [input][batch][GpuImageWrapper]
    // Output format [batch][output][feature_vector]
    bool runInference(const std::vector<std::vector<GpuImageWrapper>>& inputs, std::vector<std::vector<std::vector<float>>>& featureVectors, bool &buffersResized, const int32_t forceBatchSize = -1);

    [[nodiscard]] const std::vector<nvinfer1::Dims>& getInputDims() const { return m_inputDims; };
    [[nodiscard]] const std::vector<nvinfer1::Dims>& getContextInputDims() const { return m_contextInputDims; };
    [[nodiscard]] const std::vector<nvinfer1::Dims>& getOutputDims() const { return m_outputDims ;};

    // Utility method for transforming triple nested output array into 2D array
    // Should be used when the output batch size is 1, but there are multiple output feature vectors
    static void transformOutput(std::vector<std::vector<std::vector<float>>>& input, std::vector<std::vector<float>>& output);

    // Utility method for transforming triple nested output array into single array
    // Should be used when the output batch size is 1, and there is only a single output feature vector
    static void transformOutput(std::vector<std::vector<std::vector<float>>>& input, std::vector<float>& output);

    cudaStream_t getCudaStream(void) { return m_inferenceCudaStream;}

    const float *getBufferByName(const std::string &name, const size_t index = 0) const;
    const nvinfer1::Dims getContextDim(const std::string &name) const;

    void setTimingEnable(const bool enable);
    void setUseCudaGraph(const bool useCudaGraph);
    cv::Mat getDebugImage(const size_t imageIdx);

protected:
    // Holds pointers to the input and output GPU buffers
    std::vector<void *> m_buffers;
    // Model-specified input and output dims.  This can include -1 for
    // a dynamically-sized dimension
    std::vector<nvinfer1::Dims> m_inputDims;
    const Options m_options;
private:
    // Given the model input size and an input, return the fully defined dimensions for the input
    // For cases where there are no dynamic dimensions, this should just return modelInputDims.
    // Otherwise, do whatever calcs are needed to fix the dynamic dimension
    virtual nvinfer1::Dims inputDimsFromInputImage(const GpuImageWrapper& batchInput, const nvinfer1::Dims &modelInputDims) = 0;
    virtual void blobFromGpuImageWrappers(const std::vector<GpuImageWrapper>& batchInput, size_t bufferIdx) = 0;
    bool allocateInputOutputTensors(bool &buffersResized);
    // Converts the engine options into a string
    std::string serializeEngineOptions(const Options& options, const std::string &modelPath, const std::string& onnxModelFilename);

    void getDeviceNames(std::vector<std::string>& deviceNames) const;

    std::vector<uint32_t> m_outputLengthsFloat{};
    // Current input dims for the most recently requested infer call
    std::vector<nvinfer1::Dims> m_contextInputDims;
    std::vector<nvinfer1::Dims> m_outputDims;
    std::vector<nvinfer1::Dims> m_prevTensorDims;
    std::vector<std::string> m_inputTensorNames;
    std::vector<std::string> m_outputTensorNames;
    std::map<std::string, size_t> m_tensorNameToBufferMap;

    cudaStream_t m_inferenceCudaStream;
    cudaGraph_t m_cudaGraph;
    cudaGraphExec_t m_cudaGraphInstance;
    bool m_needNewCudaGraph{true};
    int m_lastBatchSize{0};
    bool m_useCudaGraph{false};
    // Must keep IRuntime around for inference, see: https://forums.developer.nvidia.com/t/is-it-safe-to-deallocate-nvinfer1-iruntime-after-creating-an-nvinfer1-icudaengine-but-before-running-inference-with-said-icudaengine/255381/2?u=cyruspk4w6
    std::unique_ptr<nvinfer1::IRuntime> m_runtime{nullptr};
    std::unique_ptr<Int8EntropyCalibrator2> m_calibrator{nullptr};
    std::unique_ptr<nvinfer1::ICudaEngine> m_engine{nullptr};
    std::unique_ptr<nvinfer1::IExecutionContext> m_context{nullptr};
    Logger m_logger;
    std::string m_engineName;
    std::unique_ptr<Timings> m_timings{};
};

#endif