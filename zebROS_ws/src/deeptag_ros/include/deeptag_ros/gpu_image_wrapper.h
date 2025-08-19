#ifndef GPU_IMAGE_WRAPPER_INC__
#define GPU_IMAGE_WRAPPER_INC__
#include <cstddef>
#include "driver_types.h"  // for cudaStream_t

#include "deeptag_ros/image_format.h"
namespace cv
{
    class Mat;
}

class GpuImageWrapper
{
public:
    GpuImageWrapper();
    GpuImageWrapper(const GpuImageWrapper &other) = delete;
    GpuImageWrapper(GpuImageWrapper &&other) noexcept;

    GpuImageWrapper &operator=(const GpuImageWrapper &other) = delete;
    GpuImageWrapper &operator=(GpuImageWrapper &&other) noexcept;

    virtual ~GpuImageWrapper();

    void upload(const cv::Mat &cpuImg, cudaStream_t cudaStream);
    void download(cv::Mat &cpuImg, cudaStream_t cudaStream) const;

    size_t channels(void) const;
    size_t rows(void) const;
    size_t cols(void) const;

    size_t imageSizeInBytes(void) const;

    void *getDataPtr(void) const;
private:

    void reallocateToMatch(const cv::Mat &cpuImg, cudaStream_t);
    static size_t cvTypeBytesPerPixel(int type);
    static size_t bytesPerPixel(const imageFormat format);

    void *m_dataPtr{nullptr};
    size_t m_rows{0};
    size_t m_cols{0};
    int m_cvType{0};
};

#endif
