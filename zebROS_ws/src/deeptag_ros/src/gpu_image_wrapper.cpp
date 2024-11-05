#include "deeptag_ros/gpu_image_wrapper.h"
#include <cuda_runtime.h>                // for cudaMemcpyAsync, cudaFree
#include <opencv2/core/hal/interface.h>  // for CV_CN_SHIFT, CV_16S, CV_16U
#include <iostream>                      // for operator<<, basic_ostream
#include <opencv2/core/mat.hpp>          // for Mat
#include <opencv2/core/mat.inl.hpp>      // for Mat::type
#include "deeptag_ros/cuda_utils.h"

GpuImageWrapper::GpuImageWrapper() = default;

GpuImageWrapper::GpuImageWrapper(GpuImageWrapper &&other) noexcept
: m_dataPtr{other.m_dataPtr}
, m_rows{other.m_rows}
, m_cols{other.m_cols}
, m_cvType{other.m_cvType}
{
    other.m_dataPtr = nullptr;
}

GpuImageWrapper &GpuImageWrapper::operator=(GpuImageWrapper &&other) noexcept
{
    if (&other != this)
    {
        m_dataPtr = other.m_dataPtr;
        other.m_dataPtr = nullptr;
        m_rows = other.m_rows;
        m_cols = other.m_cols;
        m_cvType = other.m_cvType;
    }
    return *this;
}

GpuImageWrapper::~GpuImageWrapper()
{
    if (m_dataPtr)
    {
        cudaSafeCall(cudaFree(m_dataPtr));
    }
}

void GpuImageWrapper::upload(const cv::Mat &cpuImg, cudaStream_t cudaStream)
{
    reallocateToMatch(cpuImg, cudaStream);

    cudaSafeCall(cudaMemcpyAsync(m_dataPtr, cpuImg.data, imageSizeInBytes(), cudaMemcpyHostToDevice, cudaStream));
}

void GpuImageWrapper::download(cv::Mat &cpuImg, cudaStream_t cudaStream) const
{
    cpuImg.create(m_rows, m_cols, m_cvType);

    cudaSafeCall(cudaMemcpyAsync(cpuImg.data, m_dataPtr, imageSizeInBytes(), cudaMemcpyDeviceToHost, cudaStream));
}

size_t GpuImageWrapper::rows(void) const
{
    return m_rows;
}
size_t GpuImageWrapper::cols(void) const
{
    return m_cols;
}

size_t GpuImageWrapper::channels(void) const
{

    return 1 + (m_cvType >> CV_CN_SHIFT);
}

size_t GpuImageWrapper::bytesPerPixel(const imageFormat format)
{
    switch(format)
    {
    case imageFormat::IMAGE_MONO8:
        return 1;
    case imageFormat::IMAGE_RGB8:
    case imageFormat::IMAGE_BGR8:
        return 3;
    case imageFormat::IMAGE_RGBA8:
    case imageFormat::IMAGE_BGRA8:
        return 4;
    case imageFormat::IMAGE_RGB32F:
    case imageFormat::IMAGE_BGR32F:
        return 3 * sizeof(float);
    case imageFormat::IMAGE_RGBA32F:
    case imageFormat::IMAGE_BGRA32F:
        return 4 * sizeof(float);
    }
    std::cout << "Invalid image format ( " << static_cast<int>(format) << " in " << __PRETTY_FUNCTION__ << std::endl;
    return 0;
}

size_t GpuImageWrapper::cvTypeBytesPerPixel(int type)
{
    unsigned depth = type & CV_MAT_DEPTH_MASK;
    unsigned chans = 1 + (type >> CV_CN_SHIFT);
    switch (depth)
    {
        case CV_8U:
        case CV_8S:
            return chans;
        case CV_16U:
        case CV_16S:
            return chans * 2;
        case CV_32S:
        case CV_32F:
            return chans * 4;
        case CV_64F:
            return chans * 8;
    }
    std::cerr << "Invalid type in " << __PRETTY_FUNCTION__ << std::endl;
    return -1;
}

size_t GpuImageWrapper::imageSizeInBytes(void) const
{
    return m_rows * m_cols * cvTypeBytesPerPixel(m_cvType);
}

void * GpuImageWrapper::getDataPtr(void) const
{
    return m_dataPtr;
}

void GpuImageWrapper::reallocateToMatch(const cv::Mat &cpuImg, cudaStream_t stream)
{
    const auto cpuImgBytesPerPixel = cvTypeBytesPerPixel(cpuImg.type());
    if ((cpuImg.rows == static_cast<int>(m_rows)) &&
        (cpuImg.cols == static_cast<int>(m_cols)) &&
        (cpuImgBytesPerPixel == cvTypeBytesPerPixel(m_cvType)))
    {
        return;
    }
    m_rows = cpuImg.rows;
    m_cols = cpuImg.cols;
    m_cvType = cpuImg.type();
    cudaSafeCall(cudaFreeAsync(m_dataPtr, stream));
    cudaSafeCall(cudaMallocAsync(&m_dataPtr, imageSizeInBytes(), stream));
}
