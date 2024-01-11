#include "deeptag_ros/deeptag_impls.h"

DeepTag::DeepTag(const cv::Mat &cpuImg,
                 const bool tiledDetection,
                 const bool useScaledImage,
                 const DeepTagType dictionaryType,
                 const cv::Mat &cameraMatrix,
                 const cv::Mat &distCoeffs,
                 const double tagRealSizeInMeter,
                 const std::string &detectOnnxModelPath,
                 const std::string &decodeOnnxModelPath)
{
    // Various c++ nonsense to convert from a variable dictionaryType
    // into code which will make compile-time batch & grid size template
    // params for the implementation
    auto makeImplUnique = [&]<size_t numTiles, bool useScaledImage, size_t gridSize>(void)
    {
        // numTiles == 0 and no scaled image input means there are no
        // inputs at all - that's an invalid combination of template params
        if constexpr ((numTiles > 0) || useScaledImage)
        {
            m_pImpl = std::make_unique<DeepTagImpl<numTiles, useScaledImage, gridSize>>(dictionaryType,
                                                                                        cameraMatrix,
                                                                                        distCoeffs,
                                                                                        tagRealSizeInMeter,
                                                                                        detectOnnxModelPath,
                                                                                        decodeOnnxModelPath);
        }
        else
        {
            throw std::runtime_error("At least one of tiledDetection and useScaledImage must be true");
        }
    };

    auto makeImplScaled = [&useScaledImage, makeImplUnique]<size_t numTiles, size_t gridSize>()
    {
        if (useScaledImage)
        {
            makeImplUnique.operator()<numTiles, true, gridSize>();
        }
        else
        {
            makeImplUnique.operator()<numTiles, false, gridSize>();
        }
    };

    auto makeImpl = [&dictionaryType, makeImplScaled]<size_t numTiles>()
    {
        if (dictionaryType == DeepTagType::APRILTAG_16H5)
        {
            makeImplScaled.operator()<numTiles, 4>();
        }
        else if (dictionaryType == DeepTagType::APRILTAG_25H9)
        {
            makeImplScaled.operator()<numTiles, 5>();
        }
        else if ((dictionaryType == DeepTagType::APRILTAG_36H10) ||
                 (dictionaryType == DeepTagType::APRILTAG_36H11))
        {
            makeImplScaled.operator()<numTiles, 6>();
        }
        else
        {
            throw std::runtime_error("Invalid dictionaryType in DeepTag constructor");
        }
    };

    if (!tiledDetection)
    {
        makeImpl.operator()<0>();
    }
    else if (cpuImg.rows <= 720)
    {
        makeImpl.operator()<4>();
    }
    else
    {
        makeImpl.operator()<9>();
    }
}

DeepTag::~DeepTag() = default;

std::vector<DeepTagResult> DeepTag::runInference(const cv::Mat &cpuImg)
{
    return m_pImpl->runInference(cpuImg);
}

void DeepTag::visualize(cv::Mat &image, const std::vector<DeepTagResult> &results) const
{
    return m_pImpl->visualize(image, results);
}

void DeepTag::setTimingsEnabled(const bool enabled)
{
    return m_pImpl->setTimingsEnabled(enabled);
}