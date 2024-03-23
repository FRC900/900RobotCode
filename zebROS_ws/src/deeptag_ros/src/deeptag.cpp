#include "deeptag_ros/deeptag_impls.h"

DeepTag::DeepTag(const cv::Size &imgSize,
                 const bool tiledDetection,
                 const bool useScaledImage,
                 const DeepTagType dictionaryType,
                 const cv::Mat &cameraMatrix,
                 const cv::Mat &distCoeffs,
                 const double tagRealSizeInMeter,
                 const std::string &modelPath,
                 const std::string &detectOnnxModelFileName,
                 const std::string &decodeOnnxModelFileName)
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
                                                                                        modelPath,
                                                                                        detectOnnxModelFileName,
                                                                                        decodeOnnxModelFileName);
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
    else //if (imgSize.height <= 720)
    {
        makeImpl.operator()<4>();
    }
    // else
    // {
    //     makeImpl.operator()<9>();
    // }
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

void DeepTag::visualizeStage1Grid(cv::Mat &image)
{
    return m_pImpl->visualizeStage1Grid(image);
}
void DeepTag::visualizeStage1SSD(cv::Mat &image)
{
    return m_pImpl->visualizeStage1SSD(image);
}
void DeepTag::saveInputImage(void)
{
    return m_pImpl->saveInputImage();
}
void DeepTag::setTimingsEnabled(const bool enabled)
{
    return m_pImpl->setTimingsEnabled(enabled);
}

// Expose a bunch of config params for dynamic reconfigure
void DeepTag::setCornerMinCenterScore(const double cornerMinCenterScore)
{
    m_pImpl->setCornerMinCenterScore(cornerMinCenterScore);
}
void DeepTag::setSSDMinCenterScore(const double ssdMinCenterScore)
{
    m_pImpl->setSSDMinCenterScore(ssdMinCenterScore);
}
void DeepTag::setGridGrouperSigma(const int gridGrouperSigma)
{
    m_pImpl->setGridGrouperSigma(gridGrouperSigma);
}
void DeepTag::setSSDGrouperSigma(const int ssdGrouperSigma)
{
    m_pImpl->setSSDGrouperSigma(ssdGrouperSigma);
}
double DeepTag::getCornerMinCenterScore(void) const 
{
    return m_pImpl->getCornerMinCenterScore();
}
double DeepTag::getSSDMinCenterScore(void) const 
{
    return m_pImpl->getSSDMinCenterScore();
}
int DeepTag::getGridGrouperSigma(void) const 
{
    return m_pImpl->getGridGrouperSigma();
}
int DeepTag::getSSDGrouperSigma(void) const
{
    return m_pImpl->getSSDGrouperSigma();
}

void DeepTag::setNMSConfidenceThreshold(const double nms_confidence_threshold)
{
    return m_pImpl->setNMSConfidenceThreshold(nms_confidence_threshold);
}
void DeepTag::setNMSNMSThreshold(const double nms_nms_threshold)
{
    return m_pImpl->setNMSNMSThreshold(nms_nms_threshold);
}
double DeepTag::getNMSConfidenceThreshold(void) const
{
    return m_pImpl->getNMSConfidenceThreshold();
}
double DeepTag::getNMSNMSThreshold(void) const
{
    return m_pImpl->getNMSNMSThreshold();
}

void DeepTag::setMinGridMatchRatio(const double minGridMatchRatio)
{
    m_pImpl->setMinGridMatchRatio(minGridMatchRatio);
}
double DeepTag::getMinGridMatchRatio(void) const
{
    return m_pImpl->getMinGridMatchRatio();
}