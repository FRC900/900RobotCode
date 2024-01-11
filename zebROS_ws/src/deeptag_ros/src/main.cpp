#include <iostream>
#include "opencv2/opencv.hpp"

#include "deeptag_ros/deeptag.h"

#define DEBUG
#include "deeptag_ros/debug.h"

int main(int argc, char *argv[])
{
    // Parse the command line arguments
    const std::string detectOnnxModelPath = "/home/ubuntu/deeptag-pytorch/models/arucotag_roi_detector.onnx";
    const std::string decodeOnnxModelPath = "/home/ubuntu/deeptag-pytorch/models/arucotag_decoder.onnx";

    // Read the input image
    const std::string inputImage = argv[1];
    auto cpuImg = cv::imread(inputImage);
    if (cpuImg.empty())
    {
        throw std::runtime_error("Unable to read image at path: " + inputImage);
    }

    constexpr double tagSizeInMeter = 0.152;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    if (cpuImg.rows <= 720)
    {
        // 720p
        cameraMatrix = (cv::Mat_<double>(3, 3) << 530.11, 0, 643.09, 0, 530.01, 358.5255, 0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1, 8) << -0.0414513, 0.0097052, -0.00014156, 0.000671092, -0.00485765, 0, 0, 0);
    }
    else
    {
        // 1080p
        cameraMatrix = (cv::Mat_<double>(3, 3) << 1060.22, 0, 969.18, 0, 1060.02, 540.051, 0, 0, 1);
        distCoeffs = (cv::Mat_<double>(1, 8) << -0.0414513, 0.0097052, -0.00014156, 0.000671092, -0.00485765, 0, 0, 0);
    }

    DeepTag deepTag{cpuImg,                      // input image, used for image resolution
                    true,                        // tiled detection - config item
                    true,                        // use scaled-down full image in addition to tiles - config item
                    DeepTagType::APRILTAG_16H5,  // tag type - config item
                    cameraMatrix,                // from camera info
                    distCoeffs,                  // from camera info
                    tagSizeInMeter,              // physical tag size - config item
                    detectOnnxModelPath,         // model path - config item?
                    decodeOnnxModelPath};        // model path - config item?

    while(true)
    {
    for (int iteration = 0; iteration < 1; iteration++)
    {
        if (iteration > 10)
        {
            deepTag.setTimingsEnabled(true);
        }

        const auto result = deepTag.runInference(cpuImg);

#ifdef DEBUG
        for (const auto &r : result)
        {
            std::cout << r << std::endl;
        }

        cv::Mat debugImg = cpuImg.clone();
        deepTag.visualize(debugImg, result);
        cv::imshow(inputImage.c_str(), debugImg);
        cv::imwrite(inputImage + "_out.png", debugImg);
        cv::waitKey(50);
#endif
    }
    }

    return 0;
}
