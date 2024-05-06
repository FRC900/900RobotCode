#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <random> // for Forgy method of initializing k-means for depth detection

// K_MEANS = k-means, Forgy method of initializing (k-means++ would likely be better, but Forgy is working ok for now)
// CONTOURS = find depth by finding contours and taking the lowest value of the biggest contour
// CONTOURS_NON_ADAPTIVE = contours, but using thresholding based on the median instead of adaptive thresholding
enum DepthCalculationAlgorithm { K_MEANS, CONTOURS, CONTOURS_NON_ADAPTIVE, TWO_D_ONLY };

float usefulDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, DepthCalculationAlgorithm algorithm = CONTOURS, bool debug = false, int k = 3, float tolerance = 1e-3);
