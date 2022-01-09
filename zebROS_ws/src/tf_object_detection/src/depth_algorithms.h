#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <random> // for Forgy method of initializing k-means for depth detection

// K_MEANS = k-means, Forgy method of initializing (k-means++ would likely be better, but Forgy is working ok for now)
// CONTOURS = find depth by finding contours and taking the lowest value of the biggest contour
// CONTOURS_NON_ADAPTIVE = contours, but using thresholding based on the median instead of adaptive thresholding
enum DepthCalculationAlgorithm { K_MEANS, CONTOURS, CONTOURS_NON_ADAPTIVE };

constexpr float depth_epsilon = 0.01; // If depth values are greater than this value (in meters) away from each other, they are treated as unique. See `kMeansDepthMat`.
constexpr float percent_from_bottom_contours = 0; // The contours algorithm finds the biggest contour in the image and returns the lowest value, excluding this percent of low values (outliers).
																									// For example, if this is 0, the contours algorithm will return the lowest value in the largest contour.
																									// If this is 0.01, the algorithm will throw out the lowest 1% of data and then return the lowest value.

constexpr DepthCalculationAlgorithm algorithm = CONTOURS; // The algorithm used to find depth data

float findMedianOfMat(cv::Mat mat);
float contoursDepthMat(const cv::Mat& depth_, const cv::Rect& bound_rect, bool debug = false, bool adaptive = true);
float kMeansDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, bool debug = false, size_t maximumK = 3, float tolerance = 1e-3);
float usefulDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, bool debug = false, DepthCalculationAlgorithm algorithm = CONTOURS, int k = 3, float tolerance = 1e-3);
