#ifndef UTILITIES
#define UTILITIES
#include <vector>
#include <utility>

#include "opencv2/core/core.hpp"
#include "field_obj_tracker/objtype.hpp"
#include "image_geometry/pinhole_camera_model.h"

namespace zv_utils {

std::pair<float, float> minOfDepthMat(const cv::Mat& img, const cv::Mat& mask, const cv::Rect& bound_rect, int range);
float avgOfDepthMat(const cv::Mat& img, const cv::Mat& mask, const cv::Rect& bound_rect);
void shrinkRect(cv::Rect &rect_in, float shrink_factor);

//void printIsometry(const Eigen::Transform<double, 3, Eigen::Isometry> m);
double slope_list(const std::vector<double>& x, const std::vector<double>& y);
std::pair<double,double> slopeOfMasked(ObjectType ot, const cv::Mat &depth, const cv::Mat &mask, const image_geometry::PinholeCameraModel &model);
double normalCFD(const std::pair<double, double> &meanAndStdev, double value);

}

#endif
