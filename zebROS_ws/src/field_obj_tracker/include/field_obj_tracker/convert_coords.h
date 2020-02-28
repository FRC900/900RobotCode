#include <iostream>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>

class ConvertCoords
{
private:
	image_geometry::PinholeCameraModel model_;

public:
	ConvertCoords(const image_geometry::PinholeCameraModel &model);
	cv::Point3f screen_to_world( const cv::Rect &bounding_rect, const std::string &debug_name, const float depth ) const;
	cv::Point2f world_to_screen( const cv::Point3f &pos, const std::string &debug_name ) const;
};
