#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <XmlRpcValue.h>

double field_width, field_height, inches_per_pixel; //field dimensions in inches
int height;
const int width = 1200;
const double border = width;

cv::Point map_to_image(cv::Point map_coord)
{
	const double image_x = map_coord.x / inches_per_pixel + border;
	const double image_y = (height - map_coord.y / inches_per_pixel) + border;
	return cv::Point(image_x, image_y);
}

void drawRotatedRectangle(cv::Mat& image, const cv::Point &centerPoint, const cv::Size &rectangleSize, const double rotationDegrees, const cv::Scalar &color)
{
	// Create the rotated rectangle
	cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);

	// We take the edges that OpenCV calculated for us
	cv::Point2f vertices2f[4];
	rotatedRectangle.points(vertices2f);

	// Convert them so we can use them in a fillConvexPoly
	cv::Point vertices[4];
	for(int i = 0; i < 4; ++i){
		vertices[i] = vertices2f[i];
	}

	// Now we can fill the rotated rectangle with our specified color
	cv::fillConvexPoly(image, vertices, 4, color);
}

void drawPoly(cv::Mat& image, const std::vector<cv::Point>& points, const cv::Scalar &color)
{
	// Convert map coordinates to image coordinates
	const size_t num_points = points.size();
	cv::Point image_vertices[num_points];
	for(size_t i = 0; i < num_points; ++i){
		image_vertices[i] = map_to_image(points[i]);
	}

	// Now we can fill the triangle with our specific color
	cv::fillConvexPoly(image, image_vertices, num_points, color);
}

void drawCircle(cv::Mat& image, const cv::Point &center, const double radius, const cv::Scalar &color)
{
	// Convert field coordinates to image coordinates
	cv::Point image_center = map_to_image(center);

	// Now we can draw a filled circle
	cv::circle(image, image_center, radius, color, cv::FILLED, 8);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "create_field_map");
  ros::NodeHandle nh_;

	XmlRpc::XmlRpcValue xml_obstacles_list, xml_obstacle;
	std::string imagename;

  if (!nh_.getParam("field_dims/width", field_width)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }

  if (!nh_.getParam("field_dims/height", field_height)) {
    ROS_ERROR("field dimension not specified");
    return -1;
  }

	if (!nh_.getParam("obstacles", xml_obstacles_list)) {
    ROS_ERROR("failed to load obstacles");
    return -1;
  }

	if (!nh_.getParam("imagename", imagename)) {
    ROS_ERROR("failed to load image name");
    return -1;
  }

	height = width / (field_width / field_height);
	inches_per_pixel = field_width / width;

	// Create mat large enough to hold field plus a border
	cv::Mat image(cv::Size(width + 2 * border, height + 2 * border), CV_8UC1, cv::Scalar(255));
	ROS_INFO_STREAM(image.size() << " " << image.depth() << " " << image.channels());

	// Draw field border
	cv::rectangle(image, cv::Point(0,0), cv::Point(image.cols-1, image.rows-1), cv::Scalar(0,0,0), 4);
#if 0
	for (int row = 0; row < image.rows; row++)
	{
		for (int col = 0; col < image.cols; col++)
		{
			const double xpos_in_inches = static_cast<double>(col - border) * inches_per_pixel;
			const double ypos_in_inches = static_cast<double>(row - border) * inches_per_pixel;
			if ((xpos_in_inches < 0) || (ypos_in_inches < 0))
			{
				image.at<uchar>(row, col) = 0;
			}
			if (xpos_in_inches > field_width)
			{
				image.at<uchar>(row, col) = 0;
			}
			if (ypos_in_inches > field_height)
			{
				image.at<uchar>(row, col) = 0;
			}
		}
	}
#endif

	// Draw field obstacles
	for (size_t i = 0; i < (unsigned) xml_obstacles_list.size(); i++) {
		xml_obstacle = xml_obstacles_list[i];
		std::string type = xml_obstacle["type"];

		if(type == "circle")
		{
			cv::Point center = cv::Point(xml_obstacle["center"][0], xml_obstacle["center"][1]);
			drawCircle(image, center, (double)xml_obstacle["radius"], cv::Scalar(0,0,0));
		}
		else if(type == "polygon")
		{
			std::vector<cv::Point> points;
			std::string p = "p";
			int num_points = xml_obstacle["num_points"];
			for(int i = 1; i < num_points + 1; ++i)
			{
				points.push_back(cv::Point(xml_obstacle[p + std::to_string(i)][0], xml_obstacle[p + std::to_string(i)][1]));
			}
			drawPoly(image, points, cv::Scalar(0,0,0));
		}
  }

	// Calculations for various inputs to stage and map_server
	const double meter_per_pixel = (field_width * .0254) / width;
	ROS_INFO_STREAM("meters per pixel: " << meter_per_pixel);
	ROS_INFO_STREAM("width x height: " << meter_per_pixel * image.cols << " " << meter_per_pixel * image.rows);
	ROS_INFO_STREAM("pose " << (meter_per_pixel * image.cols) / 2. - border * meter_per_pixel << " " << (meter_per_pixel * image.rows) / 2. - border * meter_per_pixel);

	// Write mapserver yaml file
	std::string yaml_file_name(imagename);
	auto ext_pos = yaml_file_name.find_last_of(".png");
	if (ext_pos == std::string::npos)
		ext_pos = yaml_file_name.find_last_of(".PNG");
	if (ext_pos == std::string::npos)
		ext_pos = yaml_file_name.find_last_of(".jpg");
	if (ext_pos == std::string::npos)
		ext_pos = yaml_file_name.find_last_of(".JPG");
	if (ext_pos == std::string::npos)
		ROS_ERROR_STREAM("Can't figure out extension of " << imagename << ", not writing yaml file");
	else
	{
		yaml_file_name.erase(ext_pos-3, 4);
		yaml_file_name += ".yaml";
		std::ofstream of(yaml_file_name);
		of << "image: " << imagename << std::endl;
		of << "resolution: " << meter_per_pixel << std::endl;
		of << "origin: [" << meter_per_pixel * -border << ", " << meter_per_pixel * -border << ", 0]" << std::endl;
		of << "occupied_thresh: 0.65" << std::endl;
		of << "free_thresh: 0.196" << std::endl;
		of << "negate: 0" << std::endl;
	}

	// Write and show field image
	cv::imwrite(imagename, image);
	cv::imshow("image", image);
	cv::waitKey(0);

	return 0;
}
