#include <iostream>
#include <opencv2/opencv.hpp>

// https://stackoverflow.com/questions/43342199/draw-rotated-rectangle-in-opencv-c
// Include center point of your rectangle, size of your rectangle and the degrees of rotation
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
	cv::fillConvexPoly(image,
			vertices,
			4,
			color);
}

int main(void)
{

	// In inches
	constexpr double field_width  = 30. * 12.;
	constexpr double field_height = 15. * 12.;

	// In pixels
	constexpr int width  = 1200;
	constexpr int height = width / (field_width / field_height);
	constexpr double border = width * .05;

	// Create mat large enough to hold field plus a border
	cv::Mat image(cv::Size(width + 2 * border, height + 2 * border), CV_8UC1, cv::Scalar(255));
	std::cout << image.size() << " " << image.depth() << " " << image.channels() << std::endl;

	const double inches_per_pixel = field_width / width;
	std::cout << "inches per pixel = " << inches_per_pixel << std::endl;


	// Calculations for various inputs to stage and map_server
	double meter_per_pixel = (field_width * .0254) / width;
	std::cout << "meters per pixel = " << meter_per_pixel << std::endl;
	std::cout << "width x height " << meter_per_pixel * image.cols << " " << meter_per_pixel * image.rows << std::endl;
	std::cout << "pose " << (meter_per_pixel * image.cols) / 2. - border * meter_per_pixel << " " << (meter_per_pixel * image.rows) / 2. - border * meter_per_pixel << std::endl;
	cv::imwrite("2021_empty_field.png", image);
	cv::imshow("Input", image);
	cv::waitKey(0);
	return 0;
}

