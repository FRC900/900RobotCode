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
	constexpr double field_width  = 52. * 12. + 5. + 1./4.;
	constexpr double field_height = 26. * 12. + 11. + 1./4.;

	// In pixels
	constexpr int width  = 1200;
	constexpr int height = width / (field_width / field_height);
	constexpr double border = width * .05;

	// Inches
	constexpr double ds_wall_slope_height = 69.5;
	constexpr double ds_wall_slope_width = (field_width - 557.84) / 2.;

	constexpr double trench_to_ds_wall = 344.453;
	constexpr double trench_to_side_wall = 53.5;
	constexpr double trench_width = 2;
	constexpr double trench_length = 30;


	// Create mat large enough to hold field plus a border
	cv::Mat image(cv::Size(width + 2 * border, height + 2 * border), CV_8UC1, cv::Scalar(255));
	std::cout << image.size() << " " << image.depth() << " " << image.channels() << std::endl;

	const double inches_per_pixel = field_width / width;
	std::cout << "inches per pixel = " << inches_per_pixel << std::endl;

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
			// Angled DS wall
			if (ypos_in_inches < ds_wall_slope_height)
			{
				const double near_x_intercept = ds_wall_slope_width - ypos_in_inches * (ds_wall_slope_width / ds_wall_slope_height);
				if ((xpos_in_inches < near_x_intercept) ||
				    (xpos_in_inches > (field_width - near_x_intercept)))
				{
					image.at<uchar>(row, col) = 0;
				}
			}
			// Angled DS wall
			if ((field_height - ypos_in_inches) < ds_wall_slope_height)
			{
				const double near_x_intercept = ds_wall_slope_width - (field_height - ypos_in_inches) * (ds_wall_slope_width / ds_wall_slope_height);
				if ((xpos_in_inches < near_x_intercept) ||
				    (xpos_in_inches > (field_width - near_x_intercept)))
				{
					image.at<uchar>(row, col) = 0;
				}
			}

			// Trench
			if ((xpos_in_inches > trench_to_ds_wall) && (xpos_in_inches < (trench_to_ds_wall + trench_length)) &&
			    (ypos_in_inches > trench_to_side_wall) && (ypos_in_inches < (trench_to_side_wall + trench_width)))
			{
				image.at<uchar>(row, col) = 0;
			}
			if ((xpos_in_inches < (field_width - trench_to_ds_wall)) &&
				(xpos_in_inches > (field_width - (trench_to_ds_wall + trench_length))) &&
				(ypos_in_inches < (field_height - trench_to_side_wall)) &&
				(ypos_in_inches > (field_height - (trench_to_side_wall + trench_width))))
			{
				image.at<uchar>(row, col) = 0;
			}
			//std::cout << xpos_in_inches << ", " << ypos_in_inches << std::endl;
		}
	}

	// Draw pillars as rotated rects
	const double pillar_dx = 4.629 / inches_per_pixel;
	const double pillar_dy = 11.150 / inches_per_pixel;
	const double pillar_hypot = hypot(pillar_dy, pillar_dy);

	const double pillar_corner_to_center_x = 8.471;
	const double pillar_corner_to_center_y = 3.28l;
	const double pillar_1_x = (205.897 + pillar_corner_to_center_x) / inches_per_pixel + border;
	const double pillar_1_y = (121.155 + pillar_corner_to_center_y) / inches_per_pixel + border;
	drawRotatedRectangle(image,
			cv::Point(pillar_1_x, pillar_1_y),
			cv::Size(pillar_hypot, pillar_hypot),
			atan2(pillar_dy, pillar_dx) * 180. / M_PI,
			cv::Scalar(0,0,0));

	const double pillar_2_x = (350.715 + pillar_corner_to_center_x) / inches_per_pixel + border;
	const double pillar_2_y =  (61.169 + pillar_corner_to_center_y) / inches_per_pixel + border;
	drawRotatedRectangle(image,
			cv::Point(pillar_2_x, pillar_2_y),
			cv::Size(pillar_hypot, pillar_hypot),
			atan2(pillar_dy, pillar_dx) * 180. / M_PI,
			cv::Scalar(0,0,0));

	const double pillar_3_x = (406.905 + pillar_corner_to_center_x) / inches_per_pixel + border;
	const double pillar_3_y = (195.651 + pillar_corner_to_center_y) / inches_per_pixel + border;
	drawRotatedRectangle(image,
			cv::Point(pillar_3_x, pillar_3_y),
			cv::Size(pillar_hypot, pillar_hypot),
			atan2(pillar_dy, pillar_dx) * 180. / M_PI,
			cv::Scalar(0,0,0));

	const double pillar_4_x = (262.087 + pillar_corner_to_center_x) / inches_per_pixel + border;
	const double pillar_4_y = (255.637 + pillar_corner_to_center_y) / inches_per_pixel + border;
	drawRotatedRectangle(image,
			cv::Point(pillar_4_x, pillar_4_y),
			cv::Size(pillar_hypot, pillar_hypot),
			atan2(pillar_dy, pillar_dx) * 180. / M_PI,
			cv::Scalar(0,0,0));

	// Calculations for various inputs to stage and map_server
	double meter_per_pixel = (field_width * .0254) / width;
	std::cout << "meters per pixel = " << meter_per_pixel << std::endl;
	std::cout << "width x height " << meter_per_pixel * image.cols << " " << meter_per_pixel * image.rows << std::endl;
	std::cout << "pose " << (meter_per_pixel * image.cols) / 2. - border * meter_per_pixel << " " << (meter_per_pixel * image.rows) / 2. - border * meter_per_pixel << std::endl;
	cv::imwrite("2020Field.png", image);
	cv::imshow("Input", image);
	cv::waitKey(0);
	return 0;
}

