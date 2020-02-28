#include "goal_detection/Utilities.hpp"
#include <numeric>
using namespace std;

namespace zv_utils {

	std::pair<float, float> minOfDepthMat(const cv::Mat& img, const cv::Mat& mask, const cv::Rect& bound_rect, int range) {

		if (img.empty())
			return make_pair(-1,-1);
		if ((img.rows != mask.rows) || (img.cols != mask.cols))
			return make_pair(-2,-2);

		//cout << bound_rect << endl;
		float min = numeric_limits<float>::max();
		float max = numeric_limits<float>::lowest();
		int min_loc_x;
		int min_loc_y;
		int max_loc_x;
		int max_loc_y;
		bool found = false;
		for (int j = bound_rect.tl().y + 1; j < bound_rect.br().y; j++) //for each row
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);

			for (int i = bound_rect.tl().x + 1; i < bound_rect.br().x; i++) //for each pixel in row
			{
				//cout << i << " " << j << " " << ptr_img[i] << " " << (int)ptr_mask[i] << endl;
				if (ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
				{
					found = true;
					if (ptr_img[i] > max)
					{
						max = ptr_img[i];
						max_loc_x = i;
						max_loc_y = j;
					}

					if (ptr_img[i] < min)
					{
						min = ptr_img[i];
						min_loc_x = i;
						min_loc_y = j;
					}
				}
			}
		}
		if(!found)
		{
			return make_pair(-3, -3);
		}
		float sum_min   = 0;
		int num_pix_min = 0;
		for (int j = min_loc_y - range; j < (min_loc_y + range); j++)
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);
		    for (int i = min_loc_x - range; i < (min_loc_x + range); i++)
		    {
		        if ((0 < i) && (i < img.cols) && (0 < j) && (j < img.rows) && ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
		        {
		            sum_min += ptr_img[i];
		            num_pix_min++;
		        }
		    }
		}
		float sum_max = 0;
		int num_pix_max = 0;
		for (int j = max_loc_y - range; j < (max_loc_y + range); j++)
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);
		    for (int i = max_loc_x - range; i < (max_loc_x + range); i++)
		    {
		        if ((0 < i) && (i < img.cols) && (0 < j) && (j < img.rows) && ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
		        {
		            sum_max += ptr_img[i];
		            num_pix_max++;
		        }
		    }
		}
		// Need to debug this more but for now fix it
		// by returning a negative number (i.e. failure)
		// if it happens
		float min_dist = sum_min / num_pix_min;
		if (isinf(min_dist))
			min_dist = -4;
		float max_dist = sum_max / num_pix_max;
		if (isinf(max_dist))
			max_dist = -4;
		return std::make_pair(min_dist, max_dist);
	}

	std::string GetMatDepth(const cv::Mat& mat)
	{
		const int depth = mat.depth();

		switch (depth)
		{
			case CV_8U:  return "CV_8U";
			case CV_8S:  return "CV_8S";
			case CV_16U: return "CV_16U";
			case CV_16S: return "CV_16S";
			case CV_32S: return "CV_32S";
			case CV_32F: return "CV_32F";
			case CV_64F: return "CV_64F";
			default:
						 return "Invalid depth type of matrix!";
		}
	}

	std::string GetMatType(const cv::Mat& mat)
	{
		const int mtype = mat.type();

		switch (mtype)
		{
			case CV_8UC1:  return "CV_8UC1";
			case CV_8UC2:  return "CV_8UC2";
			case CV_8UC3:  return "CV_8UC3";
			case CV_8UC4:  return "CV_8UC4";

			case CV_8SC1:  return "CV_8SC1";
			case CV_8SC2:  return "CV_8SC2";
			case CV_8SC3:  return "CV_8SC3";
			case CV_8SC4:  return "CV_8SC4";

			case CV_16UC1: return "CV_16UC1";
			case CV_16UC2: return "CV_16UC2";
			case CV_16UC3: return "CV_16UC3";
			case CV_16UC4: return "CV_16UC4";

			case CV_16SC1: return "CV_16SC1";
			case CV_16SC2: return "CV_16SC2";
			case CV_16SC3: return "CV_16SC3";
			case CV_16SC4: return "CV_16SC4";

			case CV_32SC1: return "CV_32SC1";
			case CV_32SC2: return "CV_32SC2";
			case CV_32SC3: return "CV_32SC3";
			case CV_32SC4: return "CV_32SC4";

			case CV_32FC1: return "CV_32FC1";
			case CV_32FC2: return "CV_32FC2";
			case CV_32FC3: return "CV_32FC3";
			case CV_32FC4: return "CV_32FC4";

			case CV_64FC1: return "CV_64FC1";
			case CV_64FC2: return "CV_64FC2";
			case CV_64FC3: return "CV_64FC3";
			case CV_64FC4: return "CV_64FC4";

			default:
						   return "Invalid type of matrix!";
		}
	}
	float avgOfDepthMat(const cv::Mat& img, const cv::Mat& mask, const cv::Rect& bound_rect)
	{
		double sum = 0.0;
		unsigned count = 0;
		//std::cout << "Mat Depth : " << GetMatDepth(img);
		//std::cout << " type : " << GetMatType(img) << std::endl;
		for (int j = bound_rect.tl().y+1; j < bound_rect.br().y; j++) //for each row
		{
			const float *ptr_img  = img.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);

			for (int i = bound_rect.tl().x+1; i < bound_rect.br().x; i++) //for each pixel in row
			{
				if (ptr_mask[i] && !(isnan(ptr_img[i]) || isinf(ptr_img[i]) || (ptr_img[i] <= 0)))
				{
					sum += ptr_img[i];
					//depths.push_back(ptr_img[i]);
					count += 1;
				}
			}
		}
		if (count == 0)
			return -1;
		return sum / count;
	}

	void shrinkRect(cv::Rect &rect_in, float shrink_factor) {

		rect_in.tl() = rect_in.tl() + cv::Point(shrink_factor/2.0 * rect_in.width, shrink_factor/2.0 * rect_in.height);
		rect_in.br() = rect_in.br() - cv::Point(shrink_factor/2.0 * rect_in.width, shrink_factor/2.0 * rect_in.height);

	}
#if 0
	void printIsometry(const Eigen::Transform<double, 3, Eigen::Isometry> m) {

		Eigen::Vector3d xyz = m.translation();
		Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
		cout << "Camera Translation: " << xyz << endl;
		cout << "Camera Rotation: " << rpy << endl;
	}
#endif

	//gets the slope that the masked area is facing away from the camera
	//useful when used with a contour to find the angle that an object is faciing

	double slope_list(const std::vector<double>& x, const std::vector<double>& y) {
	    const auto n    = x.size();
	    const auto s_x  = std::accumulate(x.begin(), x.end(), 0.0);
	    const auto s_y  = std::accumulate(y.begin(), y.end(), 0.0);
	    const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
	    const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
	    const auto a    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
	    return a;
	}

	std::pair<double,double> slopeOfMasked(const ObjectType &ot, const cv::Mat &depth, const cv::Mat &mask, const image_geometry::PinholeCameraModel &model) {

		CV_Assert(mask.depth() == CV_8U);
		vector<double> slope_x_values;
		vector<double> slope_y_values;
		vector<double> slope_z_values;

		for (int j = 0; j < depth.rows; j++) {

			const float *ptr_depth = depth.ptr<float>(j);
			const uchar *ptr_mask = mask.ptr<uchar>(j);

			for (int i = 0; i < depth.cols; i++) {
				if (ptr_mask[i] && (ptr_depth[i] > 0)) {
					cv::Point3f pos = ot.screenToWorldCoords(cv::Rect(i,j,0,0), ptr_depth[i], model);
					slope_x_values.push_back(pos.x);
					slope_y_values.push_back(pos.y);
					slope_z_values.push_back(pos.z);
				}
			}
		}

		return std::make_pair<double,double>(slope_list(slope_x_values, slope_y_values), slope_list(slope_z_values, slope_y_values));

	}

	double normalCFD(const pair<double,double> &meanAndStddev, double value)
	{
		double z_score = (value - meanAndStddev.first) / meanAndStddev.second;
		return 0.5 * erfc(-z_score * M_SQRT1_2);
	}

}
