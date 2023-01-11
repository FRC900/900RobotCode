#include "depth_algorithms.h"

// Find the median of a continuous cv::Mat
float findMedianOfMat(cv::Mat mat) {
	float median = 0;
	if (mat.isContinuous()) {
		// copy matrix data to a vector
		std::vector<float> vec;
		mat = mat.reshape(0, 1);
		mat.copyTo(vec);
		// remove 255 (when this is called, values that are masked are 255)
		vec.erase(std::remove(vec.begin(), vec.end(), 255), vec.end());
		if ((vec.size() % 2) != 0) { // if odd
			auto it = vec.begin() + vec.size()/2;
			std::nth_element(vec.begin(), it, vec.end()); // sort vector so that the middle value is sorted, but not any of the other values
			median = vec[vec.size()/2];
		} else { // if even
			auto it = vec.begin() + vec.size()/2;
			std::nth_element(vec.begin(), it, vec.end()); // sort vector so that the middle value is sorted, but not any of the other values
			median += vec[vec.size()/2];
			it = vec.begin() + vec.size()/2 - 1;
			std::nth_element(vec.begin(), it, vec.end()); // sort vector so that the value before the middle value is sorted, but not any of the other values
			median += vec[vec.size()/2 - 1];
			median = median/(float)2.0;
		}
	}
	return median;
}

// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle, using contour finding
float contoursDepthMat(const cv::Mat& depth_, const cv::Rect& bound_rect, bool debug, bool adaptive) {
	if (bound_rect.size().area() == 0) { // if the ROI is zero, return -1 (no depth)
		return -1;
	}

	// Crop depth to region of interest
	cv::Mat depth = depth_(bound_rect);

	// set very large outliers and nan to 0 so they can be removed later. TODO see if the ZED actually reports negative depth
	float nan_ = std::numeric_limits<float>::quiet_NaN();
	cv::Mat inf = depth>=900;
	cv::Mat neg_inf = depth<=-900;
	cv::Mat nan = depth!=depth;
	depth.setTo(0, inf);
	depth.setTo(0, neg_inf);
	depth.setTo(0, nan);

	if (debug) {
		double min, max; // cv::minMaxLoc requires doubles which is why a double is used here. Also, this is only enabled when debug==true.
		cv::minMaxLoc(depth, &min, &max);
		ROS_INFO_STREAM("min: " << min << ", max: " << max);
		cv::Mat dest;
		depth.copyTo(dest);
		cv::normalize(dest, dest, 0, 255, cv::NORM_MINMAX);
		dest.convertTo(dest, CV_8UC1);
		cv::imshow("Depth", dest);
		cv::waitKey(1);
	}

	// convert depth to a 0-255 grayscale image (for contour finding)
	cv::Mat depthDifferentFormat;
	cv::Mat zeros = depth==0; // no depth is 0 with the ZED. May also need to check for inf and nan.
	depth.copyTo(depthDifferentFormat);
	cv::normalize(depthDifferentFormat, depthDifferentFormat, 0, 128, cv::NORM_MINMAX); // 0-128 because outliers will be 255 (and we want a clear background)
	depthDifferentFormat.setTo(255, zeros); // set zeros (no depth) to 255
	depthDifferentFormat.convertTo(depthDifferentFormat, CV_8UC1, 1);

	// Find the median of the image
	float median = findMedianOfMat(depthDifferentFormat);

	// Create a cv::Mat for thresholding output
	cv::Mat threshOutput;

	if (adaptive) {
		// use adaptive thresholding to convert image to black and white for finding
		// contours
		int blockSize = depthDifferentFormat.size().area() / 25;
		blockSize = blockSize > 1 ? (blockSize % 2 == 0 ? blockSize + 1 : blockSize) : 5; // block size must be at least 3 and odd
		cv::adaptiveThreshold(depthDifferentFormat, threshOutput, 1, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, blockSize, 0);
	} else {
		// thresholding using median
		cv::threshold(depthDifferentFormat, threshOutput, median, 1, cv::THRESH_BINARY_INV);
	}

	// find contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(threshOutput, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	// create a mask
	cv::Mat mask = cv::Mat::zeros(threshOutput.size(), CV_8UC1);
	int largestContourIndex;
	float largestArea = 0;
	for (size_t i = 0; i < contours.size(); i++) {
		std::vector<cv::Point> contour = contours[(int)i];
		cv::Rect rect = cv::boundingRect(contour);
		if (rect.area() > largestArea) {
			largestArea = rect.area();
			largestContourIndex = (int)i;
		}
	}
	// draw the largest contour onto the mask
	cv::Scalar color = cv::Scalar(255);
	cv::drawContours(mask, contours, largestContourIndex, color, -1, cv::LINE_8, hierarchy, 0);

	// make a new image for the original depth cut out by the mask, and fill it with std::numeric_limits<float>::max()
	// std::numeric_limits<float>::max() = ignore value, see line 168
	cv::Mat masked = cv::Mat::ones(depth.size(), depth.type()) * std::numeric_limits<float>::max();
	depth.copyTo(masked, mask);

	// if debug is enabled,
	if (debug) {
		// show the masked image
		cv::Mat destination = cv::Mat::zeros(depth.size(), depth.type());
		depth.copyTo(destination, mask);

		cv::normalize(destination, destination, 0, 255, cv::NORM_MINMAX);
		destination.convertTo(destination, CV_8UC1);
		cv::imshow("Masked", destination);
		cv::waitKey(0);
	}

	// copy matrix data to a vector
	std::vector<float> vec;
	masked = masked.reshape(0, 1);
	masked.copyTo(vec);
	// remove std::numeric_limits<float>::max() values from the mask
	vec.erase(std::remove(vec.begin(), vec.end(), std::numeric_limits<float>::max()), vec.end());
	// and 0 values (no depth)
	vec.erase(std::remove(vec.begin(), vec.end(), 0), vec.end());
	// sort vector
	std::sort(vec.begin(), vec.end());

	return (vec.size() != 0 ? vec[(size_t)(percent_from_bottom_contours * vec.size())] : usefulDepthMat(depth_, bound_rect, K_MEANS, debug)); // return lowest value after throwing out the bottom <percent_from_bottom_contours>%.
	// If there are no values, fall back to k-means.

	/* Test results (adaptive thresholding):
	[ INFO] [1635948213.029497061]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_8.jpg
	[ INFO] [1635948213.689598629]: Calculated depth is 113.001 ✅
	[ INFO] [1635948229.253951900]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_behind_power_cell.png
	[ INFO] [1635948229.870174338]: Calculated depth is 138.001 ✅
	[ INFO] [1635948236.848476178]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal.png
	[ INFO] [1635948237.445102845]: Calculated depth is 113.001 ✅
	[ INFO] [1635948248.428153064]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_loading_bay.png
	[ INFO] [1635948249.227361594]: Calculated depth is 141.001 ✅
	[ INFO] [1635948255.684437760]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_low_goal.png
	[ INFO] [1635948256.286312490]: Calculated depth is 92.001 ✅
	[ INFO] [1635948266.234706198]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_power_cell.png
	[ INFO] [1635948266.316279430]: Calculated depth is 0.001 ✅
	[ INFO] [1635948273.702984310]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/goal_behind_power_cell.png
	[ INFO] [1635948275.086796499]: Calculated depth is 0.001 **non-cropped version** ❌
	[ INFO] [1635948302.568242461]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/goal_with_power_cell_in_front.jpg
	[ INFO] [1635948302.598558216]: Calculated depth is 0.001 ❌
	[ INFO] [1635948312.950812216]: Received /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/test_weirdness.jpg
	[ INFO] [1635948312.980837687]: Calculated depth is 73.001 ✅
	*/
}

// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle, using k-means
float kMeansDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, bool debug, size_t maximumK, float tolerance)
{
	// setup randomizing (for initialization of k-means)
	std::random_device seeder;
	std::mt19937 engine(seeder());

	// Start of checking if k is too high

	// Convert cropped depth matrix to vector
	std::vector<float> vec;
	cv::Mat mat = depth(bound_rect);
	for (int i = 0; i < mat.rows; i++) {
    vec.insert(vec.end(), mat.ptr<float>(i), mat.ptr<float>(i)+mat.cols*mat.channels());
  }

	// Find unique (defined as >`depth_epsilon` meters away) values in the vector
	std::sort(vec.begin(), vec.end());
	auto last = std::unique(vec.begin(), vec.end(), [](float first, float second){ return fabs(second - first)<depth_epsilon; });
	vec.erase(last, vec.end());

	// Set k to maximumK if there are more than maximumK unique values, otherwise set it to the number of unique values
	size_t k = std::min(vec.size(), maximumK);

	// End of checking if k is too high

	// initialize arrays (and a vector) for k-means
	float centroids[k];
	float prevCentroids[k];
	std::vector<float> clusters[k];

	// Pick centroids
	std::sample(vec.begin(), vec.end(), centroids, k, engine);

	// Print centroids
	if (debug) {
		ROS_INFO_STREAM("k = " << k);
		ROS_INFO_STREAM("Centroids: ");
		for (size_t i = 0; i < k; i++) {
			ROS_INFO_STREAM(centroids[i]);
		}
	}

	while (true) { // once the algorithm converges this returns
		for (int j = bound_rect.tl().y+1; j < bound_rect.br().y; j++) // for each row
		{
			const float *ptr_depth = depth.ptr<float>(j);

			for (int i = bound_rect.tl().x+1; i < bound_rect.br().x; i++) // for each pixel in row
			{
				if (!(isnan(ptr_depth[i]) || isinf(ptr_depth[i]) || (ptr_depth[i] <= 0)))
				{
					// Calculate which centroid/mean the current pixel is closest to
					size_t closestCentroidIndex = -1;
					float smallestDiff = std::numeric_limits<float>::max();
					for (size_t c = 0; c < k; c++) {
						float diff = fabs(centroids[c] - ptr_depth[i]);
						if (diff < smallestDiff) {
							closestCentroidIndex = c;
							smallestDiff = diff;
						}
					}
					// Append the pixel's value to the cluster corresponding to that centroid
					clusters[closestCentroidIndex].push_back(ptr_depth[i]);
				}
			}
		}

		// Recalculate centroids using the average of the cluster closest to each centroid
		// (or set the centroid to std::numeric_limits<float>::max() if there are no values in the cluster)
		for (size_t i = 0; i < k; i++) {
			float sum = 0;
			for (float f : clusters[i]) {
				sum += f;
			}
			if (clusters[i].size() != 0) {
				centroids[i] = sum / (float)clusters[i].size();
			} else { // If the centroid's cluster has no values, set it to std::numeric_limits<float>::max() (basically remove it)
				centroids[i] = std::numeric_limits<float>::max();
			}
			clusters[i].clear(); // Clear clusters
		}

		// Calculate and print the difference between the current and previous centroids
		// this lets us see when the difference is very low (in which case the algorithm will be done)
		float diff = 0;
		for (size_t i = 0; i < k; i++) {
			diff += fabs(centroids[i] - prevCentroids[i]);
		}
		if (debug) {
			ROS_INFO_STREAM("diff: " << diff);
		}

		// If the difference is less than the tolerance, return the closest centroid
		if (diff <= tolerance) {
			return (*std::min_element(centroids, centroids+k)) == std::numeric_limits<float>::max() ? -1 : (*std::min_element(centroids, centroids+k));
		}

		// If the above statement didn't return, copy centroids to prevCentroids and
		// re-run the algorithm
		memcpy(prevCentroids, centroids, sizeof(prevCentroids));
	}
}

// Get the most useful depth value in the cv::Mat depth contained within
// the supplied bounding rectangle
float usefulDepthMat(const cv::Mat& depth, const cv::Rect& bound_rect, DepthCalculationAlgorithm algorithm, bool debug, int k, float tolerance)
{
	switch (algorithm) {
		case CONTOURS:
			return contoursDepthMat(depth, bound_rect, debug);
		case CONTOURS_NON_ADAPTIVE:
			return contoursDepthMat(depth, bound_rect, debug, false);
		case K_MEANS:
			return kMeansDepthMat(depth, bound_rect, debug, k, tolerance);
		case TWO_D_ONLY:
			return 1.;
	}
}
