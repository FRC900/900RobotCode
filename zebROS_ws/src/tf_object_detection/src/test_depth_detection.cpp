#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "depth_algorithms.h"

#include <std_msgs/String.h> // for testing depth detection on the `test_depth` topic

DepthCalculationAlgorithm algorithm = CONTOURS;

// A callback function for testing usefulDepthMat. This is for a subscriber that
// subscribes to the `test_depth` topic.
void testUsefulDepthMatCallback(const std_msgs::String::ConstPtr& msg) {
	// the message will be a filepath to an image file for testing
	ROS_INFO_STREAM("Received " << msg->data);
	cv::Mat depth;
	if (msg->data.back() == 'r') { // likely a .ex`r` file, treating it as one
		// make sure to set outliers to zero in the file
		depth = cv::imread(msg->data, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH); // read image
		depth.convertTo(depth, CV_32FC1);
		std::vector<cv::Mat> channels(3);
		cv::split(depth, channels);
		depth = channels[0];
	} else {
		depth = cv::imread(msg->data, cv::IMREAD_GRAYSCALE); // read image as grayscale
		depth.convertTo(depth, CV_32FC1);
		// Add random noise
		cv::Mat noise(depth.size(), depth.type());
		cv::randn(noise, 0, 5);
		cv::Mat double_noise(depth.size(), depth.type());
		cv::randn(double_noise, 1, 0.1);
		depth += 1; // with the ZED, a zero means no depth
	}

 	// Calculate the most useful depth using contour finding (default algorithm) and print it
	cv::Rect depth_rect = cv::Rect(0, 0, depth.size().width, depth.size().height);
	ROS_INFO_STREAM("Calculated depth is " << usefulDepthMat(depth, depth_rect, algorithm, true));
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "test_depth_detection");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // Add a subscriber to subscribe to testing messages
  // Example of message to send: rostopic pub /test_depth std_msgs/String /home/ubuntu/2020Offseason/zebROS_ws/src/tf_object_detection/test_bitmaps/cropped_goal_behind_power_cell.png
  ros::Subscriber sub = nh.subscribe("test_depth", 10, testUsefulDepthMatCallback);

  ros::spin();

  return 0;
}
