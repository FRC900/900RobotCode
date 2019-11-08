#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <sys/types.h>
#include <sys/stat.h>

#include "aviout.hpp"

void callback(const sensor_msgs::ImageConstPtr &frameMsg)
{
	static std::unique_ptr<AVIOut> aviOut = nullptr;
	cv_bridge::CvImagePtr cvFrame = cv_bridge::toCvCopy(frameMsg, sensor_msgs::image_encodings::BGR8);

	if (aviOut == nullptr)
	{
		int index = -1;
		int rc;
		char name[PATH_MAX];
		do
		{
			sprintf(name, "/home/ubuntu/Videos/cap%d_0_0.avi", ++index);
			struct stat statbuf;
			rc = stat(name, &statbuf);
		}
		while (rc == 0);

		sprintf(name, "/home/ubuntu/Videos/cap%d_0.avi", index);
		aviOut = std::make_unique<AVIOut>(name, cvFrame->image.size(), std::numeric_limits<int>::max(), 1);
	}

	aviOut->saveFrame(cvFrame->image, cv::Mat());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mjpeg_writer");
	ros::NodeHandle nh("~");

	// Sync up timestamps to find image and depth
	// data from the same frame
	auto sub = nh.subscribe("/c920_camera/image_raw", 5, callback);
	//auto sub = nh.subscribe("/zed_goal/left/image_rect_color", 5, callback);

	ros::spin();

	return 0;
}
