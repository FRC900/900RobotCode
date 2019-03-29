#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Header.h>

#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>

#include "teraranger_array/RangeArray.h"

#include "goal_detection/GoalDetection.h"

#include <sstream>

#include "GoalDetector.hpp"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;
GoalDetector *gd = NULL;
bool batch = true;
bool down_sample = false;
double hFov = 105; //105.;
double camera_angle = -25.0;

void callback(const ImageConstPtr &frameMsg, const ImageConstPtr &depthMsg)
{
	cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
	cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

	// Avoid copies by using pointers to RGB and depth info
	// These pointers are either to the original data or to
	// the downsampled data, depending on the down_sample flag
	const Mat *framePtr = &cvFrame->image;
	const Mat *depthPtr = &cvDepth->image;

	// To hold downsampled images, if necessary
	Mat frame;
	Mat depth;

	// Downsample for speed purposes
	if (down_sample)
	{
		pyrDown(*framePtr, frame);

		pyrDown(*depthPtr, depth);

		// And update pointers to use the downsampled
		// versions of the RGB and depth data
		framePtr = &frame;
		depthPtr = &depth;
	}

	// Initialize goal detector object the first time
	// through here. Use the size of the frame
	// grabbed from the ZED messages
	if (gd == NULL)
	{
		const Point2f fov(hFov * (M_PI / 180.),
						  hFov * (M_PI / 180.) * ((double)framePtr->rows / framePtr->cols));
		gd = new GoalDetector(fov, framePtr->size(), !batch);
	}
	//Send current color and depth image to the actual GoalDetector
	gd->findBoilers(*framePtr, *depthPtr);

	vector< GoalFound > gfd = gd->return_found();
	goal_detection::GoalDetection gd_msg;

	for(size_t i = 0; i < gfd.size(); i++)
	{
		geometry_msgs::Point32 dummy;
		gd_msg.header.seq = frameMsg->header.seq;
		gd_msg.header.stamp = frameMsg->header.stamp;
		// Remove _optical_frame from the camera frame ID if present
		std::string frame_id = frameMsg->header.frame_id;
		const size_t idx = frame_id.rfind("_optical_frame");
		if (idx != std::string::npos)
			frame_id.erase(idx);
		gd_msg.header.frame_id = frame_id;
		dummy.x = gfd[i].pos.y;
		dummy.y = gfd[i].pos.x;
		dummy.z = gfd[i].pos.z;
		gd_msg.location.push_back(dummy);
	}

	gd_msg.valid = gd->Valid();

	pub.publish(gd_msg);

	if (!batch)
	{
		Mat thisFrame(framePtr->clone());
		gd->drawOnFrame(thisFrame, gd->getContours(cvFrame->image));
		imshow("Image", thisFrame);
		waitKey(5);
	}

	if (gd_msg.valid == false)
	{
		return;
	}
/*
	//Transform between goal frame and odometry/map.
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = cvFrame->header.frame_id;
	transformStamped.child_frame_id = "goal";

	transformStamped.transform.translation.x = gd_msg.location.x;
	transformStamped.transform.translation.y = gd_msg.location.y;
	transformStamped.transform.translation.z = gd_msg.location.z;

	tf2::Quaternion q;
	q.setRPY(0, 0, 0);

	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);

	//Transform between a fixed frame and the goal.
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	geometry_msgs::TransformStamped transformStampedOdomCamera;
	try
	{
		transformStampedOdomCamera = tfBuffer.lookupTransform("odom", cvFrame->header.frame_id,
									 ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		ros::Duration(1.0).sleep();
		return;
	}

	geometry_msgs::TransformStamped transformStampedOdomGoal;

	tf2::doTransform(transformStamped, transformStampedOdomGoal, transformStampedOdomCamera);

	br.sendTransform(transformStampedOdomGoal);
*/
}

double distance_from_terabee = -1;
void multiflexCB(const teraranger_array::RangeArray& msg)
{
    double min_dist = std::numeric_limits<double>::max();
	distance_from_terabee = -1;
	for(int i = 0; i < 2; i++)
	{
		const double range = static_cast<double>(msg.ranges[i].range);
		if(!std::isnan(range))
			min_dist = std::min(min_dist, range);
	}
	if (min_dist != std::numeric_limits<double>::max())
		distance_from_terabee = min_dist;
}

void callback_no_depth(const ImageConstPtr &frameMsg)
{
	cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
	cv::Mat depthMat(cvFrame->image.size(), CV_32FC1, Scalar(distance_from_terabee));
	callback(frameMsg, cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthMat).toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_detect");

	ros::NodeHandle nh("~");
	down_sample = false;
	int sub_rate = 2;
	int pub_rate = 1;
	nh.getParam("down_sample", down_sample);
	nh.getParam("sub_rate", sub_rate);
	nh.getParam("pub_rate", pub_rate);
	nh.getParam("batch", batch);

	bool no_depth = false;
	nh.getParam("no_depth", no_depth);

	nh.getParam("hFov", hFov);
	nh.getParam("camera_angle", camera_angle);

	std::shared_ptr<message_filters::Subscriber<Image>> frame_sub;
	std::shared_ptr<message_filters::Subscriber<Image>> depth_sub;
	typedef sync_policies::ApproximateTime<Image, Image>  SyncPolicy;
	std::shared_ptr<Synchronizer<SyncPolicy>> sync;

	std::shared_ptr<ros::Subscriber> rgb_sub;
	if (!no_depth)
	{
		ROS_INFO("starting goal detection using ZED");
		frame_sub = std::make_shared<message_filters::Subscriber<Image>>(nh, "/zed_goal/left/image_rect_color", sub_rate);
		depth_sub = std::make_shared<message_filters::Subscriber<Image>>(nh, "/zed_goal/depth/depth_registered", sub_rate);
		// ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(xxx)
		sync = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(10), *frame_sub, *depth_sub);
		sync->registerCallback(boost::bind(&callback, _1, _2));
	}
	else
	{
		ROS_INFO("starting goal detection using webcam");
		rgb_sub = std::make_shared<ros::Subscriber>(nh.subscribe("/c920_camera/image_raw", sub_rate, callback_no_depth));
	}
	ros::Subscriber terabee_sub = nh.subscribe("/multiflex_1/ranges_raw", 1, &multiflexCB);

	// Set up publisher
	pub = nh.advertise<goal_detection::GoalDetection>("goal_detect_msg", pub_rate);

	ros::spin();

	delete gd;

	return 0;
}
