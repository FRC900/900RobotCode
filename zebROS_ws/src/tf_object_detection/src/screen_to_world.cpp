#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "field_obj/Detection.h"
#include "field_obj/TFDetection.h"
#include "field_obj_tracker/convert_coords.h"
#include "field_obj_tracker/objtype.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "depth_algorithms.h"

ros::Publisher pub;

sensor_msgs::CameraInfo caminfo;
bool caminfovalid {false};

DepthCalculationAlgorithm algorithm = CONTOURS;

// Capture camera info published about the camera - needed for screen to world to work
void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info)
{
	caminfo = *info;
	caminfovalid = true;
}

// For each object in objDetectionMsg, look up the depth reported in depthMsg at the center of the
// object's bounding rectangle. Use that to convert from 2D screen coordinates to 3D world coordinates
void callback(const field_obj::TFDetectionConstPtr &objDetectionMsg, const sensor_msgs::ImageConstPtr &depthMsg)
{
	if (!caminfovalid)
	{
		return;
	}

	cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

	// Initialize published message with header info from objDetectionMsg
	field_obj::Detection out_msg;
	out_msg.header = objDetectionMsg->header;
	// Remove _optical_frame from the camera frame ID if present
	std::string frame_id = objDetectionMsg->header.frame_id;
	const size_t idx = frame_id.rfind("_optical_frame");
	if (idx != std::string::npos)
	{
		frame_id.erase(idx);
		frame_id += "_frame";
	}
	out_msg.header.frame_id = frame_id;
	// Create objects needed to convert from 2d screen to 3d world coords
	const ConvertCoords cc(caminfo);
	// Iterate over each object. Convert from camera to world coords.
	for (const auto &camObject : objDetectionMsg->objects)
	{
		if (fabs(camObject.br.x - camObject.tl.x) < 1) {
			ROS_INFO_STREAM("less than 1 x");
			return;
		}

		if (fabs(camObject.br.y - camObject.tl.y) < 1) {
			ROS_INFO_STREAM("less than 1 x");
			return;
		}
		// Create an output object, copy over info from the object detection info
		field_obj::Object worldObject;
		worldObject.id = camObject.label;
		worldObject.confidence = camObject.confidence;
		// Generate a bounding rect (in camera coords) from the camera object
		const cv::Point rectTL(camObject.tl.x, camObject.tl.y);
		const cv::Point rectBR(camObject.br.x, camObject.br.y);
		const cv::Rect  rect(rectTL, rectBR);

		// Find the center of the detected object
		const cv::Point2f objRectCenter = 0.5 * (rectTL + rectBR);

		// Get the distance to the object by finding contours within the depth data inside the
		// object's bounding rectangle.
		const float objDistance = usefulDepthMat(cvDepth->image, rect, algorithm, false);
		if (objDistance < 0 || isnan(objDistance))
		{
			ROS_ERROR_STREAM_THROTTLE(2, "Depth of object at " << objRectCenter << " with bounding rect " << rect << " invalid : " << objDistance);
			return;
		}
		const cv::Point3f world_coord_scaled = cc.screen_to_world(rect, worldObject.id, objDistance);

		// Convert from camera_optical_frame to camera_frame - could do this
		// using transforms in the future?
		worldObject.location.x = world_coord_scaled.z;
		worldObject.location.y = -world_coord_scaled.x;
		worldObject.location.z = world_coord_scaled.y;

		worldObject.angle = atan2(worldObject.location.y, worldObject.location.x) * 180. / M_PI;

		// Add the 3d object info to the list of objects in the output message
		out_msg.objects.push_back(worldObject);
	}

	pub.publish(out_msg);

	if (out_msg.objects.size() > 0)
	{
		//Transform between goal frame and odometry/map.
		static tf2_ros::TransformBroadcaster br;
		for(size_t i = 0; i < out_msg.objects.size(); i++)
		{
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = out_msg.header.stamp;
			transformStamped.header.frame_id = out_msg.header.frame_id;
			std::stringstream child_frame;
			// tf gets messed up if a transform name starts with a number
  			if (isdigit(out_msg.objects[i].id[0])) {
				child_frame << "obj_";
			}
			child_frame << out_msg.objects[i].id;
			child_frame << "_";
			child_frame << i;
			transformStamped.child_frame_id = child_frame.str();

			transformStamped.transform.translation.x = out_msg.objects[i].location.x;
			transformStamped.transform.translation.y = out_msg.objects[i].location.y;
			transformStamped.transform.translation.z = out_msg.objects[i].location.z;

			// Can't detect rotation yet, so publish 0 instead
			tf2::Quaternion q;
			q.setRPY(0, 0, 0);

			transformStamped.transform.rotation.x = q.x();
			transformStamped.transform.rotation.y = q.y();
			transformStamped.transform.rotation.z = q.z();
			transformStamped.transform.rotation.w = q.w();

			br.sendTransform(transformStamped);
		}
	}
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "tf_object_screen_to_world");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	image_transport::ImageTransport it(nh);

	std::string algorithm_str;
	double timeout;

	nh_param.param<std::string>("depth_algorithm", algorithm_str, "CONTOURS");
	nh_param.param<double>("timeout", timeout, 0.05);

	ROS_INFO_STREAM("[screen_to_world] algorithm=" << algorithm_str << ", timeout=" << timeout);

	if (algorithm_str == "CONTOURS")
	{
		algorithm = CONTOURS;
	}
	else if (algorithm_str == "CONTOURS_NON_ADAPTIVE")
	{
		algorithm = CONTOURS_NON_ADAPTIVE;
	}
	else if (algorithm_str == "K_MEANS")
	{
		algorithm = K_MEANS;
	}
	else if (algorithm_str == "TWO_D_ONLY")
	{
		algorithm = TWO_D_ONLY;
	}
	else
	{
		ROS_ERROR_STREAM("Unknown depth algorithm type : " << algorithm_str);
		return -1;
	}

	// All algorithms require depth info, with the exception of TWO_D_ONLY
	// For the depth algorithms, create a ROS synchronizer to collect
	// image + depth into a single callback
	std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
	std::unique_ptr<message_filters::Subscriber<field_obj::TFDetection>> obsub;
	typedef message_filters::sync_policies::ApproximateTime<field_obj::TFDetection, sensor_msgs::Image> ObjDepthSyncPolicy;
	std::unique_ptr<message_filters::Synchronizer<ObjDepthSyncPolicy>> obj_depth_sync;

	// For the 2-D only mode, just pass in a dummy image ptr - this should never
	// be used so hopefully just a default initialized sensor img will work as a placeholder
	ros::Subscriber sub;
	boost::shared_ptr<sensor_msgs::Image> dummy_image_ptr(new sensor_msgs::Image());

	cv::Mat img(cv::Size(720, 1280), CV_32FC1, cv::Scalar(0));

	std_msgs::Header header;		 // empty header
	header.seq = 0;					 // user defined counter
	header.stamp = ros::Time::now(); // time
	cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, img);
	img_bridge.toImageMsg(*dummy_image_ptr); // from cv_bridge to sensor_msgs::Image

	if (algorithm != TWO_D_ONLY)
	{
		// Create a filter subscriber to camera depth info
		depth_sub = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_objdetect/depth/depth_registered", 1);

		// And another filer subscriber to the TF object detection
		obsub = std::make_unique<message_filters::Subscriber<field_obj::TFDetection>>(nh, "obj_detection_msg", 1);

		// Create a synchronizer which combines the two - it calls the callback function when it matches
		// up a message from each subscriber which have the (approximately) same timestamps
		// TODO - try this with an exact synchronizer?
		obj_depth_sync = std::make_unique<message_filters::Synchronizer<ObjDepthSyncPolicy>>(ObjDepthSyncPolicy(10), *obsub, *depth_sub);

		obj_depth_sync->setMaxIntervalDuration(ros::Duration(timeout));
		obj_depth_sync->registerCallback(boost::bind(callback, _1, _2));
	}
	else
	{
		sub = nh.subscribe<field_obj::TFDetection>("obj_detection_msg", 2, boost::bind(callback, _1, dummy_image_ptr));
	}

	// Set up a simple subscriber to capture camera info
	ros::Subscriber camera_info_sub_ = nh.subscribe("/zed_objdetect/left/camera_info", 2, camera_info_callback);

	// And a publisher to published converted 3d coords
	pub = nh.advertise<field_obj::Detection>("object_detection_world", 2);

	ros::spin();
}
