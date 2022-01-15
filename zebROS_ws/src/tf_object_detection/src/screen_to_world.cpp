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
		return;

	cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

	// Initialize published message with header info from objDetectionMsg
	field_obj::Detection out_msg;
	out_msg.header = objDetectionMsg->header;

	// Create objects needed to convert from 2d screen to 3d world coords
	const ConvertCoords cc(caminfo);
	// Iterate over each object. Convert from camera to world coords.
	for (const auto &camObject : objDetectionMsg->objects)
	{
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
		const float objDistance = usefulDepthMat(cvDepth->image, rect, false, algorithm);
		if (objDistance < 0)
		{
			ROS_ERROR_STREAM("Depth of object at " << objRectCenter << " with bounding rect " << rect << " objDistance < 0 : " << objDistance);
			continue;
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
			child_frame << out_msg.objects[i].id;
			child_frame << "_";
			child_frame << i;
			transformStamped.child_frame_id = child_frame.str();

			transformStamped.transform.translation.x = out_msg.objects[i].location.x;
			transformStamped.transform.translation.y = out_msg.objects[i].location.y;
			transformStamped.transform.translation.z = out_msg.objects[i].location.z;

			// Can't detect rotation yet, so publish 0 instead
			tf2::Quaternion q;
			q.setRPY(0, 0, out_msg.objects[i].angle);

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
	image_transport::ImageTransport it(nh);

	// Create a filter subscriber to camera depth info
	std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
	depth_sub = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_objdetect/depth/depth_registered", 1);

	// And another filer subscriber to the TF object detection
	std::unique_ptr<message_filters::Subscriber<field_obj::TFDetection>> obsub;
	obsub = std::make_unique<message_filters::Subscriber<field_obj::TFDetection>>(nh, "obj_detection_msg", 1);

	// Create a synchronizer which combines the two - it calls the callback function when it matches
	// up a message from each subscriber which have the (approximately) same timestamps
	// TODO - try this with an exact synchronizer?
	typedef message_filters::sync_policies::ApproximateTime<field_obj::TFDetection, sensor_msgs::Image> ObjDepthSyncPolicy;
	std::unique_ptr<message_filters::Synchronizer<ObjDepthSyncPolicy>> obj_depth_sync;
	obj_depth_sync = std::make_unique<message_filters::Synchronizer<ObjDepthSyncPolicy>>(ObjDepthSyncPolicy(10), *obsub, *depth_sub);

	// obj_depth_sync->setMaxIntervalDuration(ros::Duration(1, 0)); // for testing rosbags
	obj_depth_sync->registerCallback(boost::bind(callback, _1, _2));

	// Set up a simple subscriber to capture camera info
	ros::Subscriber camera_info_sub_ = nh.subscribe("/zed_objdetect/left/camera_info", 2, camera_info_callback);

	// And a publisher to published converted 3d coords
	pub = nh.advertise<field_obj::Detection>("object_detection_world", 2);

	ros::spin();
}
