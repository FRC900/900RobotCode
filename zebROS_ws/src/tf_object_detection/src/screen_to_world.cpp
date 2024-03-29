#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "field_obj/Detection.h"
#include "field_obj/TFDetection.h"
#include "field_obj_tracker/convert_coords.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pluginlib/class_list_macros.h>

#include "depth_algorithms.h"

namespace tf_object_detection 
{
	class ScreenToWorld : public nodelet::Nodelet {
	public:
		ScreenToWorld() = default;
		virtual ~ScreenToWorld() = default;
	private:
		void onInit() override {
			NODELET_INFO("********** Starting nodelet '%s' **********", getName().c_str());
            ros::NodeHandle nh = getNodeHandle();
			ros::NodeHandle nhp = getMTPrivateNodeHandle();
			image_transport::ImageTransport it(nh);

			std::string algorithm_str;
			double timeout;

			nhp.param<std::string>("depth_algorithm", algorithm_str, "CONTOURS");
			nhp.param<double>("timeout", timeout, 0.05);

			NODELET_INFO_STREAM("[screen_to_world] algorithm=" << algorithm_str << ", timeout=" << timeout);

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
				NODELET_ERROR_STREAM("Unknown depth algorithm type : " << algorithm_str);
				return;
			}

			// All algorithms require depth info, with the exception of TWO_D_ONLY
			// For the depth algorithms, create a ROS synchronizer to collect
			// image + depth into a single callback
			// For the 2-D only mode, just pass in a dummy image ptr - this should never
			// be used so hopefully just a default initialized sensor img will work as a placeholder
			dummy_image_ptr_.reset(new sensor_msgs::Image());

			img_ = cv::Mat(cv::Size(720, 1280), CV_32FC1, cv::Scalar(0));

			std_msgs::Header header;		 // empty header
			header.seq = 0;					 // user defined counter
			header.stamp = ros::Time::now(); // time
			img_bridge_ = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, img_);
			img_bridge_.toImageMsg(*dummy_image_ptr_); // from cv_bridge to sensor_msgs::Image

			if (algorithm != TWO_D_ONLY)
			{
				// Create a filter subscriber to camera depth info
				depth_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_objdetect/depth/depth_registered", 1);

				// And another filer subscriber to the TF object detection
				obsub_ = std::make_unique<message_filters::Subscriber<field_obj::TFDetection>>(nh, "obj_detection_msg", 1);

				// Create a synchronizer which combines the two - it calls the callback function when it matches
				// up a message from each subscriber which have the (approximately) same timestamps
				// TODO - try this with an exact synchronizer?
				obj_depth_sync_ = std::make_unique<message_filters::Synchronizer<ObjDepthSyncPolicy>>(ObjDepthSyncPolicy(10), *obsub_, *depth_sub_);

				obj_depth_sync_->setMaxIntervalDuration(ros::Duration(timeout));
				obj_depth_sync_->registerCallback(boost::bind(&ScreenToWorld::callback, this, _1, _2));
			}
			else
			{
				sub_ = nh.subscribe<field_obj::TFDetection>("obj_detection_msg", 1, boost::bind(&ScreenToWorld::callback, this, _1, dummy_image_ptr_));
			}

			// Set up a simple subscriber to capture camera info
			camera_info_sub_ = nh.subscribe("/zed_objdetect/left/camera_info", 1, &ScreenToWorld::camera_info_callback, this);

			// And a publisher to published converted 3d coords
			pub_ = nhp.advertise<field_obj::Detection>("object_detection_world", 1);
		};
		void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info) {
			std::scoped_lock l(cam_info_mutex_);
			caminfo = *info;
			caminfovalid = true;
		};
		void callback(const field_obj::TFDetectionConstPtr &objDetectionMsg, const sensor_msgs::ImageConstPtr &depthMsg) {
			std::scoped_lock l(cam_info_mutex_);
			if (!caminfovalid)
			{
				return;
			}

			cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

			// Initialize published message with header info from objDetectionMsg
			field_obj::Detection out_msg;
			out_msg.header = objDetectionMsg->header;
			// TODO : replace with a transform to base link?
			// Or at least to the non-optical_frame version of the camera frame
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
					NODELET_INFO_STREAM("less than 1 x");
					continue;
				}

				if (fabs(camObject.br.y - camObject.tl.y) < 1) {
					NODELET_INFO_STREAM("less than 1 x");
					continue;
				}
				// Create an output object, copy over info from the object detection info
				field_obj::Object worldObject;
				worldObject.id = camObject.label;
				worldObject.confidence = camObject.confidence;
				// Generate a bounding rect (in camera coords) from the camera object
				const cv::Point rectTL(std::floor(camObject.tl.x), std::ceil(camObject.tl.y));
				const cv::Point rectBR(std::floor(camObject.br.x), std::ceil(camObject.br.y));
				const cv::Rect  rect(rectTL, rectBR);

				// Get the distance to the object by finding contours within the depth data inside the
				// object's bounding rectangle.
				const float objDistance = usefulDepthMat(cvDepth->image, rect, algorithm, false);
				if (objDistance < 0 || isnan(objDistance))
				{
					// Find the center of the detected object
					const cv::Point2f objRectCenter = 0.5 * (rectTL + rectBR);
					NODELET_ERROR_STREAM_THROTTLE(0.5, "Depth of object at " << objRectCenter << " with bounding rect " << rect << " invalid : " << objDistance);
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

			pub_.publish(out_msg);

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
		};

		std::unique_ptr<image_transport::SubscriberFilter> depth_sub_;
		std::unique_ptr<message_filters::Subscriber<field_obj::TFDetection>> obsub_;
		using ObjDepthSyncPolicy = message_filters::sync_policies::ApproximateTime<field_obj::TFDetection, sensor_msgs::Image>;
		std::unique_ptr<message_filters::Synchronizer<ObjDepthSyncPolicy>> obj_depth_sync_;

		boost::shared_ptr<sensor_msgs::Image> dummy_image_ptr_;
		cv::Mat img_;
		cv_bridge::CvImage img_bridge_;

		ros::Subscriber sub_;
		ros::Publisher pub_;
		ros::Subscriber camera_info_sub_;
		// I don't know how to set default values for these class members
		DepthCalculationAlgorithm algorithm;
		ros::Publisher pub;
		sensor_msgs::CameraInfo caminfo;
		bool caminfovalid{false};
		std::mutex cam_info_mutex_;
	};
}

PLUGINLIB_EXPORT_CLASS(tf_object_detection::ScreenToWorld, nodelet::Nodelet)