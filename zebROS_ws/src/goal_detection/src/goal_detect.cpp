#include <iostream>
#include <mutex>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>

#include "teraranger_array/RangeArray.h"

#include "field_obj/Detection.h"
#include "field_obj/Object.h"
#include "objtype.hpp"
#include "convert_coords.h"

#include "GoalDetector.hpp"
#include "goal_detection/GoalDetectionConfig.h"
#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"

namespace goal_detection
{
	class GoalDetect : public nodelet::Nodelet
	{
		public:

			GoalDetect(void)
			{
			}

			~GoalDetect()
			{
			}

		protected:
			void onInit() override
			{
				nh_ = getMTPrivateNodeHandle();
				image_transport::ImageTransport it(nh_);
				int sub_rate = 1;
				int pub_rate = 1;
				nh_.getParam("sub_rate", sub_rate);
				nh_.getParam("pub_rate", pub_rate);

				nh_.param("hFov", config_.hFov, 105.);
				nh_.param("camera_angle", config_.camera_angle, -25.0);
				nh_.param("blue_scale", config_.blue_scale, 0.90);
				nh_.param("red_scale", config_.red_scale, 0.80);
				nh_.param("otsu_threshold", config_.otsu_threshold, 5);
				nh_.param("min_confidence", config_.min_confidence, 0.30);
				drw_.init(nh_, config_);

				bool no_depth = false;
				nh_.getParam("no_depth", no_depth);

				distance_from_terabee_ = -1;

				if (!no_depth)
				{
					ROS_INFO("starting goal detection using ZED");
					frame_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_goal/left/image_rect_color", sub_rate);
					depth_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_goal/depth/depth_registered", sub_rate);
					// ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(xxx)
					rgbd_sync_ = std::make_unique<message_filters::Synchronizer<RGBDSyncPolicy>>(RGBDSyncPolicy(10), *frame_sub_, *depth_sub_);
					rgbd_sync_->registerCallback(boost::bind(&GoalDetect::callback, this, _1, _2));

					camera_info_sub_ = nh_.subscribe("/zed_goal/left/camera_info", sub_rate, &GoalDetect::camera_info_callback, this);
				}
				else
				{
					ROS_INFO("starting goal detection using webcam");
					rgb_sub_ = std::make_unique<image_transport::Subscriber>(it.subscribe("/c920_camera/image_raw", sub_rate, &GoalDetect::callback_no_depth, this));
					terabee_sub_ = nh_.subscribe("/multiflex_1/ranges_raw", 1, &GoalDetect::multiflexCB, this);
				}

				// Set up publisher
				pub_ = nh_.advertise<field_obj::Detection>("goal_detect_msg", pub_rate);

				pub_debug_image_ = it.advertise("debug_image", 2);
			}

			void callback(const sensor_msgs::ImageConstPtr &frameMsg, const sensor_msgs::ImageConstPtr &depthMsg)
			{
				//std::lock_guard<std::mutex> l(camera_mutex_);
				if (!camera_mutex_.try_lock())  // If the previous message is still being
					return;              // processed, drop this one
				cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
				cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

				// Initialize goal detector object the first time
				// through here. Use the size of the frame
				// grabbed from the ZED messages
				if (gd_ == NULL)
				{
					const cv::Point2f fov(config_.hFov * (M_PI / 180.),
										  config_.hFov * (M_PI / 180.) * ((double)cvFrame->image.rows / cvFrame->image.cols));
					gd_ = std::make_unique<GoalDetector>(fov, cvFrame->image.size(), false);
				}
				gd_->setCameraAngle(config_.camera_angle);
				gd_->setBlueScale(config_.blue_scale);
				gd_->setRedScale(config_.red_scale);
				gd_->setOtsuThreshold(config_.otsu_threshold);
				gd_->setMinConfidence(config_.min_confidence);

				//Send current color and depth image to the actual GoalDetector
				gd_->setTargetNum(POWER_PORT_2020);
				gd_->findTargets(cvFrame->image, cvDepth->image);
				std::vector< GoalFound > gfd_power_port = gd_->return_found();

				gd_->setTargetNum(LOADING_BAY_2020);
				gd_->findTargets(cvFrame->image, cvDepth->image);
				std::vector< GoalFound > gfd_loading_bay = gd_->return_found();

				std::vector< GoalFound > gfd;
				gfd.reserve( gfd_power_port.size() + gfd_loading_bay.size() );
				gfd.insert( gfd.end(), gfd_power_port.begin(), gfd_power_port.end() );
				gfd.insert( gfd.end(), gfd_loading_bay.begin(), gfd_loading_bay.end() );

				field_obj::Detection gd_msg;

				gd_msg.header.seq = frameMsg->header.seq;
				gd_msg.header.stamp = frameMsg->header.stamp;
				std::string frame_id = frameMsg->header.frame_id;
				// Remove _optical_frame from the camera frame ID if present
				const size_t idx = frame_id.rfind("_optical_frame");
				if (idx != std::string::npos)
				{
					frame_id.erase(idx);
					frame_id += "_frame";
				}
				gd_msg.header.frame_id = frame_id;

				image_geometry::PinholeCameraModel model;
				model.fromCameraInfo(camera_info_);
				ConvertCoords cc(model);

				#if 0
					ROS_INFO_STREAM("Camera_info " << camera_info_);
				#endif

				for(size_t i = 0; i < gfd.size(); i++)
				{
					field_obj::Object dummy;
					dummy.id = gfd[i].id;
					dummy.confidence = gfd[i].confidence;

					// Bounding rect in world coords
					const cv::Point3f world_coord_scaled = cc.screen_to_world(gfd[i].rect, dummy.id, gfd[i].distance);

					dummy.location.x = world_coord_scaled.z;
					dummy.location.y = world_coord_scaled.x;
					dummy.location.z = world_coord_scaled.y;
					dummy.angle = atan2f(world_coord_scaled.x, world_coord_scaled.y) * 180. / M_PI;
					gd_msg.objects.push_back(dummy);
				}

				pub_.publish(gd_msg);

				if (pub_debug_image_.getNumSubscribers() > 0)
				{
					cv::Mat thisFrame(cvFrame->image.clone());
					gd_->drawOnFrame(thisFrame, gd_->getContours(thisFrame), gfd);
					pub_debug_image_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", thisFrame).toImageMsg());
				}

				if (gd_msg.objects.size() > 0)
				{
					//Transform between goal frame and odometry/map.
					static tf2_ros::TransformBroadcaster br;
					for(size_t i = 0; i < gfd.size(); i++)
					{
						geometry_msgs::TransformStamped transformStamped;

						transformStamped.header.stamp = gd_msg.header.stamp;
						transformStamped.header.frame_id = frame_id;
						std::stringstream child_frame;
						child_frame << "goal_";
						child_frame << i;
						transformStamped.child_frame_id = child_frame.str();

						transformStamped.transform.translation.x = gd_msg.objects[i].location.x;
						transformStamped.transform.translation.y = gd_msg.objects[i].location.y;
						transformStamped.transform.translation.z = gd_msg.objects[i].location.z;

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
				camera_mutex_.unlock();
			}

			void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info)
			{
				if (!camera_mutex_.try_lock())  // If the previous message is still being
					return;              // processed, drop this one

				camera_info_ = *info;
				camera_mutex_.unlock();
			}

			void multiflexCB(const teraranger_array::RangeArray& msg)
			{
				// If previous message is still being processed, drop
				// this one.
				if (!multiflex_mutex_.try_lock())
					return;
				double min_dist = std::numeric_limits<double>::max();
				distance_from_terabee_ = -1;
				for(int i = 0; i < 2; i++)
				{
					const double range = static_cast<double>(msg.ranges[i].range);
					if(!std::isnan(range))
						min_dist = std::min(min_dist, range);
				}
				if (min_dist != std::numeric_limits<double>::max())
					distance_from_terabee_ = min_dist;
				multiflex_mutex_.unlock();
			}

			void callback_no_depth(const sensor_msgs::ImageConstPtr &frameMsg)
			{
				cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
				cv::Mat depthMat(cvFrame->image.size(), CV_32FC1, cv::Scalar(distance_from_terabee_));
				callback(frameMsg, cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthMat).toImageMsg());
			}

			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDSyncPolicy;

			ros::NodeHandle                                                nh_;
			std::unique_ptr<image_transport::SubscriberFilter>             frame_sub_;
			std::unique_ptr<image_transport::SubscriberFilter>             depth_sub_;
			std::unique_ptr<image_transport::Subscriber>                   rgb_sub_;
			std::unique_ptr<message_filters::Synchronizer<RGBDSyncPolicy>> rgbd_sync_;
			ros::Subscriber                                                terabee_sub_;
			ros::Publisher                                                 pub_;
			image_transport::Publisher                                     pub_debug_image_;
			std::unique_ptr<GoalDetector>                                  gd_;
			double                                                         distance_from_terabee_;
			goal_detection::GoalDetectionConfig                            config_;
			DynamicReconfigureWrapper<goal_detection::GoalDetectionConfig> drw_;
			std::mutex                                                     camera_mutex_;
			std::mutex                                                     multiflex_mutex_;

			ros::Subscriber                                                camera_info_sub_;
			sensor_msgs::CameraInfo                                        camera_info_;
	};
} // namespace

PLUGINLIB_EXPORT_CLASS(goal_detection::GoalDetect, nodelet::Nodelet)
