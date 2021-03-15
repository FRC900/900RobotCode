#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "field_obj/TFDetection.h"
bool caminfovalid{false};
ros::Publisher pub_;
sensor_msgs::CameraInfo caminfo;
void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info)
{
caminfo = *info;
caminfovalid = true;

	return;
}

void callback(const field_obj::TFDetectionConstPtr &obj_detection, const sensor_msgs::ImageConstPtr &depthMsg)
{if (!caminfovalid)

	return;

	cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::    image_encodings::TYPE_32FC1);

	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(camera_info_);
	
		field_obj::Detection gd_msg;

	gd_msg.header.seq = frameMsg->header.seq;
    gd_msg.header.stamp = frameMsg->header.stamp;
    std::string frame_id = frameMsg->header.frame_id;

	ConvertCoords cc(model);
	 for(size_t i = 0; i < gfd.size(); i++)
                 {      
				 field_obj::Object dummy;
                     dummy.id = gfd[i].id;
                     dummy.confidence = gfd[i].confidence; 
                    const cv::Point3f world_coord_scaled = cc.screen_to_world(gfd[i].rect, dummy.    id, gfd[i].distance);

                    dummy.location.x = world_coord_scaled.z;
                    dummy.location.y = -world_coord_scaled.x;
                    dummy.location.z = world_coord_scaled.y;
                    dummy.angle = atan2f(world_coord_scaled.x, world_coord_scaled.y) * 180. / M_P    I;
                     gd_msg.objects.push_back(dummy);
               }
pub_.publish(gd_msg);
}
int main (int argc, char**argv){
typedef message_filters::sync_policies::ApproximateTime<field_obj::TFDetection, sensor_msgs::Image> RGBDSyncPolicy;
ros::NodeHandle nh;
image_transport::ImageTransport it(nh);

	  std::unique_ptr<image_transport::SubscriberFilter>             depth_sub_;
	depth_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_goal/depth/depth_registered",1);

	pub_ = nh.advertise<field_obj::Detection>("object_detection_world", pub_rate);

	message_filters::Subscriber<field_obj::TFDetection> Obsub (nh,"obj_detection_msg",1);
	   std::unique_ptr<message_filters::Synchronizer<RGBDSyncPolicy>> rgbd_sync_;
	rgbd_sync_ = std::make_unique<message_filters::Synchronizer<RGBDSyncPolicy>>(RGBDSyncPolicy(10), Obsub, *depth_sub_);

rgbd_sync_->registerCallback(boost::bind(callback, _1, _2)    );
ros::Subscriber camera_info_sub_ = nh.subscribe("/zed_goal/left/camera_info", 1, camera_info_callback);

	
 }
