// This node republishes fiducial markers corresponding to apriltags as the same
// transforms that the apriltag detector sends
// It also converts stage MarkerDetections into ApriltagArray/Pose Stamped messaages
// The center and corners are reprojected from 3d back to 2d using a camera model
//  - TODO grab the camera model info from tagslam camera configs
//  - TODO handle multiple sim cameras
#include <string>
#include <ros/ros.h>
#include <marker_msgs/MarkerDetection.h>

#include <geometry_msgs/TransformStamped.h>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// Array Stamped is same as pose stamped but also has 3d poses of the tags
// going to publish both because tagslam expected the one without poses, but other code will likley want with poses 
#include <apriltag_msgs/ApriltagArrayStamped.h> 
#include <apriltag_msgs/ApriltagPoseStamped.h> 

#include <apriltag_msgs/Apriltag.h> // induvidual elements 

#include <field_obj/Detection.h>
#include <std_msgs/Header.h>
#include <image_geometry/pinhole_camera_model.h>

// topic: tag_detections
/*
# ApriltagArrayStamped
std_msgs/Header header
Apriltag[] apriltags
geometry_msgs/PoseArray posearray

# ApriltagArrayStamped
std_msgs/Header header
Apriltag[] apriltags

# Apriltag
# This message contains an Apriltag Detection
int32 id                       # Id of the apriltag in its tag family, eg. 0
string family                  # Tag family, eg. tf36h11
int32 hamming                  # Hamming distance
int32 border                   # Black border
int32 bits                     # Bits, tf36h11 has 6 bits
geometry_msgs/Point center     # Center of this apriltag
geometry_msgs/Point[4] corners # Corners of this apriltag

*/

field_obj::Object apriltagDetectionObject(uint32_t id, const geometry_msgs::Point &p) {
	field_obj::Object obj;
	obj.location.x = p.x;
	obj.location.y = p.y;
	obj.location.z = p.z;
	obj.angle = atan2(p.y, p.x);
	obj.id = std::to_string(id);
	obj.confidence = 1.0;
	return obj;
}

std::pair<apriltag_msgs::Apriltag, geometry_msgs::Pose> createAprilTag(uint32_t id, const geometry_msgs::Point &p, const geometry_msgs::Quaternion &orientation, const image_geometry::PinholeCameraModel &camera_model, const double tag_size) {
	apriltag_msgs::Apriltag tag_msg;
	tag_msg.id = id;
	tag_msg.family = "sim";
	tag_msg.bits = 11;
	cv::Point2d screen_coord;
	screen_coord = camera_model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
	tag_msg.center.x = screen_coord.x;
	tag_msg.center.y = screen_coord.y;

	// This assumes we are always seeing the tag straight-on rather than at an
	// angle.  Not sure if stage provides correct angle information to do a
	// more accurate projection back to camera pixel coords.
	screen_coord = camera_model.project3dToPixel(cv::Point3d(p.x, p.y - tag_size/2, p.z - tag_size/2));
	tag_msg.corners[0].x = screen_coord.x;	
	tag_msg.corners[0].y = screen_coord.y;

	screen_coord = camera_model.project3dToPixel(cv::Point3d(p.x, p.y + tag_size/2, p.z - tag_size/2));
	tag_msg.corners[1].x = screen_coord.x;	
	tag_msg.corners[1].y = screen_coord.y;

	screen_coord = camera_model.project3dToPixel(cv::Point3d(p.x, p.y - tag_size/2, p.z + tag_size/2));
	tag_msg.corners[2].x = screen_coord.x;	
	tag_msg.corners[2].y = screen_coord.y;

	screen_coord = camera_model.project3dToPixel(cv::Point3d(p.x, p.y + tag_size/2, p.z + tag_size/2));
	tag_msg.corners[3].x = screen_coord.x;	
	tag_msg.corners[3].y = screen_coord.y;

	// doesn't really make sense to publish corners in sim
	// would be cool for testing tagslam but without camera calibration isn't useful	
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = p.x;
	pose_msg.position.y = p.y;
	pose_msg.position.z = p.z;
	pose_msg.orientation = orientation;

	return std::make_pair(tag_msg, pose_msg);
}



class SimAprilTagPub
{
	public:
		explicit SimAprilTagPub(ros::NodeHandle &n)
			: sub_(n.subscribe("/base_marker_detection", 2, &SimAprilTagPub::msgCallback, this))
            , tag_pub_(n.advertise<apriltag_msgs::ApriltagArrayStamped>("/tags", 2))
			, pose_pub_(n.advertise<apriltag_msgs::ApriltagPoseStamped>("/poses", 2))
			// , tfPub_(n.advertise<field_obj::Detection>("/tf_object_detection/tag_detection_world", 2))

		{
			ROS_INFO_STREAM("stripes and horns, Zebracorns!");
		}

		// Translate stage base_marker_detection into our custom goal detection message
		void msgCallback(const marker_msgs::MarkerDetectionConstPtr &msgIn)
		{
			// need both detections and poses to construct a ApriltagPoseStamped.msg 
			std::vector<apriltag_msgs::Apriltag> detections;
			std::vector<geometry_msgs::Pose> poses;
			std::vector<field_obj::Object> objects;

			for(size_t i = 0; i < msgIn->markers.size(); i++)
			{
				if (msgIn->markers[i].ids[0] == -1) // stage publishes odom as marker -1
					continue;                       // ignore it here
				if (msgIn->markers[i].ids[0] < 100)
					continue; // added 100 to apriltag ids

				geometry_msgs::TransformStamped transformStamped;

				const auto &p = msgIn->markers[i].pose.position;

				transformStamped.header.stamp = msgIn->header.stamp;
				transformStamped.header.frame_id = msgIn->header.frame_id;
				transformStamped.child_frame_id = "36h11" + std::to_string(msgIn->markers[i].ids[0] - 100);

				transformStamped.transform.translation.x = p.x;
				transformStamped.transform.translation.y = p.y;
				transformStamped.transform.translation.z = p.z;

				transformStamped.transform.rotation = msgIn->markers[i].pose.orientation;

				br_.sendTransform(transformStamped);

				auto [apriltag, pose] = createAprilTag(msgIn->markers[i].ids[0] - 100, p, msgIn->markers[i].pose.orientation, camera_model_, tag_size_);
				detections.push_back(apriltag);
				poses.push_back(pose);

				objects.push_back(apriltagDetectionObject(msgIn->markers[i].ids[0] - 100, p));
			}

			apriltag_msgs::ApriltagArrayStamped tags_msg;
			tags_msg.header = msgIn->header;
			tags_msg.apriltags = detections;

			apriltag_msgs::ApriltagPoseStamped poses_msg;
			poses_msg.header = msgIn->header;
			poses_msg.apriltags = detections;
			poses_msg.posearray.poses = poses;
			// should maybe also set header for posearray, not sure 

			field_obj::Detection detMsg;
			detMsg.header = msgIn->header;
			detMsg.objects = objects;

			tag_pub_.publish(tags_msg);
			pose_pub_.publish(poses_msg);
			// tfPub_.publish(detMsg);
		}

	private:
		ros::Subscriber               sub_;
        ros::Publisher                tag_pub_;
		ros::Publisher                pose_pub_;
		// ros::Publisher                tfPub_;
		tf2_ros::TransformBroadcaster br_;
		image_geometry::PinholeCameraModel camera_model_;
		double tag_size_{0.1651}; // 6.5 inches
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_apriltag_publisher");

	ros::NodeHandle nh;
	SimAprilTagPub simAprilTagPub(nh);

	ros::spin();
	return 0;
}
