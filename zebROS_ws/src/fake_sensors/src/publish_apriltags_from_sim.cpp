// This node republishes fiducial markers corresponding to apriltags as the same
// transforms that the apriltag detector sends
// (we aren't using a camera in sim so we can't publish an AprilTagDetectionArray)
// ... why can't we again?
#include <ros/ros.h>
#include <marker_msgs/MarkerDetection.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// Array Stamped is same as pose stamped but also has 3d poses of the tags
// going to publish both because tagslam expected the one without poses, but other code will likley want with poses 
#include <apriltag_msgs/ApriltagArrayStamped.h> 
#include <apriltag_msgs/ApriltagPoseStamped.h> 

#include <apriltag_msgs/Apriltag.h> // induvidual elements 

#include <field_obj/Detection.h>
#include <std_msgs/Header.h>

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

field_obj::Object apriltagDetectionObject(uint32_t id, double x, double y, double z) {
	field_obj::Object obj;
	obj.location.x = x;
	obj.location.y = y;
	obj.location.z = z;
	obj.angle = atan2(y, x);
	obj.id = std::to_string(id);
	obj.confidence = 1.0;
	return obj;
}

std::pair<apriltag_msgs::Apriltag, geometry_msgs::Pose> createAprilTag(uint32_t id, double x, double y, double z, const std_msgs::Header &header) {
	apriltag_msgs::Apriltag tag_msg;
	tag_msg.id = {id};
	tag_msg.family = "sim";
	// doesn't really make sense to publish corners in sim
	// would be cool for testing tagslam but without camera calibration isn't useful	
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = x;
	pose_msg.position.y = y;
	pose_msg.position.z = z;
	tf2::Quaternion q;
	q.setRPY(0, 0, 0);
	pose_msg.orientation.x = q.x();
	pose_msg.orientation.y = q.y();
	pose_msg.orientation.z = q.z();
	pose_msg.orientation.w = q.w();
	//tag_msg.pose.header = header;
	return std::make_pair(tag_msg, pose_msg);
}

#include <string>


class SimAprilTagPub
{
	public:
		SimAprilTagPub(ros::NodeHandle &n)
			: sub_(n.subscribe("/base_marker_detection", 2, &SimAprilTagPub::msgCallback, this)),
			  pub_(n.advertise<apriltag_msgs::ApriltagPoseStamped>("/tag_detections", 1000)),
			  tfPub_(n.advertise<field_obj::Detection>("/tf_object_detection/tag_detection_world", 1000))

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
				transformStamped.child_frame_id = "36h11" + std::to_string(msgIn->markers[i].ids[0]-100);

				transformStamped.transform.translation.x = p.x;
				transformStamped.transform.translation.y = p.y;
				transformStamped.transform.translation.z = p.z;

				// Can't detect rotation yet, so publish 0 instead
				tf2::Quaternion q;
				q.setRPY(0, 0, 0);

				transformStamped.transform.rotation.x = q.x();
				transformStamped.transform.rotation.y = q.y();
				transformStamped.transform.rotation.z = q.z();
				transformStamped.transform.rotation.w = q.w();

				br_.sendTransform(transformStamped);

				auto[apriltag, pose] = createAprilTag(msgIn->markers[i].ids[0]-100, p.x, p.y, p.z, msgIn->header);
				detections.push_back(apriltag);
				poses.push_back(pose);

				objects.push_back(apriltagDetectionObject(msgIn->markers[i].ids[0]-100, p.x, p.y, p.z));

			}

			apriltag_msgs::ApriltagPoseStamped apriltag_msg;
			apriltag_msg.apriltags = detections;
			apriltag_msg.posearray.poses = poses;
			// should maybe also set header for posearray, not sure 
			apriltag_msg.header = msgIn->header;
			

			field_obj::Detection detMsg;
			detMsg.header = msgIn->header;
			detMsg.objects = objects;

			pub_.publish(apriltag_msg);
			tfPub_.publish(detMsg);
			// pub_.publish(msgOut);
			// pubd_.publish(msgOut);
		}

	private:
		ros::Subscriber               sub_;
		ros::Publisher                pub_;
		ros::Publisher                tfPub_;
		tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_apriltag_publisher");

	ros::NodeHandle nh;
	SimAprilTagPub simAprilTagPub(nh);

	ros::spin();
	return 0;
}
