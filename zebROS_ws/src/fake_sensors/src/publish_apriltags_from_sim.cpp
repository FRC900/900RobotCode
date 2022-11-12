// This node republishes fiducial markers corresponding to apriltags as the same
// transforms that the apriltag detector sends
// (we aren't using a camera in sim so we can't publish an AprilTagDetectionArray)
#include <random>
#include <ros/ros.h>
#include <marker_msgs/MarkerDetection.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <fstream>

// see behaviors/config/2022_rumble_apriltag_locations.yaml for Z values
// maybe create dictionary (tag ID (int) : Z (double)) instead?

class Tag {
public:
	double x;
	double y;
	double z;
	bool inHub;
	Tag(double _x = 0, double _y = 0, double _z = 0, bool _inHub = false) {
		this->x = _x;
		this->y = _y;
		this->z = _z;
		this->inHub = _inHub;
	}
};

class SimAprilTagPub
{
	public:
		SimAprilTagPub(ros::NodeHandle &n)
			: sub_(n.subscribe("/base_marker_detection", 2, &SimAprilTagPub::msgCallback, this))

		{
			ROS_INFO_STREAM("Hi");
			XmlRpc::XmlRpcValue xmlTags;
			n.getParam("tags", xmlTags);
			ROS_INFO_STREAM(xmlTags << std::endl);
			for (auto pair : xmlTags) {
				ROS_INFO_STREAM(pair.first << std::endl);
				ROS_INFO_STREAM(pair.second);
				tags_[std::stoi(pair.first)] = Tag(static_cast<double>(pair.second[0]), static_cast<double>(pair.second[1]), static_cast<double>(pair.second[2]), static_cast<bool>(pair.second[3]));
			}
		}

		// Translate stage base_marker_detection into our custom goal detection message
		void msgCallback(const marker_msgs::MarkerDetectionConstPtr &msgIn)
		{
			static tf2_ros::TransformBroadcaster br;
			static tf2_ros::Buffer tfBuffer;
  		static tf2_ros::TransformListener tfListener(tfBuffer);
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
				transformStamped.transform.translation.z = tags_[msgIn->markers[i].ids[0]-100].z;

				// Can't detect rotation yet, so publish 0 instead
				tf2::Quaternion q;
				q.setRPY(0, 0, 0);

				transformStamped.transform.rotation.x = q.x();
				transformStamped.transform.rotation.y = q.y();
				transformStamped.transform.rotation.z = q.z();
				transformStamped.transform.rotation.w = q.w();

				br.sendTransform(transformStamped);
			}
			// pub_.publish(msgOut);
			// pubd_.publish(msgOut);
		}

	private:
		ros::Subscriber            sub_;
		std::map<int, Tag>         tags_; // tag id to tag info
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_apriltag_publisher");

	ros::NodeHandle nh;
	SimAprilTagPub simAprilTagPub(nh);

	ros::spin();
	return 0;
}
