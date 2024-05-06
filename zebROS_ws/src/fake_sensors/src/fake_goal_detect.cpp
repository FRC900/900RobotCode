// Simple node to generate a fake goal detection message in simulation
// Reads base_marker_detection message from stage ros, converts to our custom
// goal detect message. Adds noise to x and y positions of the converted detections
#include <random>
#include <ros/ros.h>
#include <marker_msgs/MarkerDetection.h>
#include "field_obj/Detection.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <fstream>

std::map<int, std::string> parsePBTXT(const std::string &filename) {
	std::map<int, std::string> pbtxtMap;
	std::regex pbtxtRegex("(\\d+): *(.+)"); // this feels like an evil thing to do, but whatever
	// regexr.com/6dql5
	std::ifstream t(filename);
	std::stringstream fileContents;
	fileContents << t.rdbuf();
	std::smatch res;
	std::string str = fileContents.str();

	std::string::const_iterator searchStart(str.cbegin());
	while (std::regex_search(searchStart, str.cend(), res, pbtxtRegex))
	{
		std::cout << res[1] << " = " << res[2] << std::endl;
		// +1 is because yaml is 0 indexed but stage doesn't like markers with id 0
		pbtxtMap[std::stoi(res[1]) + 1] = res[2];
		searchStart = res.suffix().first;
	}
	return pbtxtMap;
}

// Search for a leading / and, if found, remove it
// Used to clean up frame_id strings
std::string &rtrim(std::string &str)
{
	if (str[0] == '/') {
		str.erase(0, 1);
	}
	return str;   
}

class FakeGoalDetection
{
	public:
		FakeGoalDetection(ros::NodeHandle &n, const std::map<int, std::string> &objMap)
			: sub_(n.subscribe("base_marker_detection", 2, &FakeGoalDetection::cmdVelCallback, this))
			, pubt_(n.advertise<field_obj::Detection>("tag_detection_world", 2))
			, objMap_(objMap)

		{
			ros::NodeHandle n_private("~");
			n_private.param("covariance", covariance_, covariance_);
			normalDistribution_ = std::normal_distribution<double>{0, sqrt(covariance_)};
			n_private.param("publish_detections", pubDetections_, pubDetections_);
			if (pubDetections_) {
				pubd_ = n.advertise<field_obj::Detection>("object_detection_world", 2);
			}
		}

		// Translate stage base_marker_detection into our custom goal detection message
		void cmdVelCallback(const ros::MessageEvent<marker_msgs::MarkerDetection> &event)
		{
			const ros::M_string &header = event.getConnectionHeader();
			const std::string topic = header.at("topic");
			const auto idx = topic.find_last_of('_');
			std::string frame_suffix = "";
			if (idx != std::string::npos && isdigit(topic[idx + 1])) {
					frame_suffix = topic.substr(idx);
			}
			const marker_msgs::MarkerDetectionConstPtr &msgIn = event.getConstMessage();
			field_obj::Detection detectionsOut;
			detectionsOut.header = msgIn->header;
			rtrim(detectionsOut.header.frame_id);
			field_obj::Detection tagsOut;
			tagsOut.header = msgIn->header;
			rtrim(tagsOut.header.frame_id);
			for(size_t i = 0; i < msgIn->markers.size(); i++)
			{
				const auto marker = msgIn->markers[i];
				if (marker.ids[0] == -1) // stage publishes odom as marker -1
					continue;			 // ignore it here
				// check if the id - 100 is a positive number, sim only hack for apriltags
				if (marker.ids[0] >= 100) {
					field_obj::Object tag;

					const auto &p = marker.pose.position;
					tag.location.x = p.x + normalDistribution_(gen_);
					tag.location.y = p.y + normalDistribution_(gen_);
					tag.location.z = p.z + normalDistribution_(gen_);
					tag.angle = atan2(tag.location.y, tag.location.x) * 180. / M_PI;
					tag.confidence = marker.ids_confidence[0];
					tag.id = std::to_string(marker.ids[0] - 100);
					tagsOut.objects.push_back(tag);
					sendTransform(tagsOut.header, tag, i, "obj_", frame_suffix);
				} else if (pubDetections_) {
					if (objMap_.find(marker.ids[0]) != objMap_.end()) { // if ID in map
						field_obj::Object obj;

						const auto &p = marker.pose.position;
						obj.location.x = p.x;
						obj.location.y = p.y;
						obj.location.z = p.z;
						
						obj.angle = atan2(obj.location.y, obj.location.x) * 180. / M_PI;
						obj.confidence = marker.ids_confidence[0];
						obj.id = objMap_[marker.ids[0]];
						if (obj.id == "note") {
							// ROS_INFO_STREAM("Found a note!");
							if (hypot(p.x, p.y) < 1.0) {
								ROS_INFO_STREAM_THROTTLE(1, "Note too close! Dropping");
								//continue;
							}
						}
						detectionsOut.objects.push_back(obj);
						sendTransform(detectionsOut.header, obj, i, "", frame_suffix);
					} else {
						field_obj::Object dummy;

						const auto &p = marker.pose.position;
						dummy.location.x = p.x + normalDistribution_(gen_);
						dummy.location.y = p.y + normalDistribution_(gen_);
						dummy.location.z = p.z + normalDistribution_(gen_);
						dummy.angle = atan2(dummy.location.y, dummy.location.x) * 180. / M_PI;
						dummy.confidence = marker.ids_confidence[0];
						dummy.id = std::to_string(marker.ids[0]);
						ROS_INFO_STREAM("Saw else " << dummy.id);

						detectionsOut.objects.push_back(dummy);
					}
				}
			}
			pubt_.publish(tagsOut);
			if (pubDetections_) {
				pubd_.publish(detectionsOut);
			}
		}

	private:

		void sendTransform(const std_msgs::Header &header, const field_obj::Object &obj, const size_t i, const std::string &prefix, const std::string &suffix)
		{
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = header.stamp;
			transformStamped.header.frame_id = header.frame_id;
			std::stringstream child_frame;
			child_frame << prefix << obj.id << "_" << i << suffix;
			transformStamped.child_frame_id = child_frame.str();
 
 			transformStamped.transform.translation.x = obj.location.x;
 			transformStamped.transform.translation.y = obj.location.y;
 			transformStamped.transform.translation.z = obj.location.z;
 
 			// Can't detect rotation yet, so publish 0 instead
			tf2::Quaternion q;
			q.setRPY(0, 0, 0);

			transformStamped.transform.rotation.x = q.x();
			transformStamped.transform.rotation.y = q.y();
			transformStamped.transform.rotation.z = q.z();
			transformStamped.transform.rotation.w = q.w();

			br_.sendTransform(transformStamped);
		}

		std::random_device               rd_{};
		std::mt19937                     gen_{rd_()};
		std::normal_distribution<double> normalDistribution_;
		double                           covariance_{0.0004};
		ros::Subscriber                  sub_;
		ros::Publisher                   pubt_; // t for tags
		bool                             pubDetections_{true};
		ros::Publisher                   pubd_; // d for detection
		std::map<int, std::string>       objMap_;
		tf2_ros::TransformBroadcaster    br_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fake_goal_detect");

	ros::NodeHandle nh;
	FakeGoalDetection fakeGoalDetection(nh, parsePBTXT("/home/ubuntu/2023RobotCode/zebROS_ws/src/tf_object_detection/src/FRC2024.yaml"));

	ros::spin();
	return 0;
}
