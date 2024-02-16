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

std::map<int, std::string> parsePBTXT(std::string filename) {
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

class FakeGoalDetection
{
	public:
		FakeGoalDetection(ros::NodeHandle &n, const std::map<int, std::string> &objMap)
			: rd_{}
			, gen_{rd_()}
			, covariance_(0.0004)
			, sub_(n.subscribe("base_marker_detection", 2, &FakeGoalDetection::cmdVelCallback, this))
			, pub_(n.advertise<field_obj::Detection>("goal_detect_msg", 2))
			, pubd_(n.advertise<field_obj::Detection>("/tf_object_detection_zed_front/object_detection_world", 2))
			, objMap_(objMap)

		{
			n.param("covariance", covariance_, covariance_);
			normalDistribution_ = std::normal_distribution<double>{0, sqrt(covariance_)};
		}

		// Translate stage base_marker_detection into our custom goal detection message
		void cmdVelCallback(const marker_msgs::MarkerDetectionConstPtr &msgIn)
		{
			field_obj::Detection msgOut;
			msgOut.header = msgIn->header;
			static tf2_ros::TransformBroadcaster br;
			for(size_t i = 0; i < msgIn->markers.size(); i++)
			{
				if (msgIn->markers[i].ids[0] == -1) // stage publishes odom as marker -1
					continue;                       // ignore it here
				// check if the id - 100 is a positive number, sim only hack for apriltags
				if (msgIn->markers[i].ids[0] >= 100) {
					field_obj::Object dummy;

					const auto &p = msgIn->markers[i].pose.position;
					dummy.location.x = p.x + normalDistribution_(gen_);
					dummy.location.y = p.y + normalDistribution_(gen_);
					dummy.location.z = p.z + normalDistribution_(gen_);
					dummy.angle = atan2(dummy.location.y, dummy.location.x) * 180. / M_PI;
					dummy.confidence = msgIn->markers[i].ids_confidence[0];
					dummy.id = std::to_string(msgIn->markers[i].ids[0] - 100);
					msgOut.objects.push_back(dummy);
					continue;
				}

				if (objMap_.find(msgIn->markers[i].ids[0]) != objMap_.end()) { // if ID in map
					field_obj::Object obj;

					const auto &p = msgIn->markers[i].pose.position;
					obj.location.x = p.x;
					obj.location.y = p.y;
					obj.location.z = p.z;
					
					obj.angle = atan2(obj.location.y, obj.location.x) * 180. / M_PI;
					obj.confidence = msgIn->markers[i].ids_confidence[0];
					obj.id = objMap_[msgIn->markers[i].ids[0]];
					if (obj.id == "note") {
						// ROS_INFO_STREAM("Found a note!");
						if (hypot(p.x, p.y) < 2.0) {
							ROS_INFO_STREAM_THROTTLE(1, "Note too close! Dropping");
							continue;
						}
 					}
					msgOut.objects.push_back(obj);

					geometry_msgs::TransformStamped transformStamped;

					transformStamped.header.stamp = msgOut.header.stamp;
					transformStamped.header.frame_id = msgOut.header.frame_id;
					std::stringstream child_frame;
					child_frame << obj.id << "_" << i;
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

					br.sendTransform(transformStamped);
				} else {
					field_obj::Object dummy;

					const auto &p = msgIn->markers[i].pose.position;
					dummy.location.x = p.x + normalDistribution_(gen_);
					dummy.location.y = p.y + normalDistribution_(gen_);
					dummy.location.z = p.z + normalDistribution_(gen_);
					dummy.angle = atan2(dummy.location.y, dummy.location.x) * 180. / M_PI;
					dummy.confidence = msgIn->markers[i].ids_confidence[0];
					dummy.id = std::to_string(msgIn->markers[i].ids[0]);
					ROS_INFO_STREAM("Saw else " << dummy.id);

					msgOut.objects.push_back(dummy);
				}
			}
			pub_.publish(msgOut);
			pubd_.publish(msgOut);
		}

	private:
		std::random_device rd_;
		std::mt19937 gen_;
		std::normal_distribution<double> normalDistribution_;
		double covariance_;
		ros::Subscriber            sub_;
		ros::Publisher             pub_;
		ros::Publisher             pubd_; // d for detection
		std::map<int, std::string> objMap_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_goal_detect");

	ros::NodeHandle nh;
	FakeGoalDetection fakeGoalDetection(nh, parsePBTXT("/home/ubuntu/2023RobotCode/zebROS_ws/src/tf_object_detection/src/FRC2024.yaml"));

	ros::spin();
	return 0;
}
