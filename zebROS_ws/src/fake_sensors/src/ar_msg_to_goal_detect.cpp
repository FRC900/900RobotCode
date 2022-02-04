// Simple node to generate a fake goal detection message in simulation
// Reads base_marker_detection message from stage ros, converts to our custom
// goal detect message. Adds noise to x and y positions of the converted detections
#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "field_obj/Detection.h"

class ArMsgToGoalDetect
{
	public:
		ArMsgToGoalDetect(ros::NodeHandle &n)
			: sub_(n.subscribe("/ar_pose_marker", 2, &ArMsgToGoalDetect::cmdVelCallback, this))
			, pub_(n.advertise<field_obj::Detection>("goal_detect_msg", 2))

		{
		}

		// Translate ar_track_alvar marker msg into our custom goal detection message
		void cmdVelCallback(const ar_track_alvar_msgs::AlvarMarkers &msgIn)
		{
			if (msgIn.markers.size() == 0)
				return;

			field_obj::Detection msgOut;
			msgOut.header = msgIn.markers[0].header;
			if (msgOut.header.frame_id[0] == '/')
				msgOut.header.frame_id = msgOut.header.frame_id.substr(1, msgOut.header.frame_id.size());

			for (const auto &m : msgIn.markers)
			{
				field_obj::Object dummy;

				const auto &p = m.pose.pose.position;
				dummy.location.x = p.x;
				dummy.location.y = p.y;
				dummy.location.z = p.z;
				dummy.angle = atan2(dummy.location.y, dummy.location.x) * 180. / M_PI;
				dummy.confidence = m.confidence;
				dummy.id = std::to_string(m.id);
				msgOut.objects.push_back(dummy);
			}
			pub_.publish(msgOut);
		}

	private:
		ros::Subscriber sub_;
		ros::Publisher  pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_msg_to_goal_detect");

	ros::NodeHandle n;
	ArMsgToGoalDetect arMsgToGoalDetect(n);

	ros::spin();
	return 0;
}
