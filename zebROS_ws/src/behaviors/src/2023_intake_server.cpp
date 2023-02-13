#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_actions/Intake2023Action.h>
#include <std_msgs/Float64.h>

class IntakeServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::Intake2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::Intake2023Result result_;

	double speed_;
	double fast_speed_;
	ros::Duration timeout_;

	ros::Publisher intake_pub_;

public:

	// once we switch to a service-based JointPositionController, don't do this anymore
	template <typename T>
	void make_sure_publish(ros::Publisher p, T msg) {
		ros::Rate r(200);
		for (int i = 0; i < 10; i++) {
			p.publish(msg);
			r.sleep();
		}
	}

	IntakeServer2023(std::string name) :
		as_(nh_, name, boost::bind(&IntakeServer2023::executeCB, this, _1), false),
		action_name_(name),
		intake_pub_(nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/intake_leader_controller/command", 1, true))
	{
		if (!nh_.getParam("intake_speed", speed_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find intake_speed");
			return;
		}
		if (!nh_.getParam("intake_fast_speed", fast_speed_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find intake_fast_speed");
			return;
		}
		double temp_timeout_;
		if (!nh_.getParam("intake_timeout", temp_timeout_)) {
			ROS_WARN_STREAM("2023_intake_server : could not find intake_timeout, defaulting to 10 seconds");
			temp_timeout_ = 10;
		}
		timeout_ = ros::Duration(temp_timeout_);
		const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};
		as_.start();
	}

	~IntakeServer2023(void)
	{
	}

	void executeCB(const behavior_actions::Intake2023GoalConstPtr &goal)
	{
		bool success = true;

		if (intake_pub_.getNumSubscribers() <= 0)
		{
			ROS_ERROR_STREAM("2023_intake_server : intake controller does not exist. exiting.");
			result_.timed_out = true;
			success = false;
			as_.setAborted(result_);
			return;
		}

		ROS_INFO_STREAM("2023_intake_server : calling intake controller with intake_arm_extend=true, percent_out=" << ((goal->outtake ? -1.0 : 1.0) * (goal->go_fast ? fast_speed_ : speed_)));
		std_msgs::Float64 percent_out;
		percent_out.data = (goal->outtake ? -1.0 : 1.0) * (goal->go_fast ? fast_speed_ : speed_);
		// if (!intake_client_.call(srv)) {
		// 	ROS_ERROR_STREAM("2023_intake_server : intake controller service call failed. exiting.");
		// 	result_.timed_out = false;
		// 	success = false;
		// 	as_.setAborted(result_);
		// 	return;
		// } else {
		// 	result_.timed_out = false;
		// 	success = true;
		// }
		make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it
		ros::Time start = ros::Time::now();
		ros::Rate r(100);
		while (ros::ok() && !result_.timed_out && ros::Time::now() - start <= timeout_) {
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2023_intake_server : preempted. retracting & stopping intake.");
				as_.setPreempted(result_);
				// srv.request.intake_arm_extend = false;
				// srv.request.percent_out = 0;
				// if (!intake_client_.call(srv)) {
				// 	ROS_ERROR_STREAM("2023_intake_server : failed to stop motors...");
				// 	as_.setAborted(result_);
				// 	return;
				// }
				std_msgs::Float64 percent_out;
				percent_out.data = 0.0;
				make_sure_publish(intake_pub_, percent_out);
				return;
			}
			r.sleep();
		}

		if (ros::Time::now() - start > timeout_) {
			ROS_INFO_STREAM("2023_intake_server: timed out!");
			std_msgs::Float64 percent_out;
			percent_out.data = 0.0;
			make_sure_publish(intake_pub_, percent_out);
			success = false;
			as_.setAborted(result_);
		}

		if(success)
		{
			ROS_INFO_STREAM("2023_intake_server : Succeeded");
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "intake_server_2023");

	IntakeServer2023 intake("intake_server_2023");
	ros::spin();

	return 0;
}
