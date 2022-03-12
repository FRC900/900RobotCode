#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_actions/Intake2022Action.h>
#include <controllers_2022_msgs/Intake.h>

class IntakeServer2022
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::Intake2022Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::Intake2022Result result_;

	double speed_;
	double fast_speed_;
	double server_timeout_;

	ros::ServiceClient intake_client_;

public:

	IntakeServer2022(std::string name) :
		as_(nh_, name, boost::bind(&IntakeServer2022::executeCB, this, _1), false),
		action_name_(name)
	{
		if (!nh_.getParam("intake_speed", speed_))
		{
			ROS_ERROR_STREAM("2022_intake_server : could not find intake_speed");
			return;
		}
		if (!nh_.getParam("intake_fast_speed", fast_speed_))
		{
			ROS_ERROR_STREAM("2022_intake_server : could not find intake_fast_speed");
			return;
		}
		if (!nh_.getParam("intake_server_timeout", server_timeout_)) {
			ROS_WARN_STREAM("2022_intake_server : could not find intake_server_timeout, defaulting to 10 seconds");
			server_timeout_ = 10;
		}
		intake_client_ = nh_.serviceClient<controllers_2022_msgs::Intake>("/frcrobot_jetson/intake_controller/command");
		as_.start();
	}

	~IntakeServer2022(void)
	{
	}

	void executeCB(const behavior_actions::Intake2022GoalConstPtr &goal)
	{
		bool success = true;

		if(!intake_client_.waitForExistence(ros::Duration(server_timeout_)))
		{
			ROS_ERROR_STREAM("2022_intake_server : intake controller service does not exist. exiting.");
			result_.timed_out = true;
			success = false;
			as_.setAborted(result_);
			return;
		}

		ROS_INFO_STREAM("2022_intake_server : calling intake controller with intake_arm_extend=true, percent_out=" << ((goal->reverse ? -1.0 : 1.0) * (goal->go_fast ? fast_speed_ : speed_)));
		controllers_2022_msgs::Intake srv;
		srv.request.intake_arm_extend = true;
		srv.request.percent_out = (goal->reverse ? -1.0 : 1.0) * (goal->go_fast ? fast_speed_ : speed_);
		if (!intake_client_.call(srv)) {
			ROS_ERROR_STREAM("2022_intake_server : intake controller service call failed. exiting.");
			result_.timed_out = false;
			success = false;
			as_.setAborted(result_);
			return;
		} else {
			result_.timed_out = false;
			success = true;
		}
		while (ros::ok() && !result_.timed_out) {
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2022_intake_server : preempted. retracting & stopping intake.");
				as_.setPreempted(result_);
				srv.request.intake_arm_extend = false;
				srv.request.percent_out = 0;
				if (!intake_client_.call(srv)) {
					ROS_ERROR_STREAM("2022_intake_server : failed to stop motors...");
					as_.setAborted(result_);
					return;
				}
				return;
			}
		}

		if(success)
		{
			ROS_INFO_STREAM("2022_intake_server : Succeeded");
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "intake_server_2022");

	IntakeServer2022 intake("intake_server_2022");
	ros::spin();

	return 0;
}
