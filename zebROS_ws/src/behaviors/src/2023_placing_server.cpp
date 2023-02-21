#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Intake2023Action.h>
#include <behavior_actions/Fourber2023Action.h>
#include <behavior_actions/Elevater2023Action.h>
#include <behavior_actions/Placing2023Action.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <path_follower_msgs/holdPositionAction.h>

class PlacingServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::Placing2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::Placing2023Result result_;

	double latest_game_piece_position_; // left = -(1/2 width of intake), right = (1/2 width of intake). this is the center of the game piece
	uint8_t latest_game_piece_; // corresponds to the enums in various action files
	ros::Time latest_game_piece_time_;
	ros::Time latest_game_piece_position_time_;

	ros::Subscriber game_piece_sub_;
	ros::Subscriber game_piece_position_sub_;

	actionlib::SimpleActionClient<behavior_actions::Fourber2023Action> ac_fourber_;
	actionlib::SimpleActionClient<behavior_actions::Intake2023Action> ac_intake_;
	actionlib::SimpleActionClient<behavior_actions::Elevater2023Action> ac_elevater_;
	actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction> ac_hold_position_;

	double server_timeout_;
	double game_piece_timeout_;
	double outtake_time_;

public:

	PlacingServer2023(std::string name) :
		as_(nh_, name, boost::bind(&PlacingServer2023::executeCB, this, _1), false),
		action_name_(name),
		ac_fourber_("/fourber/fourber_server_2023", true),
   		ac_intake_("/intake/intake_server_2023", true),
		ac_elevater_("/elevater/elevater_server_2023", true),
		ac_hold_position_("/hold_position/hold_position_server", true)
	{
		if (!nh_.getParam("server_timeout", server_timeout_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find server_timeout, defaulting to 10 seconds");
			server_timeout_ = 10;
		}
		if (!nh_.getParam("game_piece_timeout", game_piece_timeout_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find game_piece_timeout, defaulting to 1 second");
			game_piece_timeout_ = 1;
		}
		if (!nh_.getParam("outtake_time", outtake_time_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find outtake_time, defaulting to 1 second");
			outtake_time_ = 1;
		}
		game_piece_sub_ = nh_.subscribe("/game_piece", 1, &PlacingServer2023::gamePieceCallback, this);
		game_piece_position_sub_ = nh_.subscribe("/game_piece_position", 1, &PlacingServer2023::gamePiecePositionCallback, this);
		as_.start();
	}

	~PlacingServer2023(void)
	{
	}

	template<class C, class S>
	bool waitForResultAndCheckForPreempt(const ros::Duration & timeout, actionlib::SimpleActionClient<C> & ac, actionlib::SimpleActionServer<S> & as, bool preempt_at_timeout = false)
	{
		bool negative_timeout = false;
		if (timeout < ros::Duration(0, 0)) {
			ROS_WARN("waitForResultAndCheckForPreempt : Negative timeout, waiting forever");
			negative_timeout = true;
		}

		ros::Time timeout_time = ros::Time::now() + timeout;

		ros::Rate r(10);

		while (ros::ok() && !as.isPreemptRequested()) {
			ros::spinOnce();
			// Determine how long we should wait
			ros::Duration time_left = timeout_time - ros::Time::now();

			// Check if we're past the timeout time
			if (timeout > ros::Duration(0, 0) && time_left <= ros::Duration(0, 0) && !negative_timeout) {
				if (preempt_at_timeout) {
					ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				break;
			}

			if (ac.getState().isDone()) {
				break;
			}
			r.sleep();
		}

		return ac.getState().isDone();
	}

	void gamePieceCallback(const std_msgs::UInt8 msg) {
		latest_game_piece_ = msg.data;
		latest_game_piece_time_ = ros::Time::now();
	}

	void gamePiecePositionCallback(const std_msgs::Float64 msg) {
		latest_game_piece_position_ = msg.data;
		latest_game_piece_position_time_ = ros::Time::now();
	}

	using pl = behavior_actions::Placing2023Goal;
	using fb = behavior_actions::Fourber2023Goal;
	behavior_actions::Fourber2023Goal::_mode_type nodeToMode(behavior_actions::Placing2023Goal::_node_type node) {
		switch (node) {
			case pl::HIGH:
				return fb::HIGH_NODE;
			case pl::MID:
				return fb::MIDDLE_NODE;
			case pl::HYBRID:
				return fb::LOW_NODE;
			default:
				ROS_ERROR_STREAM("2023_placing_server : invalid node to convert to mode");
				return 255;
		}
	}

	void executeCB(const behavior_actions::Placing2023GoalConstPtr &goal)
	{
		if (!ac_fourber_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_placing_server : timed out connecting to fourber server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		if (!ac_elevater_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_placing_server : timed out connecting to elevater server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		if (!ac_intake_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_placing_server : timed out connecting to intake server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		if (ros::Time::now() - latest_game_piece_time_ > ros::Duration(game_piece_timeout_)) {
			ROS_ERROR_STREAM("2023_placing_server : game piece data too old, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		if (ros::Time::now() - latest_game_piece_position_time_ > ros::Duration(game_piece_timeout_)) {
			ROS_ERROR_STREAM("2023_placing_server : game piece position data too old, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		if (goal->align_intake) {
			path_follower_msgs::holdPositionGoal hold_position_goal_;
			hold_position_goal_.pose.orientation.x = 0.0;
			hold_position_goal_.pose.orientation.y = 0.0;
			hold_position_goal_.pose.orientation.z = 0.0;
			hold_position_goal_.pose.orientation.w = 1.0;

			hold_position_goal_.pose.position.x = 0.0;
			hold_position_goal_.pose.position.y = -latest_game_piece_position_;
			hold_position_goal_.pose.position.z = 0.0;

			hold_position_goal_.isAbsoluteCoord = false;

			ROS_INFO_STREAM("2023_placing_server : holding position, y = " << hold_position_goal_.pose.position.y);

			ac_hold_position_.sendGoal(hold_position_goal_);

			if (!waitForResultAndCheckForPreempt(ros::Duration(-1), ac_hold_position_, as_)) {
				ROS_INFO_STREAM("2023_placing_server : hold position server timed out, aborting");
				result_.timed_out = true;
				as_.setAborted(result_);
				ac_hold_position_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}
		}

		behavior_actions::Fourber2023Goal fourber_goal;
		fourber_goal.piece = latest_game_piece_;
		fourber_goal.mode = nodeToMode(goal->node); // please don't change the actionlib files
		ac_fourber_.sendGoal(fourber_goal);

		if (!(waitForResultAndCheckForPreempt(ros::Duration(-1), ac_fourber_, as_) && ac_fourber_.getState() == ac_fourber_.getState().SUCCEEDED)) {
			ROS_INFO_STREAM("2023_placing_server : fourber timed out, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			ac_fourber_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			ac_elevater_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			return;
		}

		behavior_actions::Elevater2023Goal elevater_goal;
		elevater_goal.piece = latest_game_piece_;
		elevater_goal.mode = nodeToMode(goal->node); // please don't change the actionlib files
		ac_elevater_.sendGoal(elevater_goal);

		if (!(waitForResultAndCheckForPreempt(ros::Duration(-1), ac_elevater_, as_) && ac_elevater_.getState() == ac_elevater_.getState().SUCCEEDED)) {
			ROS_INFO_STREAM("2023_placing_server : elevater timed out, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			ac_fourber_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			ac_elevater_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			return;
		}

		behavior_actions::Intake2023Goal intake_goal_;
		intake_goal_.go_fast = false; // TODO change this? idk if we still want to have multiple speeds
		intake_goal_.outtake = true; // don't change this
		ac_intake_.sendGoal(intake_goal_);

		if (!(waitForResultAndCheckForPreempt(ros::Duration(outtake_time_), ac_intake_, as_, true) && ac_intake_.getState() == ac_intake_.getState().SUCCEEDED)) {
			if (ac_intake_.getState() == actionlib::SimpleClientGoalState::ACTIVE || ac_intake_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO_STREAM("2023_placing_server : stopping outtaking");
				ac_intake_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			} else {
				ROS_ERROR_STREAM("2023_placing_server : error with intake server! aborting!");
				result_.timed_out = true;
				as_.setAborted(result_);
				ac_intake_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}
		}

		as_.setSucceeded(result_);
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "placing_server_2023");

	PlacingServer2023 placer("placing_server_2023");
	ros::spin();

	return 0;
}
