#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <behavior_actions/Placing2023Action.h>
#include <behavior_actions/GamePieceState2023.h>
#include <path_follower_msgs/holdPositionAction.h>

class PlacingServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::Placing2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::Placing2023Result result_;

	behavior_actions::GamePieceState2023 game_piece_state_;
	ros::Time latest_game_piece_time_;

	ros::Subscriber game_piece_sub_;

	actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action> path_ac_;
	actionlib::SimpleActionClient<behavior_actions::Intaking2023Action> ac_intaking_;
	actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction> ac_hold_position_;

	double server_timeout_;
	double game_piece_timeout_;
	double outtake_time_;
	double time_before_reverse_;

public:

	PlacingServer2023(std::string name) :
		as_(nh_, name, boost::bind(&PlacingServer2023::executeCB, this, _1), false),
		action_name_(name),
   		ac_intaking_("/intaking/intaking_server_2023", true),
		path_ac_("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true),
		ac_hold_position_("/hold_position/hold_position_server", true)
	{
		if (!nh_.getParam("time_before_reverse", time_before_reverse_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find time_before_reverse, defaulting to 1 seconds");
			time_before_reverse_ = 1;
		}
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
		game_piece_sub_ = nh_.subscribe("/game_piece/game_piece_state", 1, &PlacingServer2023::gamePieceCallback, this);
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

	void gamePieceCallback(const behavior_actions::GamePieceState2023ConstPtr &msg) {
		game_piece_state_ = *msg;
		latest_game_piece_time_ = ros::Time::now();
	}

	std::string pathForGamePiece(uint8_t game_piece, uint8_t location) {
		std::string location_str;
		if (location == behavior_actions::Placing2023Goal::HYBRID) {
			location_str = "hybrid";
		}
		else if (location == behavior_actions::Placing2023Goal::MID) {
			location_str = "mid";
		}
		else {
			location_str = "high";
		}

		std::string piece_str;
		if (game_piece == behavior_actions::Placing2023Goal::CUBE) {
			piece_str = "cube";
		}
		else {
			piece_str = "cone";
		}

		return location_str + "_" + piece_str;
	}

	void executeCB(const behavior_actions::Placing2023GoalConstPtr &goal)
	{
		result_.success = true; // default to true, set to false if fails
		if (!path_ac_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_placing_server : timed out connecting to fourbar elevator path server, aborting");
			result_.success = false;
			as_.setAborted(result_);
			return;
		}

		if (!ac_intaking_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_placing_server : timed out connecting to intaking server, aborting");
			result_.success = false;
			as_.setAborted(result_);
			return;
		}

		uint8_t game_piece;

		if (!goal->override_game_piece) {
			game_piece = game_piece_state_.game_piece;
			ROS_INFO_STREAM("2023_placing_server : detected game piece = " << std::to_string(game_piece_state_.game_piece));
		}
		else {
			game_piece = goal->piece;
			ROS_INFO_STREAM("2023_placing_server : game piece override. detected = " << std::to_string(game_piece_state_.game_piece) << ", requested = " << std::to_string(goal->piece) << ". using requested value");
		}

		behavior_actions::FourbarElevatorPath2023Goal pathGoal;

		pathGoal.path = pathForGamePiece(game_piece, goal->node);
		pathGoal.reverse = false;
		if (goal->from_Trex) {
			pathGoal.path += "_auto";
		}

		if (goal->step == goal->MOVE) {
			ROS_INFO_STREAM("2023_placing_server : moving to placing position");

			path_ac_.sendGoal(pathGoal);

			if (!(waitForResultAndCheckForPreempt(ros::Duration(-1), path_ac_, as_) && path_ac_.getState() == path_ac_.getState().SUCCEEDED)) {
				ROS_INFO_STREAM("2023_placing_server : pather failed, aborting");
				result_.success = false;
				as_.setAborted(result_);
				path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}

			result_.success = true;
			as_.setSucceeded(result_);
			return;
		}

		if (!(goal->override_game_piece)) {
			if (ros::Time::now() - latest_game_piece_time_ > ros::Duration(game_piece_timeout_)) {
				ROS_ERROR_STREAM("2023_placing_server : game piece data too old, aborting");
				result_.success = false;
				as_.setAborted(result_);
				return;
			}
		}

		if (goal->align_intake) {
			if (ros::Time::now() - latest_game_piece_time_ > ros::Duration(game_piece_timeout_)) {
				ROS_ERROR_STREAM("2023_placing_server : game piece position data too old, aborting");
				result_.success = false;
				as_.setAborted(result_);
				return;
			}
			path_follower_msgs::holdPositionGoal hold_position_goal_;
			hold_position_goal_.pose.orientation.x = 0.0;
			hold_position_goal_.pose.orientation.y = 0.0;
			hold_position_goal_.pose.orientation.z = 0.0;
			hold_position_goal_.pose.orientation.w = 1.0;

			hold_position_goal_.pose.position.x = 0.0;
			hold_position_goal_.pose.position.y = -game_piece_state_.offset_from_center;
			hold_position_goal_.pose.position.z = 0.0;

			hold_position_goal_.isAbsoluteCoord = false;

			ROS_INFO_STREAM("2023_placing_server : holding position, y = " << hold_position_goal_.pose.position.y);

			ac_hold_position_.sendGoal(hold_position_goal_);

			if (!waitForResultAndCheckForPreempt(ros::Duration(-1), ac_hold_position_, as_)) {
				ROS_INFO_STREAM("2023_placing_server : hold position server timed out, aborting");
				result_.success = false;
				as_.setAborted(result_);
				ac_hold_position_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}
		}

		behavior_actions::Intaking2023Goal intaking_goal_;
		intaking_goal_.outtake = true; // don't change this
		ac_intaking_.sendGoal(intaking_goal_);

		if (!(waitForResultAndCheckForPreempt(ros::Duration(outtake_time_), ac_intaking_, as_, true) && ac_intaking_.getState() == ac_intaking_.getState().SUCCEEDED)) {
			if (ac_intaking_.getState() == actionlib::SimpleClientGoalState::ACTIVE || ac_intaking_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO_STREAM("2023_placing_server : stopping outtaking");
				ac_intaking_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			} else {
				ROS_ERROR_STREAM("2023_placing_server : error with intake server! aborting!");
				result_.success = false;
				as_.setAborted(result_);
				ac_intaking_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}
		}

		ros::Duration(time_before_reverse_).sleep();

		ROS_INFO_STREAM("2023_placing_server : reversing path!");

		pathGoal.path = pathForGamePiece(game_piece, goal->node) + "_reverse";
		pathGoal.reverse = false;
		
		path_ac_.sendGoal(pathGoal);

		if (!(waitForResultAndCheckForPreempt(ros::Duration(-1), path_ac_, as_) && path_ac_.getState() == path_ac_.getState().SUCCEEDED)) {
			ROS_INFO_STREAM("2023_placing_server : pather timed out, aborting");
			result_.success = false;
			as_.setAborted(result_);
			path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			return;
		}

		result_.success = true;
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
