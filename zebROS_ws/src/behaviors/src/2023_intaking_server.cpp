#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/Intake2023Action.h>
#include <behavior_actions/Elevater2023Action.h>
#include <behavior_actions/Fourber2023Action.h>
#include <behavior_actions/GamePieceState2023.h>
#include <std_msgs/UInt8.h>
#include <talon_state_msgs/TalonState.h>

class IntakingServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::Intaking2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::Intaking2023Result result_;
	behavior_actions::Intaking2023Feedback feedback_;

	behavior_actions::GamePieceState2023 game_piece_state_;
	uint8_t requested_game_piece_;

	double server_timeout_;
	double cube_time_; // time to keep intaking after we see a cube
	double cone_time_; // time to keep intaking after we see a cone

	// shouldn't need to worry about the intake timeout since the intaker server handles that

	ros::Subscriber game_piece_sub_;
	ros::Subscriber requested_game_piece_sub_;

	size_t intake_idx;
	std::string joint_;
	double current_current_;
	double current_threshold_;

	actionlib::SimpleActionClient<behavior_actions::Intake2023Action> intake_ac_;
	actionlib::SimpleActionClient<behavior_actions::Elevater2023Action> elevater_ac_;
	actionlib::SimpleActionClient<behavior_actions::Fourber2023Action> fourber_ac_;

	ros::Subscriber talon_states_sub_;

	// output_current in talon states

public:
	void gamePieceStateCallback(const behavior_actions::GamePieceState2023 &msg) {
		game_piece_state_ = msg;
	}

	void requestedPieceCallback(const std_msgs::UInt8 &msg) {
		requested_game_piece_ = msg.data;
	}

	void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
	{
		// fourbar_master_idx == max of size_t at the start
		if (intake_idx == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == joint_)
				{
					intake_idx = i;
					break;
				}
			}
		}
		if (!(intake_idx == std::numeric_limits<size_t>::max()))
		{
			current_current_ = talon_state.output_current[intake_idx];
		}
		else {
			ROS_ERROR_STREAM("2023_intaking_server : Can not find talon with name = " << joint_);
		}
	}

	IntakingServer2023(std::string name) :
		as_(nh_, name, boost::bind(&IntakingServer2023::executeCB, this, _1), false),
		action_name_(name),
		game_piece_sub_(nh_.subscribe("/game_piece/game_piece_state", 1, &IntakingServer2023::gamePieceStateCallback, this)),
		requested_game_piece_sub_(nh_.subscribe("/game_piece/requested_game_piece", 1, &IntakingServer2023::requestedPieceCallback, this)),
		intake_ac_("/intake/intake_server_2023", true),
		elevater_ac_("/elevater/elevater_server_2023", true),
		fourber_ac_("/fourber/fourber_server_2023", true)
	{
		intake_idx = std::numeric_limits<size_t>::max();
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &IntakingServer2023::talonStateCallback, this);
		game_piece_state_.game_piece = game_piece_state_.NONE; // default to no game piece
		if (!nh_.getParam("cube_time", cube_time_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find cube_time");
			return;
		}
		if (!nh_.getParam("cone_time", cone_time_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find cone_time");
			return;
		}
		if (!nh_.getParam("server_timeout", server_timeout_)) {
			ROS_WARN_STREAM("2023_intaking_server : could not find server_timeout, defaulting to 10 seconds");
			server_timeout_ = 10;
		}
		if (!nh_.getParam("joint", joint_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find joint");
			return;
		}
		if (!nh_.getParam("current_threshold", current_threshold_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find current_threshold");
			return;
		}
		const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};
		as_.start();
	}

	~IntakingServer2023(void)
	{
	}

	void executeCB(const behavior_actions::Intaking2023GoalConstPtr &goal)
	{
		if (!elevater_ac_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_intaking_server : timed out connecting to elevater server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		if (!fourber_ac_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_intaking_server : timed out connecting to fourber server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		if (!intake_ac_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_intaking_server : timed out connecting to intake server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		behavior_actions::Fourber2023Goal fourberGoal;
		fourberGoal.piece = requested_game_piece_;
		fourberGoal.mode = fourberGoal.INTAKE;
		fourberGoal.safety_positions = {};

		feedback_.status = feedback_.FOURBER;
		as_.publishFeedback(feedback_);

		fourber_ac_.sendGoal(fourberGoal);

		ros::Rate r(10);
		while (!fourber_ac_.getState().isDone()) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for fourber...");
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2023_intaking_server : preempted.");
				as_.setPreempted(result_);
				fourber_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}
			r.sleep();
		}

		ROS_INFO_STREAM("2023_intaking_server : fourber moved");

		behavior_actions::Elevater2023Goal elevaterGoal;
		elevaterGoal.piece = requested_game_piece_;
		elevaterGoal.mode = elevaterGoal.INTAKE;

		feedback_.status = feedback_.ELEVATER;
		as_.publishFeedback(feedback_);

		elevater_ac_.sendGoal(elevaterGoal);

		while (!elevater_ac_.getState().isDone()) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for elevater...");
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2023_intaking_server : preempted.");
				as_.setPreempted(result_);
				elevater_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}
			r.sleep();
		}

		behavior_actions::Intake2023Goal intakeGoal;
		intakeGoal.outtake = false;
		intakeGoal.go_fast = false;

		feedback_.status = feedback_.INTAKE;
		as_.publishFeedback(feedback_);

		intake_ac_.sendGoal(intakeGoal);

		ROS_INFO_STREAM("current: " << current_current_ << " threshold: " << current_threshold_ << " state: " << intake_ac_.getState().state_ << " aka " << intake_ac_.getState().getText() << " isdone? " << intake_ac_.getState().isDone());
		
		while (/*game_piece_state_.game_piece == behavior_actions::GamePieceState2023::NONE && */!(intake_ac_.getState().isDone()) && current_current_ < current_threshold_) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for a game piece... current = " << current_current_);
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2023_intaking_server : preempted.");
				as_.setPreempted(result_);
				intake_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}
			r.sleep();
		}

		ROS_INFO_STREAM("current: " << current_current_ << " threshold: " << current_threshold_ << " state: " << intake_ac_.getState().state_ << " aka " << intake_ac_.getState().getText() << ". gp " << std::to_string(game_piece_state_.game_piece) << " isdone? " << intake_ac_.getState().isDone());
		bool current_exceeded_ = current_current_ >= current_threshold_;

		if (intake_ac_.getState() != intake_ac_.getState().SUCCEEDED && intake_ac_.getState() != intake_ac_.getState().ACTIVE) {
			ROS_ERROR_STREAM("2023_intaking_server : intake server failed, aborting!");
			intake_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			as_.setAborted(result_);
			return;
		}

		if (game_piece_state_.game_piece == behavior_actions::GamePieceState2023::CUBE || current_exceeded_) {
			ROS_INFO_STREAM("2023_intaking_server : " << (current_exceeded_ ? "current exceeded" : "cube detected") << ", waiting " << cube_time_ << " seconds");
			ros::Time start = ros::Time::now();
			while (ros::Time::now() - start < ros::Duration(cube_time_)) {
				ros::spinOnce();
			}
		}
		else {
			ROS_INFO_STREAM("2023_intaking_server : cone detected, waiting " << cone_time_ << " seconds");
			ros::Time start = ros::Time::now();
			while (ros::Time::now() - start < ros::Duration(cone_time_)) {
				ros::spinOnce();
			}
		}

		intake_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
		feedback_.status = feedback_.DONE;
		as_.publishFeedback(feedback_);
		
		ROS_INFO_STREAM("2023_intaking_server : Succeeded");
		// set the action state to succeeded
		as_.setSucceeded(result_);
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "intaking_server_2023");

	IntakingServer2023 intaking("intaking_server_2023");
	ros::spin();

	return 0;
}