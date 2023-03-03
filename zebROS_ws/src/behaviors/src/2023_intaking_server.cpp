#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/Intake2023Action.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <behavior_actions/GamePieceState2023.h>
#include <std_msgs/UInt8.h>
#include <talon_state_msgs/TalonState.h>
#include <std_msgs/Float64.h>

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
	double path_zero_timeout_;

	// shouldn't need to worry about the intake timeout since the intaker server handles that

	ros::Subscriber game_piece_sub_;
	ros::Subscriber requested_game_piece_sub_;

	size_t intake_idx;
	size_t elevator_idx;
	std::string joint_;
	double current_current_;
	double current_threshold_;
	double elev_pos_;

	double speed_;
	double fast_speed_;
	double small_speed_; // for holding cube/cone in

	actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action> path_ac_;

	ros::Subscriber talon_states_sub_;

	ros::Publisher intake_pub_;

	double time_before_reverse_;

	// output_current in talon states

public:
	template <typename T>
	void make_sure_publish(ros::Publisher p, T msg) {
		ros::Rate r(200);
		for (int i = 0; i < 10; i++) {
			p.publish(msg);
			r.sleep();
		}
	}

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

		if (elevator_idx == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "elevator_leader")
				{
					elevator_idx = i;
					break;
				}
			}
		}
		if (!(elevator_idx == std::numeric_limits<size_t>::max()))
		{
			elev_pos_ = talon_state.position[elevator_idx];
		}
		else {
			ROS_ERROR_STREAM("2023_intaking_server : Can not find talon with name = elevator_leader");
		}
	}

	IntakingServer2023(std::string name) :
		as_(nh_, name, boost::bind(&IntakingServer2023::executeCB, this, _1), false),
		action_name_(name),
		game_piece_sub_(nh_.subscribe("/game_piece/game_piece_state", 1, &IntakingServer2023::gamePieceStateCallback, this)),
		requested_game_piece_sub_(nh_.subscribe("/game_piece/requested_game_piece", 1, &IntakingServer2023::requestedPieceCallback, this)),
		intake_pub_(nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/intake_leader_controller/command", 1, true)),
		path_ac_("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true)
	{
		elevator_idx = std::numeric_limits<size_t>::max();
		intake_idx = std::numeric_limits<size_t>::max();
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &IntakingServer2023::talonStateCallback, this);
		game_piece_state_.game_piece = game_piece_state_.NONE; // default to no game piece
		if (!nh_.getParam("time_before_reverse", time_before_reverse_)) {
			ROS_WARN_STREAM("2023_intaking_server : could not find time_before_reverse, defaulting to 1 seconds");
			time_before_reverse_ = 1;
		}
		if (!nh_.getParam("cube_time", cube_time_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find cube_time");
			return;
		}
		if (!nh_.getParam("path_zero_timeout", path_zero_timeout_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find path_zero_timeout");
			return;
		}
		if (!nh_.getParam("cone_time", cone_time_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find cone_time");
			return;
		}
		if (!nh_.getParam("server_timeout", server_timeout_)) {
			ROS_WARN_STREAM("2023_intaking_server : could not find server_timeout, defaulting to 20 seconds");
			server_timeout_ = 20;
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
		if (!nh_.getParam("intake_small_speed", small_speed_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find intake_small_speed");
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
		if (!path_ac_.waitForServer(ros::Duration(server_timeout_))) {
			ROS_ERROR_STREAM("2023_intaking_server : timed out connecting to fourbar elevator path server, aborting");
			result_.timed_out = true;
			as_.setAborted(result_);
			return;
		}

		ros::Rate r(10);

		bool elevator_is_ok = elev_pos_ < 0.2;
		ROS_INFO_STREAM("pos is  " << elev_pos_ << (elevator_is_ok ? " ok" : " no"));

		if (goal->unflip_outtake || !elevator_is_ok) {
			behavior_actions::FourbarElevatorPath2023Goal pathGoal;
			pathGoal.path = "unflip_fourbar";
			pathGoal.reverse = false;

			feedback_.status = feedback_.PATHER;
			as_.publishFeedback(feedback_);

			path_ac_.sendGoal(pathGoal);

			std_msgs::Float64 percent_out;
			percent_out.data = 0.0;
			make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

			while (!path_ac_.getState().isDone()) {
				ros::spinOnce();
				ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for unflipping...");
				if (as_.isPreemptRequested() || !ros::ok()) {
					ROS_INFO_STREAM("2023_intaking_server : preempted.");

					ros::Time start = ros::Time::now();
					while (!path_ac_.getState().isDone() && ros::ok() && (ros::Time::now() - start) < ros::Duration(path_zero_timeout_)) {
						ros::spinOnce();
						ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : unflipping");
						r.sleep();
					}
					as_.setPreempted(result_);
					return;
				}
				r.sleep();
			}
			if (goal->unflip_outtake) {
				return;
			}
		}

		behavior_actions::FourbarElevatorPath2023Goal pathGoal;
		pathGoal.path = (goal->piece == goal->VERTICAL_CONE) ? "intake_vertical_cone" : "intake_cone_cube";
		pathGoal.reverse = false;

		feedback_.status = feedback_.PATHER;
		as_.publishFeedback(feedback_);

		path_ac_.sendGoal(pathGoal);

		std_msgs::Float64 percent_out;
		percent_out.data = speed_;
		make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

		ros::Duration(0.25).sleep();

		while (!path_ac_.getState().isDone()) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for forward pather...");
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2023_intaking_server : preempted.");

				path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());

				percent_out.data = 0.0;
				make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

				ROS_INFO_STREAM("2023_intaking_server : zeroing");
				pathGoal.path = "zero";
				pathGoal.reverse = false;

				feedback_.status = feedback_.PATHER;
				as_.publishFeedback(feedback_);

				path_ac_.sendGoal(pathGoal);

				ros::Time start = ros::Time::now();
				while (!path_ac_.getState().isDone() && ros::ok() && (ros::Time::now() - start) < ros::Duration(path_zero_timeout_)) {
					ros::spinOnce();
					ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : zeroing");
					r.sleep();
				}
				as_.setPreempted(result_);
				return;
			}
			r.sleep();
		}

		feedback_.status = feedback_.INTAKE;
		as_.publishFeedback(feedback_);

		ROS_INFO_STREAM("current: " << current_current_ << " threshold: " << current_threshold_);
		
		while (/*game_piece_state_.game_piece == behavior_actions::GamePieceState2023::NONE && */current_current_ < current_threshold_) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for a game piece... current = " << current_current_);
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2023_intaking_server : preempted.");

				path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());

				percent_out.data = 0.0;
				make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

				ROS_INFO_STREAM("2023_intaking_server : zeroing");
				pathGoal.path = "zero";
				pathGoal.reverse = false;

				feedback_.status = feedback_.PATHER;
				as_.publishFeedback(feedback_);

				path_ac_.sendGoal(pathGoal);

				ros::Time start = ros::Time::now();
				while (!path_ac_.getState().isDone() && ros::ok() && (ros::Time::now() - start) < ros::Duration(path_zero_timeout_)) {
					ros::spinOnce();
					ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : zeroing");
					r.sleep();
				}
				as_.setPreempted(result_);
				return;
			}
			r.sleep();
		}

		ROS_INFO_STREAM("current: " << current_current_ << " threshold: " << current_threshold_);
		bool current_exceeded_ = current_current_ >= current_threshold_;

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

		percent_out.data = small_speed_;
		make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

		ros::Duration(time_before_reverse_).sleep();
		
		// move back to holding position

		pathGoal.path = "flip_fourbar_after_intaking";
		pathGoal.reverse = false;

		feedback_.status = feedback_.PATHER;
		as_.publishFeedback(feedback_);

		path_ac_.sendGoal(pathGoal);

		ros::Time start = ros::Time::now();
		while (!path_ac_.getState().isDone()  && (ros::Time::now() - start) < ros::Duration(path_zero_timeout_)) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : bringing game piece back... timing out after " << (ros::Time::now() - start).toSec() << " seconds");
			r.sleep();
		}

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
