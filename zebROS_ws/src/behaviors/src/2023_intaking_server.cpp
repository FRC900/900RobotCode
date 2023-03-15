#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <behavior_actions/GamePieceState2023.h>
#include <std_msgs/UInt8.h>
#include <talon_state_msgs/TalonState.h>
#include <std_msgs/Float64.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

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

	bool dynamic_reconfigure_;

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
	double cone_outtake_speed_;
	double cube_outtake_speed_;
	double small_speed_; // for holding cube/cone in

	double medium_speed_;
	double medium_time_;

	double minimum_current_time_;
	double minimum_fourbar_extension_;

	double fourbar_cur_position_;
	size_t fourbar_idx;

	actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action> path_ac_;

	ros::Subscriber talon_states_sub_;

	ros::Publisher intake_pub_;

	double time_before_reverse_;

	ros::NodeHandle nh_params_;

	ddynamic_reconfigure::DDynamicReconfigure ddr_;

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

		if (fourbar_idx == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "four_bar")
				{
					fourbar_idx = i;
					break;
				}
			}
		}
		if (!(fourbar_idx == std::numeric_limits<size_t>::max()))
		{
			fourbar_cur_position_ = talon_state.position[fourbar_idx];
		}
		else {
			ROS_ERROR_STREAM("2023_intaking_server : Can not find talon with name = four_bar");
		}
	}

	IntakingServer2023(std::string name) :
		as_(nh_, name, boost::bind(&IntakingServer2023::executeCB, this, _1), false),
		action_name_(name),
		game_piece_sub_(nh_.subscribe("/game_piece/game_piece_state", 1, &IntakingServer2023::gamePieceStateCallback, this)),
		requested_game_piece_sub_(nh_.subscribe("/game_piece/requested_game_piece", 1, &IntakingServer2023::requestedPieceCallback, this)),
		path_ac_("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true),
		intake_pub_(nh_.advertise<std_msgs::Float64>("/frcrobot_jetson/intake_leader_controller/command", 1, true)),
		nh_params_(nh_, "intaking_server_2023"),
		ddr_(nh_params_)
	{
		elevator_idx = std::numeric_limits<size_t>::max();
		intake_idx = std::numeric_limits<size_t>::max();
		fourbar_idx = std::numeric_limits<size_t>::max();
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &IntakingServer2023::talonStateCallback, this);
		game_piece_state_.game_piece = game_piece_state_.NONE; // default to no game piece

		if (!nh_.getParam("dynamic_reconfigure", dynamic_reconfigure_)) {
			ROS_WARN_STREAM("2023_intaking_server : could not find dynamic_reconfigure, defaulting to false");
			dynamic_reconfigure_ = false;
		}

		if (!nh_.getParam("time_before_reverse", time_before_reverse_)) {
			ROS_WARN_STREAM("2023_intaking_server : could not find time_before_reverse, defaulting to 0 seconds");
			time_before_reverse_ = 0;
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

		if (!nh_.getParam("minimum_fourbar_extension", minimum_fourbar_extension_))
		{
			ROS_ERROR_STREAM("2023_intaking_server : could not find minimum_fourbar_extension");
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
		// not dynamic reconfigurable, doesn't really make sense to do that

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

		if (!nh_.getParam("minimum_current_time", minimum_current_time_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find minimum_current_time");
			return;
		}

		if (!nh_.getParam("cone_outtake_speed", cone_outtake_speed_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find cone_outtake_speed");
			return;
		}

		if (!nh_.getParam("cube_outtake_speed", cube_outtake_speed_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find cube_outtake_speed");
			return;
		}
		
		if (!nh_.getParam("intake_small_speed", small_speed_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find intake_small_speed");
			return;
		}

		if (!nh_.getParam("intake_medium_speed", medium_speed_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find intake_medium_speed");
			return;
		}

		if (!nh_.getParam("intake_medium_time", medium_time_))
		{
			ROS_ERROR_STREAM("2023_intake_server : could not find intake_medium_time");
			return;
		}

		if (dynamic_reconfigure_) {
			ddr_.registerVariable<double>("time_before_reverse", &time_before_reverse_, "Time before reversing", 0, 10);
			ddr_.registerVariable<double>("cube_time", &cube_time_, "Time for intaking cubes (currently used for both because no terabees)", 0, 5);
			ddr_.registerVariable<double>("path_zero_timeout", &path_zero_timeout_, "Timeout before preempting zeroing path after we are preempted", 0, 10);
			ddr_.registerVariable<double>("cone_time", &cone_time_, "Time for intaking cones (currently used for neither because no terabees)", 0, 5);
			ddr_.registerVariable<double>("server_timeout", &server_timeout_, "Server timeout", 0, 30);
			ddr_.registerVariable<double>("current_threshold", &current_threshold_, "Current threshold before stopping intake, some weird unit", 0, 300);
			ddr_.registerVariable<double>("intake_speed", &speed_, "Intake speed (percent output)", 0, 1);
			ddr_.registerVariable<double>("cone_outtake_speed", &cone_outtake_speed_, "Cone outtake speed (percent output)", 0, 1);
			ddr_.registerVariable<double>("cube_outtake_speed", &cube_outtake_speed_, "Cube outtake speed (percent output)", 0, 1);
			ddr_.registerVariable<double>("intake_small_speed", &small_speed_, "Intake small speed for holding game piece in (percent output)", 0, 1);
			ddr_.registerVariable<double>("intake_medium_speed", &medium_speed_, "Intake medium speed for holding game piece in (percent output)", 0, 1);
			ddr_.registerVariable<double>("intake_medium_time", &medium_time_, "Time before switching from medium to small", 0, 10);
			ddr_.registerVariable<double>("minimum_current_time", &minimum_current_time_, "Time current spiking before retracting intake", 0, 1);
			ddr_.registerVariable<double>("minimum_fourbar_extension", &minimum_fourbar_extension_, "Amount four bar must be extended before checking for current", 0, 1);
		}

		ddr_.publishServicesTopics();

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

		path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now()); // cancel all goals before sending the new one
		// (we don't want anything else controlling the elevator/fourbar when intaking)

		ros::Rate r(50);

		bool elevator_is_ok = elev_pos_ < 0.2;
		ROS_INFO_STREAM("pos is  " << elev_pos_ << (elevator_is_ok ? " ok" : " no"));

		if (goal->outtake != goal->INTAKE) {
			ROS_INFO_STREAM("2023_intaking_server : outtaking");
			std_msgs::Float64 percent_out;
			percent_out.data = goal->outtake == goal->OUTTAKE_CONE ? -cone_outtake_speed_ : -cube_outtake_speed_;
			make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

			while (true) {
				ros::spinOnce();
				ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : outtaking until preempted...");
				if (as_.isPreemptRequested() || !ros::ok()) {
					ROS_INFO_STREAM("2023_intaking_server : preempted.");

					percent_out.data = 0.0;
					make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

					as_.setPreempted(result_);
					return;
				}
				r.sleep();
			}
			return;
		}

		if (goal->unflip_fourbar) {
			return;
		}

		behavior_actions::FourbarElevatorPath2023Goal pathGoal;
		if (goal->piece == goal->VERTICAL_CONE) {
			pathGoal.path = "intake_vertical_cone";
		} else if (goal->piece == goal->DOUBLE_SUBSTATION) {
			pathGoal.path = "intake_double_substation";
		} else {
			pathGoal.path = "intake_cone_cube";
		}
		pathGoal.reverse = false;

		feedback_.status = feedback_.PATHER;
		as_.publishFeedback(feedback_);

		path_ac_.sendGoal(pathGoal);

		ros::Duration(0.25).sleep();

		std_msgs::Float64 percent_out;
		percent_out.data = speed_;
		make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

		ros::Time last_sample_above = ros::TIME_MAX;

		bool current_exceeded = false;
		bool got_game_piece = false;

		while (true) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : waiting for game piece... current = " << current_current_ << ", threshold = " << current_threshold_ << ", path state = " << path_ac_.getState().getText());
			if (current_current_ > current_threshold_ && fourbar_cur_position_ > minimum_fourbar_extension_) {
				if (last_sample_above == ros::TIME_MAX) {
					last_sample_above = ros::Time::now();
				}
				if (ros::Time::now() - last_sample_above >= ros::Duration(minimum_current_time_)) {
					ROS_INFO_STREAM("2023_intaking_server : current limit hit for >= " << minimum_current_time_ << " seconds! Retracting intake!");
					current_exceeded = true;
					got_game_piece = true; // to the best of our knowledge
					break;
				}
			} else {
				last_sample_above = ros::TIME_MAX;
			}
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO_STREAM("2023_intaking_server : preempted.");
				if (current_current_ > current_threshold_ && fourbar_cur_position_ > minimum_fourbar_extension_) {
					current_exceeded = true;
					got_game_piece = true;
					// in case we preempt before minimum_current_time_, use the non-debounced current
					ROS_INFO_STREAM("2023_intaking_server : still hit current limit, running rollers");
				} else {
					ROS_WARN_STREAM("2023_intaking_server : no game piece detected, not running at small speed");
				}

				as_.setPreempted(result_);
				break;
			}
			r.sleep();
		}

		const ros::Time done = ros::Time::now();

		feedback_.status = feedback_.INTAKE;
		as_.publishFeedback(feedback_);

		if (game_piece_state_.game_piece == behavior_actions::GamePieceState2023::CUBE || current_exceeded) {
			ROS_INFO_STREAM("2023_intaking_server : " << (current_exceeded ? "current exceeded" : "cube detected") << ", waiting " << cube_time_ << " seconds");
			ros::Duration(cube_time_).sleep();
		}
		else {
			ROS_INFO_STREAM("2023_intaking_server : cone detected, waiting " << cone_time_ << " seconds");
			ros::Duration(cone_time_).sleep();
		}

		if (got_game_piece) {
			// if got_game_piece is false, then don't run the rollers
			// so make sure to set the current limit correctly so we don't drop things
			percent_out.data = medium_speed_;
			make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it

			// ros::Duration(time_before_reverse_).sleep();
		} else {
			percent_out.data = 0.0;
			make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it
		}
		// move back to holding position (basically zero so it's fine if we do this without a game piece)

		pathGoal.reverse = true;

		feedback_.status = feedback_.PATHER;
		as_.publishFeedback(feedback_);

		path_ac_.cancelGoalsAtAndBeforeTime(done); // cancel old
		path_ac_.sendGoal(pathGoal); // send new

		if (got_game_piece) {
			ros::Duration(medium_time_).sleep();
			percent_out.data = 0.0;
			make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it
			ros::Duration(time_before_reverse_).sleep();
			percent_out.data = small_speed_;
			make_sure_publish(intake_pub_, percent_out); // replace with service based JointPositionController once we write it
		}

		ros::Time start = ros::Time::now();
		while (!path_ac_.getState().isDone()) {
			ros::spinOnce();
			ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : bringing game piece back");
			if ((as_.isPreemptRequested() || !ros::ok()) && (ros::Time::now() - start) > ros::Duration(path_zero_timeout_)) {
				ROS_INFO_STREAM_THROTTLE(0.1, "2023_intaking_server : preempted... has been " << (ros::Time::now() - start).toSec() << " seconds");
				path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				break;
			}
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
