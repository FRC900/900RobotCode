#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <behavior_actions/Placing2023Action.h>
#include <behavior_actions/GamePieceState2023.h>
#include <path_follower_msgs/holdPositionAction.h>
#include <talon_state_msgs/TalonState.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>

double getYaw(const geometry_msgs::Quaternion &o) {
    tf2::Quaternion q;
    tf2::fromMsg(o, q);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

class PlacingServer2023
{
protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<behavior_actions::Placing2023Action> as_;
	std::string action_name_;
	// create messages that are used to published result
	behavior_actions::Placing2023Result result_;
	behavior_actions::Placing2023Feedback feedback_;

	behavior_actions::GamePieceState2023 game_piece_state_;
	ros::Time latest_game_piece_time_;

	ros::Subscriber game_piece_sub_;

	actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action> path_ac_;
	actionlib::SimpleActionClient<behavior_actions::Intaking2023Action> ac_intaking_;
	actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction> ac_hold_position_;

	double server_timeout_;
	double game_piece_timeout_;
	double outtake_cube_time_;
	double outtake_cone_time_;
	double time_before_reverse_;

	double elevator_threshold_;

	size_t elevator_idx_;
	double elevator_position_;
	ros::Subscriber talon_states_sub_;

	double imu_tolerance_;
	ros::Subscriber imu_sub_;
	double latest_yaw_;

	double drive_back_time_;
	ros::Publisher cmd_vel_pub_;

public:

	PlacingServer2023(std::string name) :
		as_(nh_, name, boost::bind(&PlacingServer2023::executeCB, this, _1), false),
		action_name_(name),
		path_ac_("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true),
   		ac_intaking_("/intaking/intaking_server_2023", true),
		ac_hold_position_("/hold_position/hold_position_server", true),
		cmd_vel_pub_(nh_.advertise<geometry_msgs::Twist>("/placing/cmd_vel", 1))
	{
		elevator_idx_ = std::numeric_limits<size_t>::max();
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
		if (!nh_.getParam("outtake_cube_time", outtake_cube_time_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find outtake_cube_time, defaulting to 0.5 seconds");
			outtake_cube_time_ = 0.5;
		}
		if (!nh_.getParam("outtake_cone_time", outtake_cone_time_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find outtake_cone_time, defaulting to 1 second");
			outtake_cone_time_ = 1;
		}
		if (!nh_.getParam("elevator_threshold", elevator_threshold_)) {
			// if elevator position < than this, move up. otherwise, place and retract. this only happens if align_intake is not set.
			ROS_WARN_STREAM("2023_placing_server : could not find elevator_threshold, defaulting to 0.25");
			elevator_threshold_ = 0.25;
		}
		if (!nh_.getParam("imu_tolerance", imu_tolerance_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find imu_tolerance, defaulting to 0.1 radians (~6 degrees)");
			imu_tolerance_ = 0.1;
		}
		if (!nh_.getParam("drive_back_time", drive_back_time_)) {
			ROS_WARN_STREAM("2023_placing_server : could not find imu_tolerance, defaulting to 0.5 seconds");
			drive_back_time_ = 0.5;
		}
		game_piece_sub_ = nh_.subscribe("/game_piece/game_piece_state", 1, &PlacingServer2023::gamePieceCallback, this);
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &PlacingServer2023::talonStateCallback, this);
		imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/zeroed_imu", 1, &PlacingServer2023::imuCb, this);
		as_.start();
	}

	~PlacingServer2023(void)
	{
	}

	void imuCb(const sensor_msgs::ImuConstPtr &msg) {
		latest_yaw_ = getYaw(msg->orientation);
	}

	template<class C, class S>
	bool waitForResultAndCheckForPreempt(const ros::Duration & timeout, actionlib::SimpleActionClient<C> & ac, actionlib::SimpleActionServer<S> & as, bool preempt_at_timeout = false, boost::function<bool()> should_exit = [](){return false;}, double rate = 10)
	{
		bool negative_timeout = false;
		if (timeout < ros::Duration(0, 0)) {
			ROS_WARN("waitForResultAndCheckForPreempt : Negative timeout, waiting forever");
			negative_timeout = true;
		}

		ros::Time timeout_time = ros::Time::now() + timeout;

		ros::Rate r(rate);

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

			if (should_exit()) {
				return true;
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

	void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
	{
		// elevator_idx_ == max of size_t at the start
		if (elevator_idx_ == std::numeric_limits<size_t>::max())
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "elevator_leader")
				{
					elevator_idx_ = i;
					break;
				}
			}
		}
		if (elevator_idx_ != std::numeric_limits<size_t>::max())
		{
			elevator_position_ = talon_state.position[elevator_idx_];
		}
		else {
			ROS_ERROR_STREAM("2023_placing_server: Can't find talon with name = " << "elevator_leader");
		}
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

		if (goal->step == goal->ALIGN_INTAKE) {
			ROS_INFO_STREAM("2023_placing_server : align intake");
			if (ros::Time::now() - latest_game_piece_time_ > ros::Duration(game_piece_timeout_)) {
				ROS_ERROR_STREAM("2023_placing_server : game piece position data too old, aborting");
				result_.success = false;
				as_.setAborted(result_);
				return;
			}
			double center_offset = game_piece_state_.offset_from_center;
			if (isnan(center_offset)) {
				ROS_ERROR_STREAM("2023_placing_server : game piece offset is NaN, aborting");
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
			hold_position_goal_.pose.position.y = -center_offset;
			hold_position_goal_.pose.position.z = 0.0;

			hold_position_goal_.isAbsoluteCoord = false;

			ROS_INFO_STREAM("2023_placing_server : holding position, y = " << hold_position_goal_.pose.position.y);

			bool aligned = false;

			ac_hold_position_.sendGoal(hold_position_goal_, actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction>::SimpleDoneCallback(), actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction>::SimpleActiveCallback(),
			  						    [&](const path_follower_msgs::holdPositionFeedbackConstPtr &feedback){
											aligned = feedback->isAligned;
											feedback_.intake_aligned = aligned;
											as_.publishFeedback(feedback_);
										});

			// Ideally we want to enter parking config when we are aligned, but there isn't currently a way to set that from non-teleop

			if (!waitForResultAndCheckForPreempt(ros::Duration(-1), ac_hold_position_, as_, false, [&aligned](){return aligned;})) {
				if (aligned) {
					ROS_INFO_STREAM("2023_placing_server : preempted, intake is aligned, returning success");
					as_.setSucceeded(result_);
				} else {
					ROS_INFO_STREAM("2023_placing_server : preempted, intake not aligned, aborting");
					result_.success = false;
					as_.setAborted(result_);
				}
				ac_hold_position_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				return;
			}

			as_.setSucceeded(result_);
			return;
		}

		// if the goal was align_intake it has returned by now

		behavior_actions::FourbarElevatorPath2023Goal pathGoal;

		pathGoal.path = pathForGamePiece(game_piece, goal->node);
		pathGoal.reverse = false;
		if (goal->from_Trex) {
			pathGoal.path += "_auto";
		}

		uint8_t actual_step = elevator_position_ < elevator_threshold_ ? goal->MOVE : goal->PLACE_RETRACT;

		ROS_INFO_STREAM("2023_placing_server : step = " << std::to_string(actual_step));

		if (actual_step == goal->MOVE) {
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
		} else {
			if (angles::shortest_angular_distance(latest_yaw_, M_PI) > imu_tolerance_) {
				ROS_ERROR_STREAM("2023_placing_server : IMU too far away (" << latest_yaw_ << "rad) from pi radians! Exiting!");
				result_.success = false;
				as_.setAborted(result_);
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

			behavior_actions::Intaking2023Goal intaking_goal_;
			intaking_goal_.outtake = game_piece == goal->CUBE ? intaking_goal_.OUTTAKE_CUBE : intaking_goal_.OUTTAKE_CONE; // don't change this
			ac_intaking_.sendGoal(intaking_goal_);

			if (!(waitForResultAndCheckForPreempt(ros::Duration(goal->CUBE ? outtake_cube_time_ : outtake_cone_time_), ac_intaking_, as_, true, [](){return false;}, 100) && ac_intaking_.getState() == ac_intaking_.getState().SUCCEEDED)) {
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
			ros::Rate r(50);
			ros::Time move_back_time = ros::Time::now();

			if (!goal->no_drive_back) {
				while ((ros::Time::now() - move_back_time) < ros::Duration(drive_back_time_)) {
					ros::spinOnce();
					ROS_INFO_STREAM_THROTTLE(0.1, "2023_placing_server : driving backwards");
					geometry_msgs::Twist cmd_vel;
					cmd_vel.linear.x = -0.5; // green button states
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = 0.0;
					cmd_vel_pub_.publish(cmd_vel);
					r.sleep();
				}
				geometry_msgs::Twist cmd_vel;
				cmd_vel.linear.x = 0.0; 
				cmd_vel.linear.y = 0.0;
				cmd_vel.linear.z = 0.0;
				cmd_vel.angular.x = 0.0;
				cmd_vel.angular.y = 0.0;
				cmd_vel.angular.z = 0.0;
				cmd_vel_pub_.publish(cmd_vel);
			}

			ROS_INFO_STREAM("2023_placing_server : reversing path!");
			
			pathGoal.path = pathForGamePiece(game_piece, goal->node) + "_reverse";
			pathGoal.reverse = false;
			
			path_ac_.sendGoal(pathGoal);

			// if (!(waitForResultAndCheckForPreempt(ros::Duration(-1), path_ac_, as_, false, [](){return false;}, 100) && path_ac_.getState() == path_ac_.getState().SUCCEEDED)) {
			// 	ROS_INFO_STREAM("2023_placing_server : pather timed out, aborting");
			// 	result_.success = false;
			// 	as_.setAborted(result_);
			// 	path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			// 	return;
			// }

			result_.success = true;
			as_.setSucceeded(result_);
		}
	}


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "placing_server_2023");

	PlacingServer2023 placer("placing_server_2023");
	ros::spin();

	return 0;
}
