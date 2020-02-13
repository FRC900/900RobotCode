#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>
#include <mutex>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/ShooterAction.h"
#include "behavior_actions/IndexerGoal.h"
#include "behavior_actions/IndexerAction.h"

//include controller service files and other service files
#include "controllers_2020_msgs/ShooterSrv.h"
#include "controllers_2020_msgs/IndexerSrv.h"

//include msg types for goal subscriber
#include "field_obj/Detection.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

//create the class for the actionlib server
class ShooterAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::ShooterAction> as_; //create the actionlib server
		std::string action_name_;

		//clients to call controllers
		ros::ServiceClient shooter_client_; //create a ros client to send requests to the controller

		//clients to call actionlib server
		actionlib::SimpleActionClient<behavior_actions::IndexerAction> ac_indexer_;

		ros::Subscriber ready_to_shoot_sub_;
		std::atomic<bool> ready_to_shoot_;
		ros::Subscriber goal_sub_;
		std::mutex goal_msg_mutex_;
		field_obj::Detection goal_msg_;
		ros::Subscriber num_balls_sub_;
        std::atomic<int> num_balls_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

		//Use to make pauses while still checking timed_out_ and preempted_
		bool pause(const double duration, const std::string &activity)
		{
			const double pause_start_time = ros::Time::now().toSec();

			while(!preempted_ && !timed_out_ && ros::ok())
			{
				if(as_.isPreemptRequested() || !ros::ok())
				{
					preempted_ = true;
					ROS_ERROR_STREAM("shooter_server: preempt during pause() - " << activity);
				}
				else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
				{
					timed_out_ = true;
					ROS_ERROR_STREAM("shooter_server: timeout during pause() - " << activity);
				}

				if((ros::Time::now().toSec() - pause_start_time) >= duration)
				{
					return true; //pause worked like expected
				}
			}

			return false; //wait loop must've been broken by preempt, global timeout, or ros not ok
		}

		bool getHoodAndVelocity(bool& hood_extended, double& shooter_speed)
		{
			field_obj::Detection local_goal_msg;
			{
				std::lock_guard<std::mutex> l(goal_msg_mutex_);
				local_goal_msg = goal_msg_;
			}
			// use local_goal_msg here
			hood_extended = true;
			shooter_speed = 3;
			return true;
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::ShooterGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;

			if(!ac_indexer_.waitForServer(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " couldn't find indexer actionlib server");
				as_.setPreempted();
				return;
			}

			//wait for all controller servers we need
			if(! shooter_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find controllers_2020_msgs");
				as_.setPreempted();
				return;
			}

			while(num_balls_ > 0 && !timed_out_ && !preempted_)
			{
				ROS_INFO_STREAM(action_name_ << ": spinning up the shooter");
				controllers_2020_msgs::ShooterSrv srv;
				bool shooter_hood_raise;
				double shooter_velocity;
				getHoodAndVelocity(shooter_hood_raise, shooter_velocity);
				srv.request.shooter_hood_raise = shooter_hood_raise;
				srv.request.set_velocity = shooter_velocity;
				if(!shooter_client_.call(srv))
				{
					ROS_ERROR_STREAM(action_name_ << " can't call shooter service");
					as_.setPreempted();
				}
				ROS_INFO_STREAM(action_name_ << ": called shooter service");
				double start_wait_for_ready_time = ros::Time::now().toSec();
				while(!preempted_ && !timed_out_ && ros::ok())
				{
					ROS_INFO_STREAM(action_name_ << ": waiting for the shooter to be ready");
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while calling shooter controller");
						preempted_ = true;
					}
					//test if succeeded, if so, break out of the loop
					else if(ready_to_shoot_) {
						break;
					}
					//check timed out 
					else if (ros::Time::now().toSec() - start_time_ > server_timeout_ || ros::Time::now().toSec() - start_wait_for_ready_time > wait_for_ready_timeout_) {
						ROS_ERROR_STREAM(action_name_ << ": timed out while calling shooter controller");
						timed_out_ = true;
					}
					else {
						r.sleep();
					}
				}
				if(!preempted_ && !timed_out_ && ros::ok())
				{
					ROS_INFO_STREAM(action_name_ << ": calling indexer server to feed one ball into the shooter");
					//Call actionlib server
					behavior_actions::IndexerGoal indexer_goal;
					indexer_goal.action = 1; //TODO make this into an enum
					ac_indexer_.sendGoal(indexer_goal);
					//wait for actionlib server
					waitForActionlibServer(ac_indexer_, 30, "calling indexer server"); //method defined below. Args: action client, timeout in sec, description of activity
					pause(wait_for_shoot_timeout_, "waiting for shooter to shoot after indexer fed in a ball");
				}
			}

			//set final state using client calls - if you did preempt handling before, put a check here so don't override that
			controllers_2020_msgs::ShooterSrv srv;
			srv.request.set_velocity = 0;
			srv.request.shooter_hood_raise = false;
			if(!shooter_client_.call(srv))
			{
				ROS_ERROR_STREAM(action_name_ << " can't call shooter service");
				as_.setPreempted();
			}
			ROS_INFO_STREAM(action_name_ << ": calling indexer server to go to intake position");
			//Call actionlib server
			behavior_actions::IndexerGoal indexer_goal;
			indexer_goal.action = 0; //TODO make this into an enum
			ac_indexer_.sendGoal(indexer_goal);
			//wait for actionlib server
			waitForActionlibServer(ac_indexer_, 30, "calling indexer server"); //method defined below. Args: action client, timeout in sec, description of activity

			//log result and set actionlib server state appropriately
			behavior_actions::ShooterResult result;

			if(preempted_) {
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if(timed_out_) {
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result); //timed out is encoded as succeeded b/c actionlib doesn't have a timed out state
			}
			else { //implies succeeded
				ROS_INFO("%s: Finished - Succeeded", action_name_.c_str());
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);
			}
			return;

		}

		void waitForActionlibServer(auto &action_client, double timeout, const std::string &activity)
			//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
		{
			double request_time = ros::Time::now().toSec();

			//wait for actionlib server to finish
			std::string state;
			while(!preempted_ && !timed_out_ && ros::ok())
			{
				state = action_client.getState().toString();

				if(state == "PREEMPTED") {
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server returned preempted_ during " << activity);
					preempted_ = true;
				}
				//check timeout - note: have to do this before checking if state is SUCCEEDED since timeouts are reported as SUCCEEDED
				else if (ros::Time::now().toSec() - request_time > timeout || //timeout from what this file says
						(state == "SUCCEEDED" && !action_client.getResult()->success)) //server times out by itself
				{
					ROS_ERROR_STREAM(action_name_ << ": external actionlib server timed out during " << activity);
					timed_out_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else if (state == "SUCCEEDED") { //must have succeeded since we already checked timeout possibility
					break; //stop waiting
				}
				//checks related to this file's actionlib server
				else if (as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempted_ during " << activity);
					preempted_ = true;
				}
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out during " << activity);
					timed_out_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else { //if didn't succeed and nothing went wrong, keep waiting
					ros::spinOnce();
					r.sleep();
				}
			}
		}

		void shooterReadyCB(const std_msgs::Bool &msg)
		{
			ready_to_shoot_ = msg.data;
		}

		void goalDetectionCB(const field_obj::Detection &msg)
		{
			std::lock_guard<std::mutex> l(goal_msg_mutex_);
			goal_msg_ = msg;
		}

		void numBallsCB(const std_msgs::Int32 &msg)
		{
			num_balls_ = msg.data;
		}

	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		ShooterAction(const std::string &name) :
			as_(nh_, name, boost::bind(&ShooterAction::executeCB, this, _1), false),
			action_name_(name),
			ac_indexer_("/indexer/indexer_server", true)
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize client used to call controllers
		shooter_client_ = nh_.serviceClient<controllers_2020_msgs::ShooterSrv>("/frcrobot_jetson/shooter_controller/shooter_command", false, service_connection_header);

		ready_to_shoot_sub_ = nh_.subscribe("/frcrobot_jetson/shooter_controller/ready_to_shoot", 5, &ShooterAction::shooterReadyCB, this);
		goal_sub_ = nh_.subscribe("/goal_sub", 5, &ShooterAction::goalDetectionCB, this);
		num_balls_sub_ = nh_.subscribe("/indexer_actionlib_server/num_balls", 5, &ShooterAction::numBallsCB, this);
	}

		~ShooterAction(void)
		{
		}

		//config values
		double server_timeout_;
		double wait_for_server_timeout_;
		double wait_for_ready_timeout_;
		double wait_for_shoot_timeout_;
		double wait_for_intake_position_timeout_;
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "shooter_server");
	ros::NodeHandle nh;

	//create the actionlib server
	ShooterAction shooter_action("shooter_server");

	//get config values
	ros::NodeHandle n_params_shooter(nh, "actionlib_params"); //node handle for a lower-down namespace
	if (!n_params_shooter.getParam("server_timeout", shooter_action.server_timeout_)) {
		ROS_ERROR("Could not read server_timeout in shooter_server");
		shooter_action.server_timeout_ = 10;
	}
	if (!n_params_shooter.getParam("wait_for_server_timeout", shooter_action.wait_for_server_timeout_)) {
		ROS_ERROR("Could not read wait_for_server_timeout in shooter_sever");
		shooter_action.wait_for_server_timeout_ = 10;
	}
	if (!n_params_shooter.getParam("wait_for_shoot_timeout", shooter_action.wait_for_shoot_timeout_)) {
		ROS_ERROR("Could not read wait_for_shoot_timeout in shooter_sever");
		shooter_action.wait_for_shoot_timeout_ = 10;
	}
	if (!n_params_shooter.getParam("wait_for_ready_timeout", shooter_action.wait_for_ready_timeout_)) {
		ROS_ERROR("Could not read wait_for_ready_timeout in shooter_sever");
		shooter_action.wait_for_ready_timeout_ = 10;
	}
	if (!n_params_shooter.getParam("wait_for_intake_position_timeout", shooter_action.wait_for_intake_position_timeout_)) {
		ROS_ERROR("Could not read wait_for_intake_position_timeout in shooter_sever");
		shooter_action.wait_for_intake_position_timeout_ = 10;
	}

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
