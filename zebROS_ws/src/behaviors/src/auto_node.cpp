// Top-level driver for auto behaviors.
#include <ros/ros.h>
#include <behavior_actions/AutoMode.h> //msg file
#include <behavior_actions/AutoState.h> //msg file
#include <std_srvs/Empty.h>
#include <frc_msgs/MatchSpecificData.h>
#include <geometry_msgs/Point32.h>

#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/ElevatorAction.h> //TODO remove this, it's for testing using last year's stuff
#include <behavior_actions/IntakeAction.h>
#include <behavior_actions/ShooterAction.h>
#include <path_follower_msgs/PathAction.h>

#include <thread>
#include <atomic>


//VARIABLES ---------------------------------------------------------
int auto_mode = -1; //-1 if nothing selected
double distance_from_center = 0.0; // distance from robot to center of goal; left is positive, right is negative
std::vector<std::string> auto_steps; //stores string of action names to do, read from the auto mode array in the config file

bool auto_started = false; //set to true when enter auto time period
bool auto_stopped = false; //set to true if driver stops auto (callback: stopAuto() ) - note: this node will keep doing actions during teleop if not finished and the driver doesn't stop auto
//All actions check if(auto_started && !auto_stopped) before proceeding.

double goal_to_wall_y_dist = 2.404;

enum AutoStates {
	NOT_READY,
	READY,
	RUNNING,
	DONE,
	ERROR
};
std::atomic<int> auto_state(NOT_READY); //This state is published by the publish thread


//FUNCTIONS -------

//server callback for stop autonomous execution
bool stopAuto(std_srvs::Empty::Request &req,
			  std_srvs::Empty::Response &res)
{
	ROS_INFO("Auto node - Stopping code");
	auto_stopped = true;
	return true;
}


//subscriber callback for match data
void matchDataCallback(const frc_msgs::MatchSpecificData::ConstPtr& msg)
{
	if(msg->Autonomous && msg->Enabled)
	{
		auto_started = true; //only want to set this to true, never set it to false afterwards
	}
}


//subscriber callback for dashboard data
void updateAutoMode(const behavior_actions::AutoMode::ConstPtr& msg)
{
	auto_mode = msg->auto_mode;
	distance_from_center = -1 * (goal_to_wall_y_dist - msg->distance_from_wall); // left is positive, right is negative
}


void doPublishAutostate(ros::Publisher &state_pub)
{
	behavior_actions::AutoState msg;
	msg.header.stamp = ros::Time::now();
	msg.id = auto_state;

	switch(auto_state){
		case NOT_READY: msg.string = "Not Ready"; break;
		case READY: msg.string = "Ready"; break;
		case RUNNING: msg.string = "Running"; break;
		case DONE: msg.string = "Done"; break;
		case ERROR: msg.string = "Error"; break;
		default:
					msg.string = "Unknown State";
					ROS_ERROR("Unknown auto state - weirdness in auto_node");
					break;
	}
	state_pub.publish(msg);
}


//function to publish auto node state (run on a separate thread)
//this is read by the dashboard to display it to the driver
std::atomic<bool> publish_autostate{true};
void publishAutoState(ros::NodeHandle &nh)
{
#ifdef __linux__
	//give the thread a name
    pthread_setname_np(pthread_self(), "auto_state_pub");
#endif

    //publish
	ros::Rate r(10); //TODO config
	ros::Publisher state_pub = nh.advertise<behavior_actions::AutoState>("auto_state", 1);

	publish_autostate = true;
	while(publish_autostate) {
		doPublishAutostate(state_pub);
		if (publish_autostate) {
			r.sleep();
		}
	}
	// Force one last message to go out before exiting
	doPublishAutostate(state_pub);
}


//function to wait while an actionlib server is running
template <class T>
void waitForActionlibServer(T &action_client, double timeout, const std::string &activity)
	//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
{
	const double request_time = ros::Time::now().toSec();
	ros::Rate r(10); //TODO config?

	//wait for actionlib server to finish
	std::string state;
	while(!auto_stopped && ros::ok())
	{
		state = action_client.getState().toString();

		if(state == "PREEMPTED") {
			ROS_ERROR_STREAM("Auto node - " << activity << " got preempted");
			auto_stopped = true;
		}
		//check timeout - note: have to do this before checking if state is SUCCEEDED since timeouts are reported as SUCCEEDED
		else if (ros::Time::now().toSec() - request_time > timeout || //timeout from what this file says
				(state == "SUCCEEDED" && !action_client.getResult()->success)) //server times out by itself
		{
			ROS_ERROR_STREAM("Auto node - " << activity << " timed out");
			auto_stopped = true;
			action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
		}
		else if (state == "SUCCEEDED") { //must have succeeded since we already checked timeout possibility
			break; //stop waiting
		}
		else if (auto_stopped){
			action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
		}
		else { //if didn't succeed and nothing went wrong, keep waiting
			ros::spinOnce();
			r.sleep();
		}
	}
}


std::thread auto_state_pub_thread;
void shutdownNode(AutoStates state, const std::string &msg)
{
	if (msg.length()) {
		if (auto_state == ERROR) {
			ROS_ERROR_STREAM(msg);
		}
		else {
			ROS_INFO_STREAM(msg);
		}
	}
	auto_state = state;
	publish_autostate = false; // publish last message and exit from autostate publisher thread
	if(auto_state_pub_thread.joinable()) {
		auto_state_pub_thread.join(); // waits until auto state publisher thread finishes
	}
}


bool waitForAutoStart(void)
{
	ros::Rate r(20);

	//wait for auto period to start
	while( ros::ok() && !auto_stopped )
	{
		ros::spinOnce(); //spin so the subscribers can update

		if(auto_mode > 0){
			auto_state = READY;
		}
		if(auto_started && auto_mode <= 0){
			ROS_ERROR("Auto node - Autonomous period started, please choose an auto mode");
		}

		// Valid auto mode plus auto_started flag ==> actually run auto code, return success
		if (auto_started && (auto_mode > 0)) {
			return true;
		}

		r.sleep();
	}

	shutdownNode(DONE, "Auto node - code stopped before execution");
	return false;
}


int main(int argc, char** argv)
{
	//SETUP --------------------------------------------------------------------------------------------

	//create node
	ros::init(argc, argv, "auto_node");
	ros::NodeHandle nh;

	//subscribers
	//rio match data (to know if we're in auto period)
	ros::Subscriber match_data_sub = nh.subscribe("/frcrobot_rio/match_data", 1, matchDataCallback);
	//dashboard (to get auto mode)
	ros::Subscriber auto_mode_sub = nh.subscribe("auto_mode", 1, updateAutoMode); //TODO get correct topic name (namespace)

	//auto state
	auto_state_pub_thread = std::thread(publishAutoState, std::ref(nh));

	//servers
	ros::ServiceServer stop_auto_server = nh.advertiseService("stop_auto", stopAuto); //called by teleop node to stop auto execution during teleop if driver wants

	//actionlib clients
	actionlib::SimpleActionClient<path_follower_msgs::PathAction> path_ac("/path_follower/path_follower_server", true); //TODO fix this path
	actionlib::SimpleActionClient<behavior_actions::ShooterAction> shooter_ac("/shooter/shooter_server", true);
	actionlib::SimpleActionClient<behavior_actions::IntakeAction> intake_ac("/powercell_intake/powercell_intake_server", true);

	//other variables
	ros::Rate r(10); //used in various places where we wait TODO: config?
	if(! nh.getParam("goal_to_wall_y_dist", goal_to_wall_y_dist)){
		ROS_ERROR_STREAM("Couldn't read goal_to_wall_y_dist in auto node"); //defaults to 2.404
		return 1;
	}

	// TODO - turn this into a loop
	//        Once auto has run, wait at the end of the loop until a service call
	//        resets the code back to the start. Or maybe just look for the robot to to into
	//        teleop and then disabled.  This will be useful for testing
	//        - rather than constantly restarting code, just call the service and go again.
	//WAIT FOR MATCH TO START --------------------------------------------------------------------------
	ROS_INFO("Auto node - waiting for autonomous to start");

	//wait for auto period to start
	if (!waitForAutoStart())
		return 0;

	//EXECUTE AUTONOMOUS ACTIONS --------------------------------------------------------------------------

	ROS_INFO_STREAM("Auto node - Executing auto mode " << auto_mode);
	auto_state = RUNNING;

	//read sequence of actions from config
	if(! nh.getParam("auto_mode_" + std::to_string(auto_mode), auto_steps)){
		shutdownNode(ERROR, "Couldn't read auto_mode_" + std::to_string(auto_mode) + " config value in auto node");
		return 1;
	}

	//run through actions in order
	for(size_t i = 0; i < auto_steps.size(); i++){
		if(auto_started && !auto_stopped)
		{
			ROS_INFO_STREAM("Auto node - running step " << i << ": " << auto_steps[i]);

			//read data from config needed to carry out the action
			XmlRpc::XmlRpcValue action_data;
			if(! nh.getParam(auto_steps[i], action_data)){
				shutdownNode(ERROR, "Auto node - Couldn't read data for '" + auto_steps[i] + "' auto action from config file");
				return 1;
			}
			// TODO :
			//   create a std::map<std::string, std::function(XmlRpc::XmlRpcValue)>
			//   Look up the map using action_data["type"], call the function returned
			//   from the lookup with action_data as an argument.
			//   Each auto action type would then be implmeneted in a separate function

			// CODE FOR ACTIONS HERE --------------------------------------------------
			//figure out what to do based on the action type, and do it

			if(action_data["type"] == "pause")
			{
				const double start_time = ros::Time::now().toSec();

				//read duration - user could've entered a double or an int, we don't know which
				double duration;
				if((action_data["duration"].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble) ||
				   (action_data["duration"].getType() == XmlRpc::XmlRpcValue::Type::TypeInt) ) {
					duration = (double) action_data["duration"];
				}
				else {
					shutdownNode(ERROR, "Auto node - duration is not a double or int in '" + auto_steps[i] + "' action");
					return 1;
				}

				//wait
				while (ros::Time::now().toSec() - start_time < duration && !auto_stopped && ros::ok())
				{
					ros::spinOnce();
					r.sleep();
				}
			}
			else if(action_data["type"] == "intake_actionlib_server")
			{
				//for some reason this is necessary, even if the server has been up and running for a while
				if(!intake_ac.waitForServer(ros::Duration(5))){
					shutdownNode(ERROR,"Auto node - couldn't find intake actionlib server");
					return 1;
				}

				if(action_data["goal"] == "stop")
				{
					intake_ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else {
					behavior_actions::IntakeGoal goal;
					intake_ac.sendGoal(goal);
				}
			}
			else if(action_data["type"] == "shooter_actionlib_server")
			{
				if(!shooter_ac.waitForServer(ros::Duration(5))){
					shutdownNode(ERROR, "Auto node - couldn't find shooter actionlib server");
					return 1;
				} //for some reason this is necessary, even if the server has been up and running for a while
				behavior_actions::ShooterGoal goal;
				goal.mode = 0;
				shooter_ac.sendGoal(goal);
				waitForActionlibServer(shooter_ac, 100, "intake server");
			}
			else if(action_data["type"] == "path")
			{
				if(!path_ac.waitForServer(ros::Duration(5))){ROS_ERROR("Couldn't find path server"); return 1;}
				path_follower_msgs::PathGoal goal;

				//initialize 0, 0, 0 point
				geometry_msgs::Point point;
				point.x = 0;
				point.y = 0;
				point.z = 0;
				goal.points.push_back(point);

				//read array of array of doubles
				XmlRpc::XmlRpcValue points_config = action_data["goal"]["points"];
				for(int i = 0; i < points_config.size(); i++)
				{
					point.x = (double) points_config[i][0];
					point.y = (double) points_config[i][1];
					if(action_data["goal"]["apply_offset"])
					{
						point.y -= distance_from_center; // if the robot is not centered to goal, adjust path
					}
					point.z = (double) points_config[i][2];
					goal.points.push_back(point);
				}

				path_ac.sendGoal(goal);
				waitForActionlibServer(path_ac, 100, "running path");
			}
			else
			{
				shutdownNode(ERROR, "Auto node - Invalid type of action: " + std::string(action_data["type"]));
				return 1;
			}
		}
	}

	shutdownNode(DONE, auto_stopped? "Auto node - Autonomous actions stopped before completion" : "Auto node - Autonomous actions completed!");
	return 0;
}

