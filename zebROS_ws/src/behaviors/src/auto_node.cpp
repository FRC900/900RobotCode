#include <ros/ros.h>
#include <std_msgs/String.h>
#include <behavior_actions/AutoMode.h> //msg file
#include <std_srvs/Empty.h>
#include <frc_msgs/MatchSpecificData.h>

#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/ElevatorAction.h> //TODO remove this, it's for testing using last year's stuff

#include <thread>
#include <atomic>


//VARIABLES ---------------------------------------------------------
int auto_mode = -1; //-1 if nothing selected
std::vector<std::string> auto_steps; //stores string of action names to do, read from the auto mode array in the config file

bool auto_started = false; //set to true when enter auto time period
bool auto_stopped = false; //set to true if driver stops auto (callback: stopAuto() ) - note: this node will keep doing actions during teleop if not finished and the driver doesn't stop auto
//All actions check if(auto_started && !auto_stopped) before proceeding.

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
	if(msg->Autonomous)
	{
		auto_started = true; //only want to set this to true, never set it to false afterwards
	}
}


//subscriber callback for dashboard data
void updateAutoMode(const behavior_actions::AutoMode::ConstPtr& msg)
{
	auto_mode = msg->auto_mode;
}



//function to publish auto node state (run on a separate thread)
//this is read by the dashboard to display it to the driver
void publishAutoState(ros::Publisher publisher)
{
	//give the thread a name
	pthread_setname_np(pthread_self(), "auto_state_pub_thread");

	//publish
	ros::Rate r(10); //TODO config
	std_msgs::String msg;

	while(ros::ok()){
		switch(auto_state){
			case NOT_READY: msg.data = "Not Ready"; break;
			case READY: msg.data = "Ready"; break;
			case RUNNING: msg.data = "Running"; break;
			case DONE: msg.data = "Done"; break;
			case ERROR: msg.data = "Error"; break;
			default:
				msg.data = "Unknown State";
				ROS_ERROR("Unknown auto state - weirdness in auto_node");
				break;
		}
		publisher.publish(msg);
		r.sleep();
	}
}



//function to wait while an actionlib server is running
void waitForActionlibServer(auto &action_client, double timeout, const std::string &activity)
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

	//publishers
	//auto state
#ifdef __linux__
	ros::Publisher state_pub = nh.advertise<std_msgs::String>("auto_state", 1);
	std::thread auto_state_pub_thread(publishAutoState, state_pub);
#endif

	//servers
	ros::ServiceServer stop_auto_server = nh.advertiseService("stop_auto", stopAuto); //called by teleoop node to stop auto execution during teleop if driver wants

	//actionlib clients
	//TODO
	//example:
	actionlib::SimpleActionClient<behavior_actions::ElevatorAction> elevator_ac("/elevator/elevator_server", true);


	//other variables
	ros::Rate r(10); //used in various places where we wait TODO: config?


	//WAIT FOR MATCH TO START --------------------------------------------------------------------------
	ROS_INFO("Auto node - waiting for autonomous to start");

	//wait for auto period to start
	while( (!auto_started || auto_mode <= 0) && !auto_stopped ) //the auto_mode check is for if we selected an auto mode yet
	{
		if(!ros::ok()){
			auto_state = DONE;
			if(auto_state_pub_thread.joinable()) {
				auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
			}
			return 0;
		}

		ros::spinOnce(); //spin so the subscribers can update

		if(auto_mode > 0){
			auto_state = READY;
		}
		if(auto_started && auto_mode <= 0){
			ROS_ERROR("Auto node - Autonomous period started, please choose an auto mode");
		}

		r.sleep();
	}

	if(auto_stopped){
		ROS_INFO("Auto node - code stopped before execution");
		auto_state = DONE;
		if(auto_state_pub_thread.joinable()) {
			auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
		}
		return 0;
	}

	//EXECUTE AUTONOMOUS ACTIONS --------------------------------------------------------------------------

	ROS_INFO_STREAM("Auto node - Executing auto mode " << auto_mode);
	auto_state = RUNNING;

	//read sequence of actions from config
	if(! nh.getParam("auto_mode_" + std::to_string(auto_mode), auto_steps)){
		ROS_ERROR_STREAM("Couldn't read auto_mode_" + std::to_string(auto_mode) + " config value in auto node");
		auto_state = ERROR;
		if(auto_state_pub_thread.joinable()) {
			auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
		}
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
				ROS_ERROR_STREAM("Auto node - Couldn't read data for '" << auto_steps[i] << "' auto action from config file");
				auto_state = ERROR;
				if(auto_state_pub_thread.joinable()) {
					auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
				}
				return 1;
			}


			// CODE FOR ACTIONS HERE --------------------------------------------------
			//figure out what to do based on the action type, and do it

			if(action_data["type"] == "pause")
			{
				const double start_time = ros::Time::now().toSec();

				//read duration - user could've entered a double or an int, we don't know which
				double duration;
				if(action_data["duration"].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble) {
					duration = (double) action_data["duration"];
				}
				else if(action_data["duration"].getType() == XmlRpc::XmlRpcValue::Type::TypeInt) {
					duration = (double) (int) action_data["duration"];
				}
				else {
					ROS_ERROR_STREAM("Auto node - duration is not a double or int in '" << auto_steps[i] << "' action");
					auto_state = ERROR;
					if(auto_state_pub_thread.joinable()) {
						auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
					}
					return 1;
				}

				//wait
				while (ros::Time::now().toSec() - start_time < duration && !auto_stopped && ros::ok())
				{
					ros::spinOnce();
					r.sleep();
				}
			}
			//placeholder - TODO actually do stuff here
			else if(action_data["type"] == "intake_actionlib_server")
			{
				//do stuff
			}
			//TODO remove test
			else if(action_data["type"] == "elevator_actionlib_server")
			{
				if(!elevator_ac.waitForServer(ros::Duration(5))){
					ROS_ERROR("Auto node - couldn't find elevator actionlib server");
					auto_state = ERROR;
					if(auto_state_pub_thread.joinable()){
						auto_state_pub_thread.join();
					}
					return 1;
				} //for some reason this is necessary, even if the server has been up and running for a while
				behavior_actions::ElevatorGoal goal;
				goal.setpoint_index = (int) action_data["goal"]["setpoint_index"];
				elevator_ac.sendGoal(goal);
				waitForActionlibServer(elevator_ac, 100, "elevator server thing");
			}
			else
			{
				ROS_ERROR_STREAM("Auto node - Invalid type of action: " << action_data["type"]);
				auto_state = ERROR;
				if(auto_state_pub_thread.joinable()) {
					auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
				}
				return 1;
			}
		}

	}

	if(auto_stopped){
		ROS_INFO("Auto node - Autonomous actions stopped before completion");
	}
	else {
		ROS_INFO("Auto node - Autonomous actions completed!");
	}
	auto_state = DONE;
	if(auto_state_pub_thread.joinable()) {
		auto_state_pub_thread.join(); //keeps the publishing going then closes the publish thread when ros not ok
	}
	return 0;
}

