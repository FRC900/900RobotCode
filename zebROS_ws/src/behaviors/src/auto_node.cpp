// Top-level driver for auto behaviors.
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/algorithm/string.hpp> // find and replace 
#include <boost/algorithm/string/predicate.hpp>
#include <ros/ros.h>
#include <behavior_actions/AutoMode.h> //msg file
#include <behavior_actions/AutoState.h> //msg file
#include "geometry_msgs/Twist.h"
#include <std_srvs/Empty.h>
#include <frc_msgs/MatchSpecificData.h>
#include <unordered_map>
#include "base_trajectory_msgs/GenerateSpline.h"
#include "base_trajectory_msgs/PathOffsetLimit.h"
#include <actionlib/client/simple_action_client.h>
#include <behavior_actions/DynamicPath.h>
#include <path_follower_msgs/PathAction.h>
#include <path_follower_msgs/PathFeedback.h>
#include <string>
#include <atomic>
#include <functional>
#include <utility>
#include <vector>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "angles/angles.h"
#include <std_srvs/SetBool.h>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// year specific actions
#include <behavior_actions/AlignToSpeaker2024Action.h>
#include <behavior_actions/Intaking2024Action.h>
#include <behavior_actions/Shooting2024Action.h>

#include <behavior_actions/AutoAlignSpeaker.h>
#include <behavior_actions/DriveObjectIntake2024Action.h>
#include "behavior_actions/RelocalizePoint.h"

// include tf listener/tf buffer
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/Imu.h>


enum AutoStates {
			NOT_READY,
			READY,
			RUNNING,
			DONE,
			ERROR
};

class AutoNode { 
	private:
		// ---------------- START probably not changing year to year ----------------

		ros::NodeHandle nh_;

		ros::Timer auto_state_timer_;
		ros::Publisher auto_state_pub_; // publish auto state with ros_timer
		ros::Publisher orient_command_pub_;
		std::atomic<int> auto_state_; //This state is published by the publish thread
		std::atomic<bool> publish_autostate_{true};
		//servers
		ros::ServiceServer stop_auto_server_; //called by teleop node to stop auto execution during teleop if driver wants
		ros::ServiceServer reset_maps_;

		const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};
		ros::ServiceClient spline_gen_cli_;
		ros::Publisher cmd_vel_pub_;
		ros::ServiceClient brake_srv_;
		ros::ServiceClient park_srv_;
		ros::ServiceClient tagslam_relocalize_srv_;
		
		ros::ServiceClient zero_odom_srv_;
		ros::ServiceClient toggle_relocalize_srv_;
		ros::ServiceClient relocalize_point_srv_;

		ros::ServiceClient pause_srv_;
		bool park_enabled = false;
		bool have_canceled = false; 
		//subscribers
		//rio match data (to know if we're in auto period)
		ros::Subscriber match_data_sub_;
		//dashboard (to get auto mode)
		ros::Subscriber auto_mode_sub_;
		ros::Subscriber enable_auto_in_teleop_sub_;
		ros::Subscriber orientation_effort_sub_; 
		ros::Subscriber current_yaw_sub_;
		ros::Subscriber imu_sub_;
		sensor_msgs::Imu latest_imu_;

		ros::Subscriber dist_ang_sub_;
		double speaker_dist;

		// auto mode and state
		signed char auto_mode_ = -1; //-1 if nothing selected
		int8_t alliance_color_ = frc_msgs::MatchSpecificData::ALLIANCE_COLOR_UNKNOWN;
		std::vector<std::string> auto_steps_; //stores string of action names to do, read from the auto mode array in the config file
		bool enable_teleop_ = false;
		bool auto_started_ = false; //set to true when enter auto time period
		bool auto_stopped_ = false; //set to true if driver stops auto (callback: stopAuto() ) - note: this node will keep doing actions during teleop if not finished and the driver doesn't stop auto
		//All actions check if(auto_started && !auto_stopped) before proceeding.
		// define preemptAll_
        double current_orient_effort_{0};
		double angle_tolerance_{angles::from_degrees(0.5)}; // +/- this is allowed error
		double current_yaw_;
		std::function<void()> preemptAll_;
		// I don't really see us ever actually needing to config this, but it is pretty easy to do
		ros::Rate r_ = ros::Rate(100);
		// Used to pass in dynamic paths from other nodes
		// Map of the auto action to the function to be called
		// Edit here if for changing auto year to year
		// https://stackoverflow.com/questions/8936578/how-to-create-an-unordered-map-for-string-to-function
		// invoke:
		// (this->*functionMap_["foo"])();
		// probably better way to do this
		std::unordered_map<std::string, bool(AutoNode::*)(XmlRpc::XmlRpcValue, const std::string&)> functionMap_;
		ros::ServiceServer path_finder_;

		// transform listener and buffer
		tf2_ros::Buffer tf_buffer_;
		tf2_ros::TransformListener tf_listener_;

		// ---------------- END probably not changing year to year ---------------- 

		// START probably changing year to year, mostly year specific actions but also custom stuff based on what is needed
		//actionlib clients
		actionlib::SimpleActionClient<path_follower_msgs::PathAction> path_ac_; //TODO fix this path
		actionlib::SimpleActionClient<behavior_actions::AlignToSpeaker2024Action> align_to_speaker_ac_;

		actionlib::SimpleActionClient<behavior_actions::Intaking2024Action> intaking_ac_;
		actionlib::SimpleActionClient<behavior_actions::Shooting2024Action> shooting_ac_;
		actionlib::SimpleActionClient<behavior_actions::DriveObjectIntake2024Action> auto_intake_ac_;


		// path follower and feedback
		std::map<std::string, nav_msgs::Path> premade_position_paths_;
		std::map<std::string, nav_msgs::Path> premade_velocity_paths_;
		// Inital waypoints used to make the paths, when passed into the path follower allows for more persise control
		// Can use for things like "start intake after X waypoint or X percent through"
		std::map<std::string, nav_msgs::Path> premade_position_waypoints_;
		std::map<std::string, nav_msgs::Path> premade_velocity_waypoints_;
		// Stores the waypoint that each section of the generated path corresponds to
		std::map<std::string, std::vector<int>> waypointsIdxs_;
		int old_waypoint_ = 0;
		double old_percent_complete_ = 0.0;
		double old_waypoint_percent_ = 0.0;
		// Publishes first point of auto path
		ros::Publisher first_point_pub_;
		// END probably changing year to year ----------------

	public:
		AutoNode(const ros::NodeHandle &nh) 
		: nh_(nh)
		, path_ac_("/path_follower/path_follower_server", true)
		, align_to_speaker_ac_("/align_to_speaker/align_to_speaker_2024", true)
		, intaking_ac_("/intaking/intaking_server_2024", true)
		, shooting_ac_("/shooting/shooting_server_2024", true)
		, auto_intake_ac_("/intaking/drive_object_intake", true)
		, tf_listener_(tf_buffer_)

	// Constructor
	{
		// START probably not changing year to year ----------------
		spline_gen_cli_ = nh_.serviceClient<base_trajectory_msgs::GenerateSpline>("/path_follower/base_trajectory/spline_gen", false, service_connection_header);
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		brake_srv_ = nh_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
		park_srv_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/swerve_drive_controller/toggle_park", false, service_connection_header);
		tagslam_relocalize_srv_ = nh_.serviceClient<std_srvs::Empty>("/tagslam_pub_map_to_odom", false, service_connection_header);
		zero_odom_srv_ = nh_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/reset_odom", false, service_connection_header);
		toggle_relocalize_srv_ = nh_.serviceClient<std_srvs::SetBool>("/toggle_map_to_odom", false, service_connection_header);
		relocalize_point_srv_ = nh_.serviceClient<behavior_actions::RelocalizePoint>("/relocalize_point", false, service_connection_header);
		
		pause_srv_ = nh_.serviceClient<std_srvs::SetBool>("/path_follower/pause_path", false, service_connection_header);

		imu_sub_ = nh_.subscribe("/imu/zeroed_imu", 1, &AutoNode::imuCallback, this);

		//subscribers
		//rio match data (to know if we're in auto period)
		match_data_sub_ = nh_.subscribe("/frcrobot_rio/match_data", 1, &AutoNode::matchDataCallback, this);
		//dashboard (to get auto mode)
		auto_mode_sub_ = nh_.subscribe("auto_mode", 1, &AutoNode::updateAutoMode, this); //TODO get correct topic name (namespace)
		enable_auto_in_teleop_sub_ = nh_.subscribe("/enable_auto_in_teleop", 1, &AutoNode::enable_auto_in_teleop, this);
		orientation_effort_sub_ = nh_.subscribe("/teleop/orient_strafing/control_effort", 1, &AutoNode::orientation_effort_callback, this, ros::TransportHints().tcpNoDelay());

		dist_ang_sub_ = nh_.subscribe("/speaker_align/dist_and_ang", 1, &AutoNode::distance_callback, this, ros::TransportHints().tcpNoDelay());

		// Used to pass in dynamic paths from other nodes
		path_finder_ = nh_.advertiseService("dynamic_path", &AutoNode::dynamic_path_storage, this);
        orient_command_pub_ = nh_.advertise<std_msgs::Float64>("/teleop/orientation_command", 1);
		current_yaw_sub_ = nh_.subscribe("/teleop/orient_strafing/state", 1, &AutoNode::yaw_callback, this, ros::TransportHints().tcpNoDelay());
		if (!nh_.getParam("angle_tolerance", angle_tolerance_))
		{
			ROS_ERROR_STREAM("Could not read param "
								<< "angle_tolerance_"
								<< " in auto_node");
		}

		//auto state
		auto_state_ = NOT_READY;
		// read param 
		
		#if 0 // attempt to config rate
		int auto_ros_rate = 10;
		if (!nh_.getParam("ros_rate", auto_ros_rate))
		{
			ROS_ERROR_STREAM("Could not read param "
								<< "ros_rate"
								<< " in auto_node");
		}
		r_ = ros::Rate(auto_ros_rate);
		#endif
		auto_state_pub_ = nh_.advertise<behavior_actions::AutoState>("auto_state", 1);
		auto_state_timer_ = nh_.createTimer(ros::Duration(0.1), &AutoNode::publishAutoState, this);
		//servers
		// not sure if this is needed or even works
		stop_auto_server_ = nh_.advertiseService("stop_auto", &AutoNode::stopAuto, this); //called by teleop node to stop auto execution during teleop if driver wants
		reset_maps_ = nh_.advertiseService("reset_maps", &AutoNode::resetMaps, this);
		// END probably not changing year to year ----------------


		// START change year to year
		// better way to initalize?
		first_point_pub_ = nh_.advertise<geometry_msgs::Pose>("first_point", 1, true);
		functionMap_["relocalize"] = &AutoNode::relocalizefn;
		functionMap_["pause"] = &AutoNode::pausefn;
		functionMap_["pause_path"] = &AutoNode::pausePathfn;
		functionMap_["path"] = &AutoNode::pathfn;
		functionMap_["cmd_vel"] = &AutoNode::cmdvelfn;
		functionMap_["snap_to_orientation"] = &AutoNode::snapanglefn;
		
		functionMap_["align_to_speaker"] = &AutoNode::alignToSpeakerfn;
		functionMap_["start_intake"] = &AutoNode::startIntakingfn;
		functionMap_["stop_intake"] = &AutoNode::stopIntakingfn;
		functionMap_["auto_intake"] = &AutoNode::autoIntakefn;
		functionMap_["wait_for_action"] = &AutoNode::waitForActionfn;
		functionMap_["spin_up_shooter"] = &AutoNode::spinUpShooterfn;
		functionMap_["spin_down_shooter"] = &AutoNode::spinDownShooterfn;
		functionMap_["shoot_distance"] = &AutoNode::shootDistancefn;
		functionMap_["run_closest_path"] = &AutoNode::runClosestPathfn;
		functionMap_["counter_auto_source"] = &AutoNode::counterAutofn;
		// cool trick to bring all class variables into scope of lambda
		preemptAll_ = [this](){ // must include all actions called
			path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			align_to_speaker_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			intaking_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			shooting_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			if (park_enabled) {
				std_srvs::SetBool srv;
				srv.request.data = false;
				if (!park_srv_.call(srv)) {
					ROS_INFO_STREAM_THROTTLE(5, "Failed to call park service in auto node! :(");
				} else {
					ROS_INFO_STREAM("Swerve will now not park during teleop when stopped");
					park_enabled = false;
				}
			}
		};
		// END change year to year
	}

	void imuCallback(const sensor_msgs::Imu &msg) {
		latest_imu_ = msg;
	}

	//FUNCTIONS -------
	// lots of stuff for reading params, waiting for actionlib servers, etc that
	// can be kept the same year to year, most of the stuff to change will be in the function map and what they do
	// and in the config 
	// ---------------- START NOT CHANGING year to year ----------------

	//server callback for stop autonomous execution
	bool stopAuto(std_srvs::Empty::Request &/*req*/,
				std_srvs::Empty::Response &/*res*/)
	{
		ROS_INFO("Auto node - Stopping code");
		auto_stopped_ = true;
		preemptAll_();
		return true;
	}

	void yaw_callback(const std_msgs::Float64::ConstPtr& msg) {
		current_yaw_ = msg->data; 
	}

	void distance_callback(const behavior_actions::AutoAlignSpeaker::ConstPtr& msg) {
		speaker_dist = msg->distance;
	}

	//subscriber callback for match data
	void matchDataCallback(const frc_msgs::MatchSpecificData::ConstPtr& msg)
	{
		alliance_color_ = msg->allianceColor;
		// have to do this check here because otherwise it will be modified in the check below
		// all of the cases where auto_stopped_ gets set to true are bad, preempt/aborted/timeout no real way to recover
		if (auto_stopped_) {
			shutdownNode(AutoStates::ERROR, "auto_stopped_ set to true");
			return;
		}
		// must be in auto and enabled and have never had auto_stopped_ set to true 
		// diffrent than before where we should hit the first block of being enabled in auto mode
		// not sure if we rely on this for actual auto which is only 15 seconds or if it is just when it runs forever in sim
		if((msg->Autonomous && msg->Enabled) || (msg->Enabled && enable_teleop_))
		{
			auto_stopped_ = false;
			auto_started_ = true; //only want to set this to true, never set it to false afterwards
			if (!park_enabled) {
				std_srvs::SetBool srv;
				srv.request.data = true;
				if (!park_srv_.call(srv)) {
					ROS_INFO_STREAM_THROTTLE(5, "Failed to call park service in auto node! :(");
				} else {
					ROS_INFO_STREAM("Swerve will now park during auto when stopped");
					park_enabled = true;
				}
			}
		}
		if((auto_started_ && !msg->Enabled))
		{
			auto_started_ = false;
			preemptAll_();
		}
	}

	void orientation_effort_callback(const std_msgs::Float64& msg) 
	{
		current_orient_effort_ = msg.data;
	}

	//subscriber callback for dashboard data
	void updateAutoMode(const behavior_actions::AutoMode::ConstPtr& msg)
	{
		auto_mode_ = msg->auto_mode;
	}

	void enable_auto_in_teleop(const std_msgs::Bool::ConstPtr& msg)
	{
		enable_teleop_ = msg->data;
	}

	bool dynamic_path_storage(behavior_actions::DynamicPath::Request &req, behavior_actions::DynamicPath::Response &/*res*/)
	{
		ROS_INFO_STREAM("auto_node : addding " << req.path_name << " to premade_paths");
		premade_position_paths_[req.path_name] = req.dynamic_position_path;
		premade_velocity_paths_[req.path_name] = req.dynamic_velocity_path;
		premade_position_waypoints_[req.path_name] = nav_msgs::Path();
		premade_velocity_waypoints_[req.path_name] = nav_msgs::Path();
		for (int i = 0; i < req.dynamic_position_path.poses.size(); i++) {
			waypointsIdxs_[req.path_name].push_back(0);
		}
		return true;
	}

	void DoPublishAutostate(const ros::TimerEvent& event)
	{
		// we don't actually care about the event much
		behavior_actions::AutoState msg;
		msg.header.stamp = event.current_real;
		msg.id = auto_state_;

		switch(auto_state_){
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
		auto_state_pub_.publish(msg);
	}

	//function to publish auto node state (run on a separate thread)
	//this is read by the dashboard to display it to the driver
	
	void publishAutoState(const ros::TimerEvent& event)
	{	
		if (publish_autostate_) {
			DoPublishAutostate(event);
		}
		else {
			ROS_WARN("AutoNode::publishAutoState() publishing last message");
			DoPublishAutostate(event); // should be the last publish
		}
	}


	//function to wait while an actionlib server is running
	template <class T>
	void waitForActionlibServer(T &action_client, double timeout, const std::string &activity)
		//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
	{
		const double request_time = ros::Time::now().toSec();

		//wait for actionlib server to finish
		std::string state;
		while(!auto_stopped_ && ros::ok())
		{
			state = action_client.getState().toString();

			if(state == "PREEMPTED") {
				ROS_ERROR_STREAM("Auto node - " << activity << " got preempted");
				auto_stopped_ = true;
			}
			else if(state == "ABORTED") {
				ROS_ERROR_STREAM("Auto node - " << activity << " was aborted / rejected");
				auto_stopped_ = true;
			}
			//check timeout - note: have to do this before checking if state is SUCCEEDED since timeouts are reported as SUCCEEDED
			else if (ros::Time::now().toSec() - request_time > timeout || //timeout from what this file says
					(state == "SUCCEEDED" && !action_client.getResult()->success)) //server times out by itself
			{
				ROS_ERROR_STREAM("Auto node - " << activity << " timed out");
				auto_stopped_ = true;
				action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
			}
			else if (state == "SUCCEEDED") { //must have succeeded since we already checked timeout possibility
				ROS_WARN_STREAM("Auto node - " << activity << " succeeded");
				break; //stop waiting
			}
			else if (auto_stopped_){
				ROS_WARN_STREAM("Auto node - auto_stopped_ set");
				action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
			}
			else { //if didn't succeed and nothing went wrong, keep waiting
				ros::spinOnce();
				r_.sleep();
			}
		}
	}

	void shutdownNode(AutoStates state, const std::string &msg)
	{
		if (msg.length()) {
			if (auto_state_ == ERROR) {
				ROS_ERROR_STREAM(msg);
			} else {
				ROS_INFO_STREAM(msg);
			}
		}
		auto_state_ = state;
		publish_autostate_ = false; // publish last message and exit from autostate publisher thread
		preemptAll_();
		// make 100% sure that it has published last auto state
		for (int i=0; i < 5; i++) {
			r_.sleep();
		}
		if (park_enabled) {
			std_srvs::SetBool srv;
			srv.request.data = false;
			if (!park_srv_.call(srv)) {
				ROS_INFO_STREAM_THROTTLE(5, "Failed to call park service in auto node! :(");
			} else {
				ROS_INFO_STREAM("Swerve will now not park during teleop when stopped");
				park_enabled = false;
			}
		}
		exit(0);
	}

	bool readStringParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, std::string &val)
	{
		if (!params.hasMember(param_name))
			return false;
		XmlRpc::XmlRpcValue &param = params[param_name];
		if (!param.valid())
			throw std::runtime_error(param_name + " was not a valid string type");
		if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
		{
			val = static_cast<std::string>(param);
			return true;
		}
		throw std::runtime_error("A non-string value was read for" + param_name);

		return false;
	}

	bool readIntParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, int &val)
	{
		if (!params.hasMember(param_name))
			return false;
		XmlRpc::XmlRpcValue &param = params[param_name];
		if (!param.valid())
			throw std::runtime_error(param_name + " was not a valid int type");
		if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
		{
			val = static_cast<int>(param);
			return true;
		}
		else
			throw std::runtime_error("A non-double value was read for" + param_name);

		return false;
	}


	bool readFloatParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, double &val)
	{
		if (!params.hasMember(param_name))
			return false;
		XmlRpc::XmlRpcValue &param = params[param_name];
		if (!param.valid())
			throw std::runtime_error(param_name + " was not a valid double type");
		if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
		{
			val = static_cast<double>(param);
			return true;
		}
		else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
		{
			val = static_cast<int>(param);
			return true;
		}
		else
			throw std::runtime_error("A non-double value was read for" + param_name);

		return false;
	}

	bool extractFloatVal(XmlRpc::XmlRpcValue &param, double &val)
	{
		if (!param.valid())
			throw std::runtime_error("val was not a valid double type");
		if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
		{
			val = static_cast<double>(param);
			return true;
		}
		else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
		{
			val = static_cast<int>(param);
			return true;
		}
		else
			throw std::runtime_error("A non-double value was read for value");

		return false;
	}

	bool readBoolParam(const std::string &param_name, XmlRpc::XmlRpcValue &params, bool &val)
	{
		if (!params.hasMember(param_name))
			return false;
		XmlRpc::XmlRpcValue &param = params[param_name];
		if (!param.valid())
			throw std::runtime_error(param_name + " was not a valid bool type");
		if (param.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
		{
			val = static_cast<bool>(param);
			return true;
		}
		else
			throw std::runtime_error("A non-bool value was read for" + param_name);

		return false;
	}

	bool waitForAutoEnd() // returns true if no errors
	{

		bool isOk = ros::ok();
		while (isOk && !auto_stopped_ && auto_started_)
		{
			isOk = ros::ok();
			ros::spinOnce(); // spin so the subscribers can update
			r_.sleep(); // wait for 1/20 of a second
		}
		if (!isOk) { // ROS not being ok is an error, return false
			return false;
		}
		return true;
	}

	std::string getAllianceColorString() { 
		std::string color_str = ""; 
		if (alliance_color_ == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_UNKNOWN) {
			ROS_ERROR_STREAM("UNKNOWN ALLIANCE COLOR, REQUIRED FOR AUTO - EXITING");
			return "";
		}
		else if (alliance_color_ == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED) {
			color_str = "red";
		}
		else if (alliance_color_ == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_BLUE) {
			color_str = "blue";
		}
		else {
			ROS_ERROR_STREAM("ALLIANCE COLOR FELL THROUGH, EXITING");
			return "";
		}
		return color_str;
	}
	// ---------------- END NOT CHANGING year to year ----------------

	// Called before auto actually starts, runs the path planner and stores the path in premade_paths_
 	bool preLoadPath() {
		for (size_t j = 0; j < auto_steps_.size(); j++) {
			XmlRpc::XmlRpcValue action_data;
			
			std::string color_str = getAllianceColorString(); 
			if (color_str == "") {
				return false; 
			}
			const std::string ALLIANCE = "ALLIANCE"; 
			boost::replace_all(auto_steps_[j], ALLIANCE, color_str); 
			//ROS_INFO_STREAM("Auto mode j " << j << " step: " << auto_steps_[j]); 

			// If the path was already retrieved, continue
			if (premade_position_paths_.find(auto_steps_[j]) != premade_position_paths_.end()) {
				continue;
			}

			
			// If the auto step doesn't exist as a yaml definition, continue
			if (!(boost::algorithm::ends_with(auto_steps_[j], "csv"))) {
				if(!nh_.getParam(auto_steps_[j], action_data)) {
					continue;
				}
				// If the auto step's type is not a path, continue
				if (action_data["type"] != "path") {
					continue;
				}
			}

			ROS_INFO_STREAM("Now looking for path at " << auto_steps_[j]);
			
			if (auto_steps_[j].find("_csv") != std::string::npos) {
				// CSV format:
				// 0    1 2 3   4                5    6    7
				// time,x,y,yaw,angular_velocity,xvel,yvel,waypointIdx
				ROS_INFO_STREAM("auto_node : Loading Choreo path located at behaviors/path/" << auto_steps_[j] << ".csv");
				std::ifstream csv("/home/ubuntu/2023RobotCode/zebROS_ws/src/behaviors/path/" + auto_steps_[j] + ".csv");
				std::string line;
				std::vector<std::vector<double>> path;
				while (std::getline(csv, line)) {
					std::vector<std::string> v;
					std::vector<double> doubles;
					boost::split(v, line, boost::is_any_of(","));
					for (const std::string& s : v) {
						doubles.push_back(std::stod(s));
					}
					path.push_back(doubles);
				}

				nav_msgs::Path pos_path_msg;
				pos_path_msg.header.frame_id = "map";
				pos_path_msg.header.stamp = ros::Time(0);

				nav_msgs::Path vel_path_msg;
				vel_path_msg.header.frame_id = "map";
				vel_path_msg.header.stamp = ros::Time(0);
				nav_msgs::Path waypoints;
				std::vector<int> waypointsIdx;

				uint64_t i = 0;

				for (const auto &point : path) {
					geometry_msgs::PoseStamped pose;
					pose.header.frame_id = "map";
					pose.pose.position.x = point[1];
					pose.pose.position.y = point[2];
					tf2::Quaternion q;
					q.setRPY(0, 0, point[3]);
					pose.pose.orientation = tf2::toMsg(q);
					pose.header.stamp = ros::Time(point[0]);
					pose.header.seq = i;
					pos_path_msg.poses.push_back(pose);

					geometry_msgs::PoseStamped vel_pose;
					vel_pose.header.frame_id = "map";
					vel_pose.pose.position.x = point[5];
					vel_pose.pose.position.y = point[6];
					tf2::Quaternion velQ;
					velQ.setRPY(0, 0, point[4]);
					vel_pose.pose.orientation = tf2::toMsg(velQ);
					vel_pose.header.stamp = ros::Time(point[0]);
					vel_pose.header.seq = i;
					vel_path_msg.poses.push_back(vel_pose);

					i += 1;
					waypointsIdx.push_back(point[7]);
				}

				premade_position_paths_[auto_steps_[j]] = pos_path_msg;
				premade_velocity_paths_[auto_steps_[j]] = vel_path_msg;
				premade_position_waypoints_[auto_steps_[j]] = nav_msgs::Path();
				premade_velocity_waypoints_[auto_steps_[j]] = nav_msgs::Path();
				waypointsIdxs_[auto_steps_[j]] = waypointsIdx;
				ROS_INFO_STREAM("SETTING PREMADE PATH WITH " << auto_steps_[j]);
				if (premade_position_paths_.size() == 1) {
					first_point_pub_.publish(pos_path_msg.poses[0].pose);
				}
				continue;
			}
			// If the path has no goal, return
			if (!action_data.hasMember("goal"))
			{
				ROS_ERROR_STREAM("auto_node : path " << auto_steps_[j] << " has no 'goal' data");
				return false;
			}
			XmlRpc::XmlRpcValue path_goal = action_data["goal"];
			if (!action_data["goal"].hasMember("points"))
			{
				ROS_ERROR_STREAM("auto_node : path " << auto_steps_[j] << " has no points?");
				return false;
			}
			XmlRpc::XmlRpcValue points_config = path_goal["points"];

			// Generate the waypoints of the spline
			base_trajectory_msgs::GenerateSpline spline_gen_srv;
			const size_t point_num = points_config.size() + 1;
			spline_gen_srv.request.points.resize(point_num);
			spline_gen_srv.request.points[0].positions.resize(3);
			spline_gen_srv.request.points[0].positions[0] = 0;
			spline_gen_srv.request.points[0].positions[1] = 0;
			spline_gen_srv.request.points[0].positions[2] = 0;
			for (size_t i = 0; i < point_num-1; i++)
			{
				spline_gen_srv.request.points[i+1].positions.resize(3);
				if (!extractFloatVal(points_config[i][0], spline_gen_srv.request.points[i+1].positions[0]))
				{
					ROS_INFO_STREAM("Error converting path point[" << i << "].x to double");
					break;
				}
				if (!extractFloatVal(points_config[i][1], spline_gen_srv.request.points[i+1].positions[1]))
				{
					ROS_INFO_STREAM("Error converting path point[" << i << "].y to double");
					break;
				}
				if (!extractFloatVal(points_config[i][2], spline_gen_srv.request.points[i+1].positions[2]))
				{
					ROS_INFO_STREAM("Error converting path point[" << i << "].orientation to double");
					break;
				}
			}

			std::string frame_id;
			readStringParam("frame_id", path_goal, frame_id);

			spline_gen_srv.request.header.frame_id = frame_id;
			spline_gen_srv.request.header.stamp = ros::Time::now() - ros::Duration(2); // TODO -fixme

			bool optimize_final_velocity{false};
			readBoolParam("optimize_final_velocity", path_goal, optimize_final_velocity);
			spline_gen_srv.request.optimize_final_velocity = optimize_final_velocity;

			if (path_goal.hasMember("point_frame_id"))
			{
				XmlRpc::XmlRpcValue xml_point_frame_ids = path_goal["point_frame_id"];
				if (!xml_point_frame_ids.valid())
					throw std::runtime_error("point_frame_ids not valid");
				if (xml_point_frame_ids.getType() != XmlRpc::XmlRpcValue::TypeArray)
					throw std::runtime_error("point_frame_ids not an array");

				for (int i = 0; i < xml_point_frame_ids.size(); i++)
				{
					std::string point_frame_id = xml_point_frame_ids[i];
					spline_gen_srv.request.point_frame_id.push_back(point_frame_id);
				}
			}
			if (path_goal.hasMember("path_offset_limit"))
			{
				XmlRpc::XmlRpcValue xml_path_offset_limits = path_goal["path_offset_limit"];
				if (!xml_path_offset_limits.valid())
					throw std::runtime_error("path_offset_limits not valid");
				if (xml_path_offset_limits.getType() != XmlRpc::XmlRpcValue::TypeArray)
					throw std::runtime_error("path_offset_limits not an array");

				// Add empty offset limit for initial 0,0,0 waypoint
				spline_gen_srv.request.path_offset_limit.push_back(base_trajectory_msgs::PathOffsetLimit());
				for (int i = 0; i < xml_path_offset_limits.size(); i++)
				{
					base_trajectory_msgs::PathOffsetLimit path_offset_msg;
					readFloatParam("min_x", xml_path_offset_limits[i], path_offset_msg.min_x);
					readFloatParam("max_x", xml_path_offset_limits[i], path_offset_msg.max_x);
					readFloatParam("min_y", xml_path_offset_limits[i], path_offset_msg.min_y);
					readFloatParam("max_y", xml_path_offset_limits[i], path_offset_msg.max_y);

					spline_gen_srv.request.path_offset_limit.push_back(path_offset_msg);
				}
			}

			ROS_INFO_STREAM("auto_node : calling spline_gen_cli_ with " << spline_gen_srv.request);
			if (!spline_gen_cli_.call(spline_gen_srv))
			{
				ROS_ERROR_STREAM("Can't call spline gen service in path_follower_server");
				return false;
			}
			premade_position_paths_[auto_steps_[j]] = spline_gen_srv.response.path;
			premade_position_waypoints_[auto_steps_[j]] = spline_gen_srv.response.position_waypoints;
			premade_velocity_paths_[auto_steps_[j]] = spline_gen_srv.response.path_velocity;
			premade_velocity_waypoints_[auto_steps_[j]] = spline_gen_srv.response.velocity_waypoints;
			waypointsIdxs_[auto_steps_[j]] = spline_gen_srv.response.waypointsIdx;
		}
		return true;
	}
	
	bool waitForAutoStart(ros::NodeHandle nh_)
	{
		ros::spinOnce();
		// In sim, time starts at 0. We subtract 2 seconds from the currentt time
		// when fetching transforms to make sure they've had a chance to be published
		// Make sure we don't ever use a time less than 0 because of this by skipping
		// a few seconds here
		ros::Duration(2.5).sleep();

		// preload auto mode 90 paths
		auto_mode_ = 90;
		if(nh_.getParam("auto_mode_" + std::to_string(auto_mode_), auto_steps_)) {
			if (!preLoadPath()) {
				return false;
			}
			ROS_INFO_STREAM_THROTTLE(1, "auto_node: mode auto_mode_" + std::to_string(auto_mode_));
		} 

		//wait for auto period to start
		while( ros::ok() && !auto_stopped_ )
		{
			ros::spinOnce(); //spin so the subscribers can update
			//read sequence of actions from config
			if (auto_mode_ >= 0)
			{
				if(nh_.getParam("auto_mode_" + std::to_string(auto_mode_), auto_steps_)) {
					if (!preLoadPath()) {
						return false;
					}
					ROS_INFO_STREAM_THROTTLE(1, "auto_node: mode auto_mode_" + std::to_string(auto_mode_));
				} 
				else {
					ROS_ERROR_STREAM_THROTTLE(0.5, "COULD NOT FIND AUTO MODE PARAM?");
				}
			}
			if(auto_mode_ > 0){
				auto_state_ = READY;
			}
			if(auto_started_ && auto_mode_ <= 0){
				ROS_ERROR("Auto node - Autonomous period started, please choose an auto mode");
			}
			// Valid auto mode plus auto_started_ flag ==> actually run auto code, return success
			if (auto_started_ && (auto_mode_ > 0)) {
				return true;
			}
			r_.sleep();
		}

		// shutdownNode(DONE, "Auto node - code stopped before execution");
		return false;
	}

	bool resetMaps(std_srvs::Empty::Request &/*req*/,
				std_srvs::Empty::Response &/*res*/) {

		premade_position_paths_.clear();
		premade_velocity_paths_.clear();
		ROS_INFO_STREAM("premade paths were cleared");
		return true;
	}

	// Called everytime feedback is published
	void feedbackCb(const path_follower_msgs::PathFeedbackConstPtr& feedback) {
		// ROS_INFO_STREAM("Got path_follower feedback!");


		//ROS_INFO("Total Percent complete %f", (feedback->percent_complete));
		//ROS_INFO_STREAM("Current Waypoint " << (feedback->current_waypoint));
		//ROS_INFO("Waypoint percent %f", (feedback->percent_next_waypoint));
		
		// Can also add diffrent conditions based on auto mode
		// TODO, add parsing for responses based on feedback to auto_mode_config

		// current_waypoint exists to compare against the old value and see if it has changed
		int current_waypoint = feedback->current_waypoint;
		double current_percent_complete = feedback->percent_complete;
		double current_waypoint_percent = feedback->percent_next_waypoint;

		//ROS_INFO_STREAM("New waypoint is " << current_waypoint);
		//ROS_INFO_STREAM("Old waypoint is " << old_waypoint);

		// Add conditions below when waypoint changes
		// Checks if second waypoint
		if (current_waypoint > old_waypoint_) {
			if (current_waypoint == 2) {
				ROS_INFO_STREAM("Hit waypoint 2! Time to do stuff!");

			}
		}

		
		// Add conditions based on percent's and waypoint below
		// Checks if it is the first time over 50% completed and on waypoint 1
		if (current_waypoint_percent >= 0.5 && old_waypoint_percent_ < 0.5 && current_waypoint == 1) {
			ROS_INFO_STREAM("50 percent done with first waypoint!");

		} 


		old_waypoint_percent_ = current_waypoint_percent;
		old_percent_complete_ = current_percent_complete;
		old_waypoint_ = current_waypoint;
	}

	// CODE FOR ACTIONS HERE --------------------------------------------------
	// Do not name this function "pause" will not work
	bool pausefn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		const double start_time = ros::Time::now().toSec();

		//read duration - user could've entered a double or an int, we don't know which
		double duration;
		if (!readFloatParam("duration", action_data, duration))
		{
			shutdownNode(ERROR, "Auto node - duration is not a double or int in pause action");
			return false;
		}
		#if 0
		if((action_data["duration"].getType() == XmlRpc::XmlRpcValue::Type::TypeDouble) ||
			(action_data["duration"].getType() == XmlRpc::XmlRpcValue::Type::TypeInt) ) {
			duration = static_cast<double>(action_data["duration"]);
		} else {
			shutdownNode(ERROR, "Auto node - duration is not a double or int in '" + auto_step + "' action");
			return 1;
		}
		#endif

		//wait
		while (ros::Time::now().toSec() - start_time < duration && !auto_stopped_ && ros::ok())
		{
			ros::spinOnce();
			r_.sleep();
		}
		return true;
	}

	bool alignToSpeakerfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		if(!align_to_speaker_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR,"Auto node - couldn't find align to speaker actionlib server");
			return false;
		}

		behavior_actions::AlignToSpeaker2024Goal goal;
		goal.align_forever = false;

		align_to_speaker_ac_.sendGoal(goal);
		waitForActionlibServer(align_to_speaker_ac_, 1.5, "align_to_speaker");

		return true;
	}

	bool spinUpShooterfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		ROS_INFO_STREAM("Auto node spinning up shooter!");
		if(!shooting_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR,"Auto node - couldn't find intaking shooting server");
			return false;
		}
		double distance;

		if (!action_data.hasMember("distance"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'distance' field");
			return false;
		}
		if (!readFloatParam("distance", action_data, distance))
		{
			shutdownNode(ERROR, "Auto node - distance is not a double or int in pause action");
			return false;
		}
		ROS_INFO_STREAM("Auto node - Spinning up shooter with distance " << distance);
		behavior_actions::Shooting2024Goal goal;
		goal.leave_spinning = true; // keep spinning after 
		goal.setup_only = true; // don't actually shoot anything
		goal.distance = distance;
		goal.mode = goal.SPEAKER;
		if (action_data.hasMember("mode")) {
			if (action_data["mode"] == "slide") {
				ROS_INFO_STREAM("Auto node - Spinning up shooter for slide");
				goal.mode = goal.SLIDE;
			}
		}
		shooting_ac_.sendGoal(goal); // this goal will succeed almost right away, don't cancel
		return true; 
	}

	bool shootDistancefn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		if(!shooting_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR,"Auto node - couldn't find shooting server");
			return false;
		}
		double distance;
		if (!action_data.hasMember("distance"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'distance' field");
			return false;
		}
		if (!readFloatParam("distance", action_data, distance))
		{
			shutdownNode(ERROR, "Auto node - distance is not a double or int in pause action");
			return false;
		}

		ROS_INFO_STREAM("Auto node - SHOOTING with distance " << distance);
		behavior_actions::Shooting2024Goal goal;
		goal.leave_spinning = true; // want to keep going after this shot
		goal.setup_only = false; // actually shoot something
		goal.distance = distance; 
		if (distance == 0) {
			goal.distance = speaker_dist;
			ROS_INFO_STREAM("Auto node - shooting from actual distance " << speaker_dist);
		}
		goal.mode = goal.SPEAKER;
		if (action_data.hasMember("mode")) {
			if (action_data["mode"] == "slide") {
				ROS_INFO_STREAM("Auto node - shooting slide shot");
				goal.mode = goal.SLIDE;
			}
		}
		shooting_ac_.sendGoal(goal); 
		
		waitForActionlibServer(shooting_ac_, 10, "Shooting!");
		return true;
	}
	
	bool spinDownShooterfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		if(!shooting_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR,"Auto node - couldn't find shooting server");
			return false;
		}
		ROS_WARN_STREAM("Auto Node - shooter spinning DOWN");
		behavior_actions::Shooting2024Goal goal;
		goal.cancel_movement = true;
		shooting_ac_.sendGoal(goal); // this goal will succeed almost right away, don't cancel
		// shooting_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
		return true;
	}

	bool startIntakingfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		if(!intaking_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR,"Auto node - couldn't find intaking actionlib server");
			return false;
		}

		behavior_actions::Intaking2024Goal goal;
		goal.destination = goal.SHOOTER;
		goal.run_until_preempt = true;

		intaking_ac_.sendGoal(goal); // just send and can cancel in another action
		ROS_INFO_STREAM("Auto node - Starting intake");

		// make a std_srvs::Empty request
		std_srvs::Empty srv;
		// call the relocalize service
		if (!tagslam_relocalize_srv_.call(srv)) {
			shutdownNode(ERROR, "Auto node - relocalize service call failed");
			return false;
		}

		return true;
	}

	bool stopIntakingfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		if(!intaking_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR,"Auto node - couldn't find intaking actionlib server");
			return false;
		}

		intaking_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now()); // cancel goal sent from start intaking, probably never needed but could be helpful in a pinch
		ROS_WARN_STREAM("Auto node - STOPPING INTAKE in auto");
		return true;
	}

	bool rezeroOdomfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {

		intaking_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now()); // cancel goal sent from start intaking, probably never needed but could be helpful in a pinch
		ROS_WARN_STREAM("Auto node - STOPPING INTAKE in auto");
		return true;
	}

	bool counterAutofn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		// auto_mode_12: ["zoom_mid_ALLIANCE_csv", "spin_up_slide_shooter", "auto_intake", "closest_note_path"]
		// spin up shooter slide async
		// start zoom to mid
		// auto intake
		// closest note path forever 
		
		// spin up shooter slide async
		runStep("spin_up_slide_shooter");
		
		// zoom to mid
		std::string path = "zoom_mid_source_ALLIANCE_csv";
		const std::string ALLIANCE = "ALLIANCE"; 
		boost::replace_all(path, ALLIANCE, getAllianceColorString());
		runStep(path);

		ros::Duration(0.25).sleep();

		// auto intake
		std::string auto_intake = "auto_intake_first_ALLIANCE";
		boost::replace_all(auto_intake, ALLIANCE, getAllianceColorString());
		runStep(auto_intake);

		ros::Time start_time = ros::Time::now();
		while (auto_intake_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && (ros::Time::now() - start_time).toSec() < 10) {
			ros::spinOnce();
			r_.sleep();
		}
		waitForActionlibServer(auto_intake_ac_, 10, "auto intaking first note");

		// closest note path forever
		path = "one_to_two_ALLIANCE_csv";
		boost::replace_all(path, ALLIANCE, getAllianceColorString());
		runStep(path);

		start_time = ros::Time::now();
		while (auto_intake_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && (ros::Time::now() - start_time).toSec() < 10) {
			ros::spinOnce();
			r_.sleep();
		}
		waitForActionlibServer(auto_intake_ac_, 10, "auto intaking second note");

		path = "two_to_three_ALLIANCE_csv";
		boost::replace_all(path, ALLIANCE, getAllianceColorString());
		runStep(path);

		start_time = ros::Time::now();
		while (auto_intake_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && (ros::Time::now() - start_time).toSec() < 10) {
			ros::spinOnce();
			r_.sleep();
		}
		waitForActionlibServer(auto_intake_ac_, 10, "auto intaking third note");

		path = "three_to_four_ALLIANCE_csv";
		boost::replace_all(path, ALLIANCE, getAllianceColorString());
		runStep(path);

		start_time = ros::Time::now();
		while (auto_intake_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && (ros::Time::now() - start_time).toSec() < 10) {
			ros::spinOnce();
			r_.sleep();
		}
		waitForActionlibServer(auto_intake_ac_, 10, "auto intaking fourth note");

		runStep("shoot_slide");

		// runStep("closest_note_path");
		// waitForActionlibServer(auto_intake_ac_, 10, "auto intaking");

		return true;
	}

	void autoIntakeFeedback(const behavior_actions::DriveObjectIntake2024FeedbackConstPtr& feedback) {
		ROS_INFO_STREAM("GOT DRIVE TO OBJECT FEEDBACK");
		if (feedback->tracking_obj && !have_canceled) {
			ROS_INFO_STREAM("TRACKING OBJ, KILLING PATH FOLLOWER");
			// need to kill path follower yesterday
			path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());
			have_canceled = true;
		}
		else {
			ROS_INFO_STREAM("Auto node: Have not seen object");
		}	
	}

	bool autoIntakefn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		if(!auto_intake_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR,"Auto node - couldn't find auto intake actionlib server");
			return false;
		}

		behavior_actions::DriveObjectIntake2024Goal goal;
		goal.destination = goal.SHOOTER;
		goal.drive_timeout = 20;

		double min_y_pos = 0;
		if (!action_data.hasMember("min_y_pos"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'distance' field");
		}
		else if (!readFloatParam("min_y_pos", action_data, min_y_pos))
		{
			shutdownNode(ERROR, "Auto node - min_y_pos is not a double or int in pause action");
			return false;
		}

		double max_y_pos = 0;
		if (!action_data.hasMember("max_y_pos"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'distance' field");
		}
		else if (!readFloatParam("max_y_pos", action_data, max_y_pos))
		{
			shutdownNode(ERROR, "Auto node - max_y_pos is not a double or int in pause action");
			return false;
		}

		goal.min_y_pos = min_y_pos;
		goal.max_y_pos = max_y_pos;

		have_canceled = false;
		auto_intake_ac_.sendGoal(goal, NULL, NULL, boost::bind(&AutoNode::autoIntakeFeedback, this, _1));
		ROS_INFO_STREAM("Auto node - Starting intake");

		// preempt path follower (cancel action clien
		return true;
	}

	bool waitForActionfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		// Read the action to wait for from action_data
		// It can be "auto_intake" or "shooting"
		std::string action_to_wait_for;
		if (!action_data.hasMember("action"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'action' field");
			return false;
		}
		if (!readStringParam("action", action_data, action_to_wait_for))
		{
			shutdownNode(ERROR, "Auto node - action is not a string in wait for action");
			return false;
		}

		// Wait for the action to finish
		if (action_to_wait_for == "auto_intake") {
			// Wait for client to be active first but not succeeded with a timeout of 10 seconds
			ros::Time start_time = ros::Time::now();
			while (auto_intake_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && (ros::Time::now() - start_time).toSec() < 10) {
				ros::spinOnce();
				r_.sleep();
			}
			waitForActionlibServer(auto_intake_ac_, 10, "auto intaking");
		} else if (action_to_wait_for == "shooting") {
			ros::Time start_time = ros::Time::now();
			while (shooting_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && (ros::Time::now() - start_time).toSec() < 10) {
				ros::spinOnce();
				r_.sleep();
			}
			waitForActionlibServer(shooting_ac_, 10, "shooting");
		} else {
			ROS_ERROR_STREAM("Auto node - unknown action to wait for: " << action_to_wait_for);
			return false;
		}
		
		return true;
	}

	bool relocalizefn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		// make a std_srvs::Empty request
		std_srvs::Empty srv;
		// call the relocalize service
		if (!tagslam_relocalize_srv_.call(srv)) {
			shutdownNode(ERROR, "Auto node - relocalize service call failed");
			return false;
		}

		ROS_INFO_STREAM("Auto node - relocalize service call succeeded");
		return true;
	}

	bool pausePathfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		// make a std_srvs::SetBool request
		std_srvs::SetBool srv;
		bool pause;
		if (!action_data.hasMember("pause"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'pause' field");
			return false;
		}
		if (!readBoolParam("pause", action_data, pause))
		{
			shutdownNode(ERROR, "Auto node - pause is not a bool in pause path action");
			return false;
		}

		srv.request.data = pause;

		// call the pause service
		if (!pause_srv_.call(srv)) {
			shutdownNode(ERROR, "Auto node - pause path service call failed");
			return false;
		}

		ROS_INFO_STREAM("Auto node - pause path service call succeeded");
		return true;
	}

	bool snapanglefn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) { 
		double angle_in_rad;
		if (!action_data.hasMember("angle"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'duration' field");
			return false;
		}
		if (!readFloatParam("angle", action_data, angle_in_rad))
		{
			shutdownNode(ERROR, "Auto node - duration is not a double or int in pause action");
			return false;
		}

		std_msgs::Float64 target_angle_msg;
		target_angle_msg.data = angle_in_rad;
		orient_command_pub_.publish(target_angle_msg);
		ros::Rate rate(25);
		rate.sleep(); // just to give orientation control time to update current_orient_effort_
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.z = 0;
		cmd_vel.linear.x = 0;
		cmd_vel.linear.y = 0;
		cmd_vel.angular.x = 0;
		cmd_vel.angular.y = 0;
		ros::spinOnce();

		while (fabs(angles::shortest_angular_distance(angle_in_rad, current_yaw_)) > angle_tolerance_) {
			ROS_INFO_STREAM_THROTTLE(2, "Current yaw (deg) = " << angles::to_degrees(current_yaw_) << " target angle (deg) = " << angles::to_degrees(angle_in_rad) << " current effort = " << current_orient_effort_);
			orient_command_pub_.publish(target_angle_msg);
			cmd_vel.angular.z = current_orient_effort_;
			cmd_vel_pub_.publish(cmd_vel);
			ros::spinOnce(); 	
			rate.sleep();
		}
		ROS_INFO_STREAM("Auto node : Done with snap - Current yaw (deg) = " << angles::to_degrees(current_yaw_) << " target angle (deg) = " << angles::to_degrees(angle_in_rad) << " current effort = " << current_orient_effort_);
		return true;
	}

	int last_waypoint = -1;
	std::map<int, std::vector<std::string>> waypoint_actions;

	bool pathfn(XmlRpc::XmlRpcValue action_data, const std::string&  auto_step) {
		if(!path_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR, "Couldn't find path server");
			return false;
		}
		path_ac_.cancelGoalsAtAndBeforeTime(ros::Time::now() - ros::Duration(0.1));
		int iteration_value = 1;
		bool wait_for_path = true;
		if (action_data.hasMember("goal"))
			// could this fail?
			readIntParam("iterations", action_data["goal"], iteration_value);
		
		if (action_data.hasMember("wait_for_path"))
			readBoolParam("wait_for_path", action_data, wait_for_path);

		bool reset_odom_to_path_start = false; 
		if (action_data.hasMember("reset_odom_to_path_start"))
			readBoolParam("reset_odom_to_path_start", action_data, reset_odom_to_path_start);
		
		waypoint_actions.clear();
		if (action_data.hasMember("waypoint_actions")) {
			XmlRpc::XmlRpcValue wpas = action_data["waypoint_actions"];
			ROS_INFO_STREAM("Waypoint actions exist! Type is " << wpas.getType() << " iterations value " << iteration_value);
			if (wpas.getType() == wpas.TypeArray) {
				for (int i = 0; i < wpas.size(); i++) {
					std::vector<std::string> actions_at_waypoint;
					if (wpas[i].getType() == wpas[i].TypeArray) {
						for (int j = 1; j < wpas[i].size(); j++) {
							actions_at_waypoint.push_back(wpas[i][j]);
							ROS_INFO_STREAM("ADDING action at waypoint " << wpas[i][j]);
						}
						waypoint_actions[wpas[i][0]] = actions_at_waypoint;
						// ROS_INFO_STREAM("Auto node: adding waypoint action " << actions_at_waypoint);
					}
				}
			}
		}
		else {
			ROS_WARN_STREAM("Could not find a waypoints action, this may be intentional!");
		}

		while(iteration_value > 0)
		{
			path_follower_msgs::PathGoal goal;
			if (premade_position_paths_.find(auto_step) == premade_position_paths_.end()) {
				shutdownNode(ERROR, "1221: Can't find premade path " + auto_step);
			}

			goal.position_path = premade_position_paths_[auto_step];
			goal.position_waypoints = premade_position_waypoints_[auto_step];
			goal.velocity_path = premade_velocity_paths_[auto_step];
			goal.velocity_waypoints = premade_velocity_waypoints_[auto_step];
			goal.waypointsIdx = waypointsIdxs_[auto_step];
			goal.dont_go_to_start = reset_odom_to_path_start;

			last_waypoint = -1;
			if (reset_odom_to_path_start) {
				geometry_msgs::PoseStamped first_pose = goal.position_path.poses[0];
				ROS_WARN_STREAM("FORCING RELOCALZE TO THE START OF THE PATH======= AT POINT X,Y " << first_pose.pose.position.x << ", " << first_pose.pose.position.y);
				behavior_actions::RelocalizePoint inital_point;
				inital_point.request.pose.orientation = latest_imu_.orientation;
				inital_point.request.pose.position.x = first_pose.pose.position.x;
				inital_point.request.pose.position.y = first_pose.pose.position.y;
				if (!relocalize_point_srv_.call(inital_point)) {
					ROS_ERROR_STREAM("FAILED TO CALL RELOCALIZE POINT - EXITING");
					shutdownNode(ERROR, "Map to odom service call failed");
				}
				ROS_INFO_STREAM("SETTING RELOCALIZE TO FALSE");
				std_srvs::SetBool toggle_relocalize;
				toggle_relocalize.request.data = false;
				if (!toggle_relocalize_srv_.call(toggle_relocalize)) {
					ROS_ERROR_STREAM("FAILED TO DISABLE RELOCALIZING WITH CMD_VEL - EXITING");
					shutdownNode(ERROR, "A Map to odom service call failed");
				}
				ROS_INFO_STREAM("REZEROING ODOMETRY SO PATH WORKS AS EXPECTED");
				std_srvs::Empty odom_reset; 
				if (!zero_odom_srv_.call(odom_reset)) {
					ROS_ERROR_STREAM("FAILED TO REZERO ODOM");
					shutdownNode(ERROR, "Rezeroing odom before odom/map relative path failed");
				}
				ros::Duration(0.05).sleep();
			} 

			ROS_INFO_STREAM("Auto node - sending path goal for path " << auto_step);
			path_ac_.sendGoal(goal, NULL, NULL, [&](const path_follower_msgs::PathFeedbackConstPtr& feedback){
				int waypoint = feedback->current_waypoint;
				if (last_waypoint != waypoint) {
					ROS_INFO_STREAM("CHANGED**** WAYPOINT " << std::to_string(waypoint) << " ****");
					if (waypoint_actions.find(waypoint) == waypoint_actions.end()) {
						// not found
						ROS_ERROR_STREAM("No waypoint action");
					} else {
						ROS_ERROR_STREAM("FOUND WAYPOINT ACTION len of waypoint_actions[waypoint] " << waypoint_actions[waypoint].size());
						// found
						for (std::string action_name : waypoint_actions[waypoint]) {
							ROS_INFO_STREAM("********** WAYPOINT ACTION " << action_name << " EXISTS!!!");
							ROS_INFO_STREAM("Running action " << action_name);
							runStep(action_name);
						}
					}
				}
				last_waypoint = waypoint;
			});
			ROS_INFO_STREAM("Auto node - sent path goal");

			// wait for actionlib server to finish
			if (wait_for_path) {
				
				waitForActionlibServer(path_ac_, 100, "running path");
			}
			iteration_value--;
		}
		return true;
	}

	bool runClosestPathfn(XmlRpc::XmlRpcValue action_data, const std::string& auto_step) {
		// Takes a list of paths in action_data["paths"]
		// Runs the path with a starting point that is closest to the robot's current position

		if(!path_ac_.waitForServer(ros::Duration(5))){
			shutdownNode(ERROR, "Couldn't find path server");
			return false;
		}

		// Get the list of paths
		if (!action_data.hasMember("paths")) {
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'paths' field");
			return false;
		}

		XmlRpc::XmlRpcValue paths_ = action_data["paths"];
		if (paths_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
			ROS_ERROR_STREAM("Auto action " << auto_step << " 'paths' field is not an array");
			return false;
		}

		// Replace "ALLIANCE" with "red" or "blue" depending on the alliance color
		std::vector<std::string> paths;
		for (int i = 0; i < paths_.size(); i++) {
			std::string path = static_cast<std::string>(paths_[i]);
			const std::string ALLIANCE = "ALLIANCE"; 
			boost::replace_all(path, ALLIANCE, getAllianceColorString());
			paths.push_back(path);
		}

		// Get the robot's current position using the tf buffer
		// This is the transform from the map frame to the base_link frame
		geometry_msgs::TransformStamped transformStamped;
		try {
			transformStamped = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
		} catch (tf2::TransformException &ex) {
			ROS_ERROR_STREAM("Auto node - " << ex.what());
			return false;
		}
		geometry_msgs::PoseStamped current_pose;
		current_pose.pose.position.x = transformStamped.transform.translation.x;
		current_pose.pose.position.y = transformStamped.transform.translation.y;

		// Find the path that is closest to the robot's current position
		double closest_distance = std::numeric_limits<double>::max();
		std::string closest_path;
		for (int i = 0; i < paths.size(); i++) {
			std::string path = static_cast<std::string>(paths[i]);
			if (premade_position_paths_.find(path) == premade_position_paths_.end()) {
				shutdownNode(ERROR, "Can't find premade path " + path);
			}
			nav_msgs::Path path_msg = premade_position_paths_[path];
			geometry_msgs::PoseStamped first_pose = path_msg.poses[0];
			double distance = hypot(first_pose.pose.position.x - current_pose.pose.position.x,
									first_pose.pose.position.y - current_pose.pose.position.y);
			if (distance < closest_distance) {
				closest_distance = distance;
				closest_path = path;
			}
		}

		nav_msgs::Path path_msg = premade_position_paths_[closest_path];
		geometry_msgs::PoseStamped first_pose = path_msg.poses[0];
		// Log selected path
		ROS_INFO_STREAM("Auto node - Closest path is " << closest_path << " relocalizing to point " << first_pose.pose.position.x << ", " << first_pose.pose.position.y);
		// TODO, add reloclaize 
		runStep(closest_path);

		return true;
	}

	// now ignores z and will just hold whatever orientation it starts in
	bool cmdvelfn(XmlRpc::XmlRpcValue action_data, const std::string&  auto_step) {
		#if 0 // doesn't work in sim
		if(!brake_srv_.waitForExistence(ros::Duration(15)))
		{
			ROS_ERROR("Wait (15 sec) timed out, for Brake Service in auto_node");
			return false;
		}
		#endif

		double duration_secs;
		
		// TODO find a nicer way to check for errors
		if (!action_data.hasMember("duration"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'duration' field");
			return false;
		}
		if (!readFloatParam("duration", action_data, duration_secs))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " error reading 'duration' field");
			return false;
		}
		if (!action_data.hasMember("cmd_vel"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing 'cmd_vel' field");
			return false;
		}
		XmlRpc::XmlRpcValue cmd_vel_data = action_data["cmd_vel"];
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.z = 0;
		cmd_vel.angular.x = 0;
		cmd_vel.angular.y = 0;

		if (!cmd_vel_data.hasMember("x"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing cmd_vel 'x' field");
			return false;
		}
		if (!readFloatParam("x", cmd_vel_data, cmd_vel.linear.x))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " error reading cmd_vel 'x' field");
			return false;
		}
		if (!cmd_vel_data.hasMember("y"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing cmd_vel 'y' field");
			return false;
		}
		if (!readFloatParam("y", cmd_vel_data, cmd_vel.linear.y))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " error reading cmd_vel 'y' field");
			return false;
		}
		if (!cmd_vel_data.hasMember("z"))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " missing cmd_vel 'z' field");
			return false;
		}
		if (!readFloatParam("z", cmd_vel_data, cmd_vel.angular.z))
		{
			ROS_ERROR_STREAM("Auto action " << auto_step << " error reading cmd_vel 'z' field");
			return false;
		}
		ROS_INFO_STREAM("Auto action cmd_vel x = " << cmd_vel.linear.x <<
				" y = " << cmd_vel.linear.y <<
				" z = " << cmd_vel.angular.z <<
				" for " << duration_secs << " seconds");

		const ros::Duration duration(duration_secs);
		const ros::Time start_time = ros::Time::now();
		ros::Rate rate(25);
		double starting_rotation = current_yaw_;
		std_msgs::Float64 orient_msg;
		orient_msg.data = starting_rotation;
		while (ros::ok() && !auto_stopped_ && ((ros::Time::now() - start_time) < duration))
		{
			orient_command_pub_.publish(orient_msg); // need to do this to keep control of rotation
			cmd_vel.angular.z = current_orient_effort_;
			cmd_vel_pub_.publish(cmd_vel);
			ros::spinOnce();
			rate.sleep();

		}
		cmd_vel.linear.x = 0;
		cmd_vel.linear.y = 0;
		cmd_vel.angular.z = 0;
		cmd_vel_pub_.publish(cmd_vel);
		std_srvs::Empty empty;
		if (!brake_srv_.call(empty))
		{
			ROS_ERROR_STREAM("BrakeSrv call failed in auto cmd_vel step " << auto_step);
		}
		ROS_INFO_STREAM("Auto action " << auto_step << " finished");
		return true;
	}

	int runStep(const std::string &_name) {
		const std::string ALLIANCE = "ALLIANCE"; 
		std::string name = _name;
		// boost::replace_all(name, ALLIANCE, getAllianceColorString());
		
		//read data from config needed to carry out the action
		XmlRpc::XmlRpcValue action_data;
		if(! nh_.getParam(name, action_data)){
			//shutdownNode(ERROR, "Auto node - Couldn't read data for '" + auto_steps_[i] + "' auto action from config file");
			//return 1;
		}

		//figure out what to do based on the action type, and do it
		std::string action_data_type;
		if (action_data.hasMember("type"))
		{
			action_data_type = static_cast<std::string>(action_data["type"]);
				ROS_INFO_STREAM("auto_node: Running " << action_data_type);
			// amazing syntax 
			// passes in the config data and which auto step is running
			bool result = (this->*functionMap_[action_data_type])(action_data, std::string(name));
			if (!result)
			{
				std::string error_msg = "Auto node - Error running auto action " + name;
				ROS_ERROR_STREAM(error_msg);
				shutdownNode(ERROR, error_msg);
				return 1;
			}
		}
		// check if we are a csv, in which case we do know what the type shoudl be and can continue
		else if (boost::algorithm::ends_with(name, "csv")) {
			ROS_WARN_STREAM("Assuming " << name << " is a path and setting fields based on it");
			action_data_type = "path";
			bool result = (this->*functionMap_[action_data_type])(action_data, std::string(name));
			if (!result)
			{
				std::string error_msg = "Auto node - Error running auto action " + name;
				ROS_ERROR_STREAM(error_msg);
				shutdownNode(ERROR, error_msg);
				return 1;
			}
		}
		else
		{
			ROS_ERROR_STREAM("Data for action " << name << " missing 'type' field");
			return 1;
		}
		return 0;
	}

	int init()
	{	
		if (!spline_gen_cli_.waitForExistence(ros::Duration(5.0)))
		{
			ROS_ERROR("Wait (15 sec) timed out, for Spline Gen Service in auto_node");
		}

		while(ros::ok()) { // will exit when shutdownNode is called
			//WAIT FOR MATCH TO START --------------------------------------------------------------------------
			ROS_INFO("Auto node - waiting for autonomous to start");

			//wait for auto period to start
			if (!waitForAutoStart(nh_))
				continue;

			//EXECUTE AUTONOMOUS ACTIONS --------------------------------------------------------------------------

			ROS_INFO_STREAM("Auto node - Executing auto mode " << auto_mode_);
			auto_state_ = RUNNING;

			auto_steps_.clear(); //stores string of action names to do, read from the auto mode array in the config file
			//read sequence of actions from config
			if(!nh_.getParam("auto_mode_" + std::to_string(auto_mode_), auto_steps_)){
				shutdownNode(ERROR, "Couldn't read " + ("auto_mode_" + std::to_string(auto_mode_) + " config value in auto node"));
				return 1;
			}

			//run through actions in order
			for(size_t i = 0; i < auto_steps_.size(); i++){
				std::string color_str = getAllianceColorString(); 
				if (color_str == "") {
					shutdownNode(ERROR, "Alliance string is none");
				}
				const std::string ALLIANCE = "ALLIANCE"; 
				boost::replace_all(auto_steps_[i], ALLIANCE, color_str); 
				if(auto_started_ && !auto_stopped_)
				{
					ROS_INFO_STREAM("Auto node - running step " << i << ": " << auto_steps_[i]);

					int result = runStep(auto_steps_[i]);
					if (result != 0) {
						return result;
					}

				}
			}

			auto_state_ = DONE;
			ROS_INFO_STREAM("auto_node completed, waiting for auto to end");
			if (!waitForAutoEnd()) {
				shutdownNode(ERROR, "ROS is not ok :(");
			} else {
				auto_state_ = READY;
			}
		}
		return 0;
	}
};



int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "auto_node");
	ros::NodeHandle nh;
	AutoNode autonode(nh);
	autonode.init();
	return 0;
}