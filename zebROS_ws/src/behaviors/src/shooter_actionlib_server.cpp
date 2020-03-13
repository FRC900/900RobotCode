#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <vector>
#include <functional>
#include <map>
#include <string>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/ShooterAction.h"
#include "behavior_actions/IndexerAction.h"
#include "behavior_actions/AlignToShootAction.h"

#include "behavior_actions/enumerated_indexer_actions.h"

//include controller service files and other service files
#include "controllers_2020_msgs/ShooterSrv.h"
#include "controllers_2020_msgs/IndexerSrv.h"

//include msg types
#include "field_obj/Detection.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "behavior_actions/ShooterOffset.h"


bool sortDistDescending(const std::map<std::string, double> &m1, const std::map<std::string, double> &m2)
{
	return m1.at("dist") > m2.at("dist");
}


//create the class for the actionlib server
class ShooterAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::ShooterAction> as_; //create the actionlib server
		std::string action_name_;

		//clients to call controllers
		ros::ServiceClient shooter_client_; //create a ros client to send requests to the controller

		//publisher to turn green light on/off
		ros::Publisher green_light_pub_;

		//clients to call actionlib server
		actionlib::SimpleActionClient<behavior_actions::AlignToShootAction> ac_align_;
		actionlib::SimpleActionClient<behavior_actions::IndexerAction> ac_indexer_;

		//subscribing stuff
		ros::Subscriber ready_to_shoot_sub_;
		std::atomic<bool> ready_to_shoot_{false};

		ros::Subscriber goal_sub_;
		std::mutex goal_msg_mutex_;
		field_obj::Detection goal_msg_;

		ros::Subscriber shooter_offset_sub_;
		std::atomic<double> speed_offset_{0};

		std::thread in_range_pub_thread_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

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

		double lerp(double a, double b, double f)
		{
			return a + f * (b - a);
		}

		double lerpTable(const std::vector<std::map<std::string, double>> &table, double dist)
		{
			size_t counter = 1;
			for (; counter < table.size()-1; counter++)
				if(table.at(counter).at("dist") < dist)
					break;
			return lerp(table.at(counter).at("speed"), table.at(counter-1).at("speed"), (dist-table.at(counter).at("dist"))/(table.at(counter-1).at("dist")-table.at(counter).at("dist")));
		}

		void turnGreenLightOn(bool turn_on)
		{
			/* commented out b/c we want to leave the light on all the time for now
			std_msgs::Float64 msg;
			if(turn_on) {
				msg.data = 1.0;
			}
			else {
				msg.data = 0.0;
			}
			green_light_pub_.publish(msg);
			*/
		}

		bool getHoodAndVelocity(bool& hood_extended, double& shooter_speed)
		{
			field_obj::Detection local_goal_msg;
			std::lock_guard<std::mutex> l(goal_msg_mutex_);
			local_goal_msg = goal_msg_;

			//get the goal position
			bool found_goal = false;
			geometry_msgs::Point32 goal_pos_;
			for (const field_obj::Object &obj : local_goal_msg.objects)
			{
				if(obj.id == "power_port")
				{
					goal_pos_ = obj.location;
					found_goal = true;
				}
			}

			if(found_goal)
			{
				// find transformed goal position
				geometry_msgs::PointStamped goal_pos_from_zed;
				goal_pos_from_zed.header = local_goal_msg.header;
				goal_pos_from_zed.point.x = goal_pos_.x;
				goal_pos_from_zed.point.y = goal_pos_.y;
				goal_pos_from_zed.point.z = goal_pos_.z;

				geometry_msgs::PointStamped transformed_goal_pos;
				geometry_msgs::TransformStamped zed_to_turret_transform;
				try {
					zed_to_turret_transform = tf_buffer_.lookupTransform("turret_center", "zed_camera_center", ros::Time::now());
					tf2::doTransform(goal_pos_from_zed, transformed_goal_pos, zed_to_turret_transform);
				}
				catch (tf2::TransformException &ex) {
					ROS_WARN("Shooter actionlib server failed to do ZED->turret transform - %s", ex.what());
					return false;
				}
				ROS_INFO_STREAM("shooter server - original goal_pos: (" << goal_pos_.x << ", " << goal_pos_.y << ", " << goal_pos_.z << ")");
				ROS_INFO_STREAM("shooter server - transformed goal_pos: (" << transformed_goal_pos.point.x << ", " << transformed_goal_pos.point.y << ", " << transformed_goal_pos.point.z << ")");

				//obtain distance via trig
				const double distance = std::hypot(transformed_goal_pos.point.x, transformed_goal_pos.point.y);

				if(distance > max_dist_)
				{
					ROS_ERROR_STREAM("Shooter server - farther than max dist (" << max_dist_ << "), can't shoot");
					return false;
				}
				if(distance < min_dist_)
				{
					ROS_ERROR_STREAM("Shooter server - closer than min dist (" << min_dist_ << "), can't shoot");
					return false;
				}
				//obtain speed and hood values
				hood_extended = true; //all auto shots have the hood up, hood down only for manual right-up-against the powerport
				shooter_speed = lerpTable(hood_up_table_, distance);
			}
			else {
				//ROS_ERROR("Shooter server - couldn't find a goal in getHoodAndVelocity()");
				return false; //if didn't find a goal, return false
			}

			return true;
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::ShooterGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;

			//make sure green light is on
			turnGreenLightOn(true);

			//wait for actionlib servers
			if(!ac_indexer_.waitForServer(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " couldn't find indexer actionlib server");
				turnGreenLightOn(false);
				as_.setPreempted();
				return;
			}

			//wait for all controller servers we need
			if(! shooter_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find controllers_2020_msgs");
				turnGreenLightOn(false);
				as_.setPreempted();
				return;
			}



			//align the turret
			behavior_actions::AlignToShootGoal align_goal;
			align_goal.mode = goal->mode;
			ac_align_.sendGoal(align_goal);
				//don't wait for it to finish, so we can start spinning up the shooter while its going. Will only call the indexer actionlib server if the align succeeded

			//keep shooting balls until driver says stop
			while(!timed_out_ && !preempted_ && ros::ok())
			{
				//determine shooter velocity/hood raise and go there with shooter
				ROS_INFO_STREAM(action_name_ << ": spinning up the shooter");
				controllers_2020_msgs::ShooterSrv srv;
				bool shooter_hood_raise;
				double shooter_velocity;
				if(goal->mode == 1) //near fixed shooting
				{
					shooter_hood_raise = false;
					shooter_velocity = near_shooting_speed_;
				}
				else if (goal->mode == 2) //far fixed shooting
				{
					shooter_hood_raise = true;
					shooter_velocity = far_shooting_speed_;
				}
				else //last option for goal->mode is 0=auto shooting, but also make this the default
				{
					//run a while loop in case the ZED takes its sweet time detecting a goal
					const double get_speed_start_time = ros::Time::now().toSec();
					bool got_speed = false;
					while(!timed_out_ && !preempted_ && ros::ok() && !got_speed)
					{
						got_speed = getHoodAndVelocity(shooter_hood_raise, shooter_velocity);
						if(as_.isPreemptRequested() || !ros::ok())
						{
							ROS_ERROR("Shooter server - Preempted while waiting for hood/velocity values");
							preempted_ = true;
						}
						else if(ros::Time::now().toSec() - start_time_ > server_timeout_ || ros::Time::now().toSec() - get_speed_start_time > get_speed_timeout_)
						{
							ROS_ERROR("Shooter server timed out while waiting for shooter speed/hood to be determined");
							timed_out_ = true;
						}
						else if(!got_speed)
						{
							r.sleep();
						}
					}

					if(!got_speed)
					{
						ROS_ERROR_STREAM(action_name_ << " couldn't get shooter speed, out of range or no goal detected");
						preempted_ = true;
						break;
					}

				}
				srv.request.shooter_hood_raise = shooter_hood_raise;
				srv.request.set_velocity = shooter_velocity + speed_offset_;
				if(!shooter_client_.call(srv))
				{
					ROS_ERROR_STREAM(action_name_ << " can't call shooter service");
					preempted_ = true;
				}
				ROS_INFO_STREAM(action_name_ << ": called shooter service");


				//wait for shooter ready to shoot
				double start_wait_for_ready_time = ros::Time::now().toSec();
				while(!preempted_ && !timed_out_ && ros::ok())
				{
					ROS_INFO_STREAM(action_name_ << ": waiting for the shooter to be ready");
					//check preempted
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while waiting for shooter to be ready");
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
				//feed ball into the shooter
				//wait for the align to succeed before calling the indexer
				while(!preempted_ && !timed_out_ && ros::ok())
				{
					//check if the turret align finished
					std::string align_state = ac_align_.getState().toString();
					if(align_state == "SUCCEEDED")
					{
						ROS_INFO_STREAM(action_name_ << ": calling indexer server to feed one ball into the shooter");
						//Call actionlib server
						behavior_actions::IndexerGoal indexer_goal;
						indexer_goal.action = SHOOT_ONE_BALL;
						ac_indexer_.sendGoal(indexer_goal);
						//wait for actionlib server
						waitForActionlibServer(ac_indexer_, 30, "calling indexer server", true); //method defined below. Args: action client, timeout in sec, description of activity
							//note: last arg is ignore_preempt, we want to wait for the current ball to finish shooting when the shooter is preempted - so don't stop waiting if we get preempted
						break;
					}
					else if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while waiting for align to succeed so we can call the indexer");
						preempted_ = true;
					}
					//check timed out
					else if (ros::Time::now().toSec() - start_time_ > server_timeout_ || ros::Time::now().toSec() - start_wait_for_ready_time > wait_for_ready_timeout_) {
						ROS_ERROR_STREAM(action_name_ << ": timed out while waiting for align to succeed so we can call the indexer");
						timed_out_ = true;
					}
					else {
						r.sleep();
					}
				}
			}

			//set final state using client calls
			controllers_2020_msgs::ShooterSrv srv;
			srv.request.set_velocity = 0;
			srv.request.shooter_hood_raise = false;
			if(!shooter_client_.call(srv))
			{
				ROS_ERROR_STREAM(action_name_ << " can't call shooter service");
				preempted_ = true;
			}
			ROS_INFO_STREAM(action_name_ << ": calling indexer server to go to intake position");
			//Call actionlib server
			behavior_actions::IndexerGoal indexer_goal;
			indexer_goal.action = POSITION_INTAKE;
			ac_indexer_.sendGoal(indexer_goal);
			//wait for actionlib server
			waitForActionlibServer(ac_indexer_, 30, "calling indexer server"); //method defined below. Args: action client, timeout in sec, description of activity

			//turn green light off, we're done with it
			turnGreenLightOn(false);


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

		void waitForActionlibServer(auto &action_client, double timeout, const std::string &activity, bool ignore_preempt=false)
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
				else if ( (!ignore_preempt && as_.isPreemptRequested()) || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempted_ during " << activity);
					preempted_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
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

		void shooterOffsetCB(const behavior_actions::ShooterOffset &msg)
		{
			speed_offset_ = msg.speed_offset;
		}

	public:
		//thread to publish if target is visible/within range
		void publishInRangeThread()
		{
		#ifdef __linux__
			//give the thread a name
			pthread_setname_np(pthread_self(), "shooter_in_range_pub_thread");
		#endif

			ros::Publisher in_range_pub = nh_.advertise<std_msgs::Bool>("shooter_in_range", 1);
			ros::Rate r(2); //2 hz is fine for GUI display
			std_msgs::Bool msg;

			while(ros::ok())
			{
				bool hood_extended;
				double shooter_speed;
				msg.data = getHoodAndVelocity(hood_extended, shooter_speed); //returns true if we detected a goal and are within range, false otherwise
				in_range_pub.publish(msg);

				r.sleep();
			}
		}

		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		ShooterAction(const std::string &name) :
			as_(nh_, name, boost::bind(&ShooterAction::executeCB, this, _1), false),
			action_name_(name),
			ac_align_("/align_to_shoot/align_to_shoot_server", true),
			ac_indexer_("/indexer/indexer_server", true),
			tf_listener_(tf_buffer_)
	{

		//get config values ------------------------------------
		ros::NodeHandle n_params_shooter(nh_, "actionlib_shooter_params"); //node handle for a lower-down namespace
		if (!n_params_shooter.getParam("server_timeout", server_timeout_)) {
			ROS_ERROR("Could not read server_timeout in shooter_server");
			server_timeout_ = 10;
		}
		if (!n_params_shooter.getParam("wait_for_server_timeout", wait_for_server_timeout_)) {
			ROS_ERROR("Could not read wait_for_server_timeout in shooter_server");
			wait_for_server_timeout_ = 10;
		}
		if (!n_params_shooter.getParam("wait_for_ready_timeout", wait_for_ready_timeout_)) {
			ROS_ERROR("Could not read wait_for_ready_timeout in shooter_server");
			wait_for_ready_timeout_ = 10;
		}
		if (!n_params_shooter.getParam("near_shooting_speed", near_shooting_speed_)) {
			ROS_ERROR("Could not read near_shooting_speed in shooter_server");
			near_shooting_speed_ = 300; //TODO fix
		}
		if (!n_params_shooter.getParam("far_shooting_speed", far_shooting_speed_)) {
			ROS_ERROR("Could not read far_shooting_speed in shooter_server");
			far_shooting_speed_ = 415; //TODO fix
		}
		if (!n_params_shooter.getParam("get_speed_timeout", get_speed_timeout_)) {
			ROS_ERROR("Could not read get_speed_timeout in shooter_server");
			get_speed_timeout_ = 1;
		}

		XmlRpc::XmlRpcValue hood_up_list;
		if(!n_params_shooter.getParam("hood_up_table", hood_up_list)){
			ROS_ERROR("Couldn't read hood_up_table in shooter_actionlib.yaml");
		}
		for(int i = 0; i < hood_up_list.size(); i++)
		{
			XmlRpc::XmlRpcValue &shooter_point = hood_up_list[i];

			std::map<std::string, double> shooter_map;

			if (shooter_point.hasMember("dist"))
			{
				XmlRpc::XmlRpcValue &xml_dist = shooter_point["dist"];
				if (!xml_dist.valid() || xml_dist.getType() != XmlRpc::XmlRpcValue::TypeDouble)
					ROS_ERROR("An invalid shooter distance was specified (expecting an double) in hood_up_table in shooter_actionlib.yaml");
				shooter_map["dist"] = xml_dist;
			}

			if (shooter_point.hasMember("speed"))
			{
				XmlRpc::XmlRpcValue &xml_speed = shooter_point["speed"];
				if (!xml_speed.valid() || xml_speed.getType() != XmlRpc::XmlRpcValue::TypeDouble)
					ROS_ERROR("An invalid shooter speed was specified (expecting an double) in hood_up_table in shooter_actionlib.yaml");
				shooter_map["speed"] = xml_speed;
			}

			hood_up_table_.push_back(shooter_map);
		}


		std::sort(hood_up_table_.begin(), hood_up_table_.end(), sortDistDescending);

		max_dist_ = hood_up_table_.front().at("dist");
		min_dist_ = hood_up_table_.back().at("dist");
		//end reading config ------------------------------------------------------


		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize client used to call controllers
		shooter_client_ = nh_.serviceClient<controllers_2020_msgs::ShooterSrv>("/frcrobot_jetson/shooter_controller/shooter_command", false, service_connection_header);

		//green light publisher to turn it on/off - planned to not use
		green_light_pub_ = nh_.advertise<std_msgs::Float64>("/frcrobot_rio/green_led_controller/command", 1);

		//subscribers
		ready_to_shoot_sub_ = nh_.subscribe("/frcrobot_jetson/shooter_controller/ready_to_shoot", 5, &ShooterAction::shooterReadyCB, this);
		goal_sub_ = nh_.subscribe("/goal_detection/goal_detect_msg", 5, &ShooterAction::goalDetectionCB, this);
		shooter_offset_sub_ = nh_.subscribe("/teleop/teleop_shooter_offsets", 5, &ShooterAction::shooterOffsetCB, this);

		//publish thread for if in range
		in_range_pub_thread_ = std::thread(std::bind(&ShooterAction::publishInRangeThread, this));
	}

		~ShooterAction(void)
		{
			if(in_range_pub_thread_.joinable())
			{
				in_range_pub_thread_.join();
			}
		}

		//config values
		double server_timeout_;
		double wait_for_server_timeout_;
		double wait_for_ready_timeout_;
		std::vector<std::map<std::string, double>> hood_up_table_;
		double max_dist_;
		double min_dist_;
		double near_shooting_speed_; //with hood down
		double far_shooting_speed_; //with hood up
		double get_speed_timeout_;
};


int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "shooter_server");
	ros::NodeHandle nh;

	//create the actionlib server
	ShooterAction shooter_action("shooter_server");


	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
