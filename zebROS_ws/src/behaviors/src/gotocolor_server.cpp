//THIS IS A TEMPLATE FILE. TO USE, COPY IT AND REPLACE:
// ServerName	with your server's name, e.g. CargoIntake
// server_name	with your server's name, e.g. cargo_intake
// Thing		with the name of your action file (if file is Intake.action, replace w/ Intake)


#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include <atomic>
#include <cctype>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/GoToColorAction.h"

//include controller service files and other service files
// e.g. #include "controller_package/ControllerSrv.h"
// e.g. #include "sensor_msgs/JointState.h" -has linebreak sensor data FYI
#include "frc_msgs/MatchSpecificData.h"
#include "color_spin/color_algorithm.h"
#include "controllers_2020_msgs/ControlPanelSrv.h"
#include "controllers_2020_msgs/ClimberSrv.h"
#include "std_msgs/Int8.h"

enum colors{blue, red, yellow, green};

//create the class for the actionlib server
class GoToColorControlPanelAction {
	protected:
		ros::NodeHandle nh_;
        ros::Subscriber match_sub_;
		//ros::Subscriber talon_states_sub_;
		ros::Subscriber color_detect_sub_;
		actionlib::SimpleActionServer<behavior_actions::GoToColorAction> as_; //create the actionlib server
		std::string action_name_;

		//ros::Publisher cmd_vel_publisher_;

		//clients to call controllers
		//e.g. ros::ServiceClient mech_controller_client_; //create a ros client to send requests to the controller
		ros::ServiceClient color_algorithm_client_;
		ros::ServiceClient control_panel_controller_client_;
		ros::ServiceClient climber_controller_client_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		bool preempted_;
		bool timed_out_;
		ros::Rate r{10}; //used for wait loops, curly brackets needed so it doesn't think this is a function
		double start_time_;

		std::atomic<char> goal_color_{0};
		char current_color_;

		/*
		std::atomic<double> cmd_vel_forward_speed_;
        std::atomic<bool> stopped_;
		*/

        //config variables, with defaults
        double server_timeout_; //overall timeout for your server
        double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one

	public:

		double climb_wait;
		double spin_wait;

		/*
		double drive_forward_speed;
		double back_up_speed;

		double avg_current;header
		double avg_current_limit;

		int num_drive_motors;

		double friction_wait;
		double back_up_wait;
		*/

		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		GoToColorControlPanelAction(const std::string &name, double server_timeout, double wait_for_server_timeout) :
			as_(nh_, name, boost::bind(&GoToColorControlPanelAction::executeCB, this, _1), false),
			action_name_(name),
                        server_timeout_(server_timeout),
                        wait_for_server_timeout_(wait_for_server_timeout)
			//ac_elevator_("/elevator/elevator_server", true) example how to initialize other action clients, don't forget to add a comma on the previous line
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		match_sub_=nh_.subscribe("/frcrobot_rio/match_data", 1, &GoToColorControlPanelAction::matchColorCallback, this);
		color_detect_sub_=nh_.subscribe("/color_detected", 1, &GoToColorControlPanelAction::detectColorCallback, this);
		//talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &GoToColorControlPanelAction::talonStateCallback, this);

		color_algorithm_client_=nh_.serviceClient<color_spin::color_algorithm>("color_algorithm", false, service_connection_header);
		control_panel_controller_client_=nh_.serviceClient<controllers_2020_msgs::ControlPanelSrv>("control_panel_controller", false, service_connection_header);
		climber_controller_client_=nh_.serviceClient<controllers_2020_msgs::ClimberSrv>("climber_controller", false, service_connection_header);

		//initialize the publisher used to send messages to the drive base
        //cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	}

		~GoToColorControlPanelAction (void)
		{
		}

		//Use to make pauses while still checking timed_out_ and preempted_
		bool pause(const double duration, const std::string &activity)
		{
			const double pause_start_time = ros::Time::now().toSec();

			while(!preempted_ && !timed_out_ && ros::ok())
			{
				if(as_.isPreemptRequested() || !ros::ok())
				{
					preempted_ = true;
					ROS_ERROR_STREAM("server_name_server: preempt during pause() - " << activity);
				}
				else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
				{
					timed_out_ = true;
					ROS_ERROR_STREAM("server_name_server: timeout during pause() - " << activity);
				}

				if((ros::Time::now().toSec() - pause_start_time) >= duration)
				{
					return true; //pause worked like expected
				}
			}

			return false; //wait loop must've been broken by preempt, global timeout, or ros not ok
		}

		// Basic thread which spams cmd_vel to the drive base to
		// continually drive forward during the climb
		/*
		void cmdVelThread()
		{
				ROS_INFO_STREAM("the callback is being called");
				geometry_msgs::Twist cmd_vel_msg;
				stopped_ = false;

				ros::Rate r(20);

				while(ros::ok() && !stopped_)
				{
						cmd_vel_msg.linear.x = cmd_vel_forward_speed_;
						cmd_vel_msg.linear.y = 0.0;
						cmd_vel_msg.linear.z = 0.0;
						cmd_vel_msg.angular.x = 0.0;
						cmd_vel_msg.angular.y = 0.0;
						cmd_vel_msg.angular.z = 0.0;

						cmd_vel_publisher_.publish(cmd_vel_msg);
						r.sleep();
				}
		}
		*/

		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::GoToColorGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;

			if(goal_color_ == '\0')
			{
				ROS_ERROR_STREAM("Can't spin to color, match data hasn't given a color");
				return;
			}

			if(! control_panel_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find control_panel_controller");
				as_.setPreempted();
				return;
			}

			if(! climber_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find climber_controller");
				as_.setPreempted();
				return;
			}

			/*
			std::thread cmdVelThread(std::bind(&GoToColorControlPanelAction::cmdVelThread, this));
			cmd_vel_foward_speed_ = 0;
			*/

			//Extend the climber to deploy the mechanism
			if(!preempted_ && !timed_out_ && ros::ok())
			{
				controllers_2020_msgs::ClimberSrv climb_srv;
				climb_srv.request.winch_percent_out = 0;
				climb_srv.request.climber_deploy = true;
				climb_srv.request.climber_elevator_brake = true;
				if(!climber_controller_client_.call(climb_srv))
				{
					ROS_ERROR_STREAM(action_name_ << ": preempt while calling climb service");
					preempted_ = true;
				}
			}

			pause(climb_wait,"Extending climber");


			/*
			if(!preempted_ && !timed_out_ && ros::ok())
				cmd_vel_forward_speed_ = drive_forward_speed; //Drive forward while we rotate

			//Drive forward until we hit the wheel
			while(avg_current < avg_current_limit && !preempted_ && !timed_out_ && ros::ok())
			{
				//check preempted_
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempt while driving forward");
					preempted_ = true;
				}
				//check timed out - TODO might want to use a timeout for this specific controller call rather than the whole server's timeout?
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out while driving forward");
					timed_out_ = true;
				}
				//otherwise, pause then loop again
				else {
					r.sleep();
				}
			}
			*/

			double panel_rotations;
			//Loop while calling rotation
			while(!preempted_ && !timed_out_ && ros::ok())
			{
				if(goal_color_ == current_color_) //If the color detected is the color we want, break out of loop
					break;

				//Call algorithm to calculate rotation needed
				color_spin::color_algorithm spin_srv;
				spin_srv.request.sensor_color = current_color_;
				spin_srv.request.fms_color = goal_color_;
				if(color_algorithm_client_.call(spin_srv))
				{
					panel_rotations = spin_srv.response.rotate;
				} else {
					ROS_ERROR_STREAM(action_name_ << ": preempt while calling color algorithm");
					preempted_ = true;
				}

				if(!preempted_ && !timed_out_ && ros::ok()) //Send rotation to control panel controller
				{
					controllers_2020_msgs::ControlPanelSrv panel_srv;
                    panel_srv.request.control_panel_rotations = panel_rotations; //Might need to be negative, but hopefully not?
					if(!control_panel_controller_client_.call(panel_srv)) {
						ROS_ERROR_STREAM(action_name_ << ": preempt while calling control panel controller");
						preempted_ = true;
					}
				}

				//check preempted_
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempt while calling control panel controller");
					preempted_ = true;
				}
				//check timed out
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out while calling control panel controller");
					timed_out_ = true;
				}
				//otherwise, pause then loop again
				else {
					pause(spin_wait,"Spinning panel");
				}
			}

			/*
			pause(friction_wait, "Holding friction while wheel stops"); //Pause a bit so we can hold friction while the wheel stops spinning

			//Drive backwards
			if(!preempted_ && !timed_out_ && ros::ok())
				cmd_vel_forward_speed_ = -back_up_speed; //Drive forward while we rotate

			pause(back_up_wait, "Backing up");

			//Finish -----------------------------------------------
			stopped_ = true; //Stop driving
			*/

			//Retract the climber
			controllers_2020_msgs::ClimberSrv climb_srv;
			climb_srv.request.winch_percent_out = 0;
			climb_srv.request.climber_deploy = false;
			climb_srv.request.climber_elevator_brake = true;
			if(!climber_controller_client_.call(climb_srv))
			{
				ROS_ERROR_STREAM(action_name_ << ": preempt while calling climb service");
				preempted_ = true;
			}

			pause(climb_wait,"Retracting climber");

			//log result and set actionlib server state appropriately
			behavior_actions::GoToColorResult result;

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


		template <class T>
		void waitForActionlibServer(T &action_client, double timeout, const std::string &activity)
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
					r.sleep();
				}
			}
		}

		void matchColorCallback(const frc_msgs::MatchSpecificData &match_data){
			if(match_data.gameSpecificData.size() > 0)
				goal_color_= std::toupper(match_data.gameSpecificData.at(0));
		}

		void detectColorCallback(const std_msgs::Int8 &color_data){
			switch(color_data.data) {
				case blue:
					current_color_ = 'B';
					break;

				case red:
					current_color_ = 'R';
					break;

				case yellow:
					current_color_ = 'Y';
					break;

				case green:
					current_color_ = 'G';
					break;

				default:
					ROS_ERROR_STREAM(action_name_ << ": color detect didn't return valid color");
			}
		}

		/*
		void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
		{
			double current_sum = 0;
			for(size_t i = 0; i < talon_state.name.size(); i++)
			{
				if(talon_state.name[i] == "") //Insert drive motor talon names
				{
					current_sum += talon_state.output_current[i];
				}
			}
			avg_current = current_sum/num_drive_motors;
		}
		*/
};
int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "go_to_color_server");

	//get config values
	ros::NodeHandle n;

    double server_timeout = 100;
    double wait_for_server_timeout = 10;

	//create the actionlib server
	GoToColorControlPanelAction go_to_color_control_panel("go_to_color_server", server_timeout, wait_for_server_timeout);

	/*
	if (!n.getParam("/actionlib_gotocolor_params/drive_forward_speed", go_to_color_control_panel.drive_forward_speed))
        {
                ROS_ERROR("Could not read drive_forward_speed in go_to_color_control_panel_server");
                go_to_color_control_panel.drive_forward_speed = 0.2;
        }
	*/

	if (!n.getParam("/actionlib_gotocolor_params/climb_wait_time", go_to_color_control_panel.climb_wait))
        {
                ROS_ERROR("Could not read climb_wait_time in go_to_color_control_panel_server");
                go_to_color_control_panel.climb_wait = 1;
        }

	/*
	if (!n.getParam("/actionlib_gotocolor_params/friction_wait_time", go_to_color_control_panel.friction_wait))
        {
                ROS_ERROR("Could not read friction_wait_time in go_to_color_control_panel_server");
                go_to_color_control_panel.friction_wait = .2;
        }

	if (!n.getParam("/actionlib_gotocolor_params/back_up_time", go_to_color_control_panel.back_up_wait))
        {
                ROS_ERROR("Could not read back_up_time in go_to_color_control_panel_server");
                go_to_color_control_panel.back_up_wait = .5;
        }

	if (!n.getParam("/actionlib_gotocolor_params/back_up_speed", go_to_color_control_panel.back_up_speed))
        {
                ROS_ERROR("Could not read back_up_time in go_to_color_control_panel_server");
                go_to_color_control_panel.back_up_speed = .5;
        }

	if (!n.getParam("/actionlib_gotocolor_params/avg_current_limit", go_to_color_control_panel.avg_current_limit))
        {
                ROS_ERROR("Could not read avg_current_limit in go_to_color_control_panel_server");
                go_to_color_control_panel.avg_current_limit = .01;
        }

	if (!n.getParam("/actionlib_gotocolor_params/num_drive_motors", go_to_color_control_panel.num_drive_motors))
        {
                ROS_ERROR("Could not read num_drive_motors in go_to_color_control_panel_server");
                go_to_color_control_panel.num_drive_motors = 4;
        }
	*/

	if (!n.getParam("/actionlib_gotocolor_params/spin_pause", go_to_color_control_panel.spin_wait))
        {
                ROS_ERROR("Could not read spin_pause in go_to_color_control_panel_server");
                go_to_color_control_panel.spin_wait = 1;
        }

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
