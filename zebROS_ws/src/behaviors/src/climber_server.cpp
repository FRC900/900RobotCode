#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/ClimbAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/ElevatorGoal.h"
#include "std_srvs/SetBool.h" //for the climber controller
#include "geometry_msgs/Twist.h" //for the drivebase
#include <atomic>
#include <ros/console.h>
#include "behaviors/enumerated_elevator_indices.h"
#include "frc_msgs/MatchSpecificData.h"
#include <thread>


//define global variables that will be defined based on config values

double elevator_deploy_timeout;
double elevator_climb_timeout;
double running_forward_timeout;
double elevator_climb_low_timeout;
double match_time_lock;
double wait_for_server_timeout;
double drive_forward_speed;

class ClimbAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::ClimbAction> as_; //create the actionlib server
		std::string action_name_;

		//create clients to call other actionlib servers
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ae_; //to call the elevator

		//create clients/subscribers to activate controllers
		ros::ServiceClient climber_controller_client_; //create a ros client to send requests to the climber controller (piston in the leg)
		ros::ServiceClient climber_engage_client_; //ros client to engage the climber via the elevator controller 
		ros::Publisher cmd_vel_publisher_;
		/*
		   std::atomic<int> linebreak_true_count; //counts how many times in a row the linebreak reported there's a cargo since we started trying to intake/outtake
		   std::atomic<int> linebreak_false_count; //same, but how many times in a row no cargo*/
		behaviors::ClimbResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		ros::Subscriber match_data_sub_;

		std::atomic<double> cmd_vel_forward_speed_;
		std::atomic<bool> stopped_;

		// Data from subscribers
		double match_time_remaining_;
		bool finished_climb_;

	public:
		//make the executeCB function run every time the actionlib server is called
		ClimbAction(const std::string &name) :
			as_(nh_, name, boost::bind(&ClimbAction::executeCB, this, _1), false),
			action_name_(name),
			ae_("/elevator/elevator_server", true)
	{
		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//get the match timer
		match_data_sub_ = nh_.subscribe("/frcrobot_jetson/match_data", 1,&ClimbAction::matchStateCallback,this); 
		//initialize the client being used to call the climber controller
		climber_controller_client_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/climber_controller/climber_feet_retract", false, service_connection_header);
		//initialize the client being used to call the climber controller to engage the climber
		climber_engage_client_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/climber_controller/climber_release_endgame", false, service_connection_header);

		//initialize the publisher used to send messages to the drive base
		cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
		//start subscribers subscribing
		//joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &ClimbAction::jointStateCallback, this);
	}

		~ClimbAction(void) 
		{
		}

		void cmdVelCallback()
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

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::ClimbGoalConstPtr &goal) {
			if(match_time_remaining_ > match_time_lock)
			{
				ROS_ERROR_STREAM("can not climb, too much time remaining in match");
				return;
			}

			if(!ae_.waitForServer(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("The elevator server was not loaded before the climber server needed it");
				as_.setPreempted();
				return;
			}
			if(!climber_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("The climber foot server was not loaded before the climber server needed it");
				as_.setPreempted();
				return;
			}
			if(!climber_engage_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("The climber engage server was not loaded before the climber server needed it");
				as_.setPreempted();
				return;
			}

			/* Steps to climb:
			 * Climber Server Step 0
			 * 1. Deploy foot via climber controller
			 * 2. Raise elevator to right height via elevator actionlib server
			 * 3. Deploy climber engagement piston via elevator controller
			 * 4. Lower elevator to right height to make the robot climb (this should be slow... change pid somehow?)
			 * 5. Retract foot to drop robot onto platform
			 * (driver drives forward a bit)
			 *
			 * Climber Server Step 1
			 * 6. Pull climber all the way up
			 * (driver drives forward a bit)
			 *
			 * Climber Server Step 2
			 * 7. Push climber a bit down to prop back wheels up
			 */
			ros::Rate r(20);

			std::thread cmdVelThread(std::bind(&ClimbAction::cmdVelCallback, this));

			//define variables that will be reused for each controller call/actionlib server call
			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;
			bool finished_before_timeout = false;

			if(goal->step == 0)
			{
				ROS_INFO("Running climber server step 0");
				cmd_vel_forward_speed_ = 0;
				//deploy foot using climber controller -----------------------------------------------
				//define service to send
				std_srvs::SetBool srv;
				srv.request.data = false; //shouldn't do anything, this is default
				//call controller
				if(!climber_controller_client_.call(srv))
				{
					ROS_ERROR("Foot deploy failed in climber controller");
					preempted = true;
				}
				else {
					//only spin if we're not going to error out
					// TODO - Why spin here?
					ros::spinOnce();
				}

				// raise elevator to right height so we can engage the climber ------------------------------------------------
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 0: raising elevator before climber is engaged");

					//call the elevator actionlib server
					//define the goal to send
					// TODO - try moving up fast for this step?
					behaviors::ElevatorGoal goal;
					goal.setpoint_index = ELEVATOR_DEPLOY;
					goal.place_cargo = 0; //doesn't actually do anything
					goal.raise_intake_after_success = true;
					//send the goal
					ae_.sendGoal(goal);
					finished_before_timeout = ae_.waitForResult(ros::Duration(elevator_deploy_timeout)); //wait until the action finishes, whether it succeeds, times out, or is preempted
					if(!finished_before_timeout){
						ROS_ERROR("climber server step 0: first elevator raise timed out");
						timed_out = true;
					}
					//determine the outcome of the goal
					// TODO - add a check for !timed_out here
					if(!ae_.getResult()->success)
					{
						if(ae_.getResult()->timed_out == true)
						{
							timed_out = true;
						}
						else
						{
							preempted = true;
						}
					}

					//check if we got a preempt while we were waiting
					// TODO - add a check for !timed_out and !preempted here
					if(as_.isPreemptRequested())
					{
						preempted = true;
					}
				} //end of raise elevator to right height before engaging

				//engage climber with elevator controller -----------------------------------------------------------------
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 0: engaging the climber with the elvator");

					//call the elevator controller
					//define the goal to send
					std_srvs::SetBool srv;
					srv.request.data = true;
					//call controller
					if(!climber_engage_client_.call(srv))
					{
						ROS_ERROR("climber server step 0: Climber engage failed in climber controller");
						preempted = true;
					}
					// TODO - Why spin here?
					ros::spinOnce();
				}

				// delay to make sure that we engaged properly
				// TODO - if we need to spin, spin in a loop here for 1 second here?
				ros::Duration(1).sleep();

				cmd_vel_forward_speed_ = drive_forward_speed;

				//lower elevator to make robot rise off ground
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 0: lowering elevator to make robot climb");

					//call the elevator actionlib server
					//define the goal to send
					behaviors::ElevatorGoal goal;
					goal.setpoint_index = ELEVATOR_CLIMB;
					goal.place_cargo = 0; //doesn't actually do anything 
					goal.raise_intake_after_success = true;
					//send the goal
					ae_.sendGoal(goal);
					finished_before_timeout = ae_.waitForResult(ros::Duration(elevator_deploy_timeout)); //wait until the action finishes, whether it succeeds, times out, or is preempted
					if(!finished_before_timeout){
						timed_out = true;
						ROS_ERROR("climber server step 0: second elevator move timed out");
					}

					//determine the outcome of the goal
					if(!ae_.getResult()->success) //this might mean preempted or timed out
					{
						ROS_INFO_STREAM("elevator did not succeed -- climber_server");
						preempted = true;
					}

					//check if we got a preempt while we were waiting
					if(as_.isPreemptRequested())
					{
						ROS_INFO_STREAM("preempt was requested -- climber_server");
						preempted = true;
					}
				} //end of lowering elevator to make robot climb

				ROS_INFO_STREAM("preempted = " << preempted);

				//handle preempting/timed out
				if(preempted || timed_out)
				{
					//send robot back to ground - go to deploy setpoint
					ROS_INFO("Climber server step 0: running preempt/timeout handling - moving robot to ground");

					//call the elevator actionlib server
					//define the goal to send
					behaviors::ElevatorGoal goal;
					goal.setpoint_index = ELEVATOR_DEPLOY;
					goal.place_cargo = 0; //doesn't actually do anything 
					goal.raise_intake_after_success = true;
					//send the goal
					ae_.sendGoal(goal);
					finished_before_timeout = ae_.waitForResult(ros::Duration(elevator_climb_timeout));
					if(!finished_before_timeout) //wait until the action finishes, whether it succeeds, times out, or is preempted
					{
						ROS_ERROR("climber server step 0: preempt/timeout handling elevator move timed out");
					}
				}

				ROS_INFO_STREAM("preempted = " << preempted);

				if(!preempted && !timed_out)
				{
					//retract climber foot to make robot fall
					//define service to send
					std_srvs::SetBool srv;
					srv.request.data = true;
					//call controller
					if(!climber_controller_client_.call(srv))
					{
						ROS_ERROR("climber server step 1: Foot retract failed in climber controller");
						preempted = true;
					}
					else {
						//only spin if we're not going to error out
						ros::spinOnce();
					}
				}

				ROS_INFO_STREAM("Driving forward");
				double start_time = ros::Time::now().toSec();
				while(ros::ok() && !preempted && !timed_out)
				{
					ROS_INFO_STREAM("DRIIIVING");
					timed_out = (ros::Time::now().toSec() -  start_time) > running_forward_timeout;
					preempted = as_.isPreemptRequested();
					r.sleep();
				}
				if(timed_out)
					ROS_INFO_STREAM("Driving forward has timed out");
				if(preempted)
					ROS_INFO_STREAM("Driving forward was preempted");


				//preempt handling: do nothing

			}
			else if(goal->step == 1)
			{
				if(!preempted && !timed_out && ros::ok())
				{
					// TODO - is this info correct?
					ROS_INFO("climber server step 0: raising elevator to make robot fall a bit");

					//call the elevator actionlib server
					//define the goal to send
					behaviors::ElevatorGoal goal;
					goal.setpoint_index = ELEVATOR_CLIMB_LOW;
					goal.place_cargo = 0; //doesn't actually do anything 
					goal.raise_intake_after_success = true;
					//send the goal
					ae_.sendGoal(goal);
					finished_before_timeout = ae_.waitForResult(ros::Duration(elevator_climb_timeout));
					if(!finished_before_timeout) //wait until the action finishes, whether it succeeds, times out, or is preempted
					{
						ROS_ERROR("climber server step 1: pulling up elevator timed out");
						timed_out = true;
					}

					//determine the outcome of the goal
					if(!ae_.getResult()->success) //this might mean preempted or timed out
					{
						ROS_INFO_STREAM("elevator did not succeed -- climber_server");
						preempted = true;
					}

					//check if we got a preempt while we were waiting
					if(as_.isPreemptRequested())
					{
						ROS_INFO_STREAM("preempt was requested -- climber_server");
						preempted = true;
					}
				} //end of raising elevator to make robot climb
			}
			else if(goal->step == 2)
			{
				ROS_INFO("Running climber server step 1");

				//pull climber all the way up - move elevator to climber deploy setpoint
				ROS_INFO("climber server step 1: raising elevator to pull climber back up");

				//call the elevator actionlib server
				//define the goal to send
				behaviors::ElevatorGoal goal;
				goal.setpoint_index = ELEVATOR_RAISE;
				goal.place_cargo = 0; //doesn't actually do anything
				goal.raise_intake_after_success = true;
				//send the goal
				ae_.sendGoal(goal);
				finished_before_timeout = ae_.waitForResult(ros::Duration(elevator_deploy_timeout));
				if(!finished_before_timeout) //wait until the action finishes, whether it succeeds, times out, or is preempted
					ROS_ERROR("climber server step 1: elevator raise timed out");

				//determine the outcome of the goal
				if(!ae_.getResult()->success) //this might mean preempted or timed out
				{
					preempted = true;
				}
				else {
					//Drive forward until drive forward timeout at end of game
					ROS_INFO_STREAM("Driving forward at end of climb");
					double start_time = ros::Time::now().toSec();
					
					while(ros::ok() && !preempted && !timed_out)
					{
						timed_out = (ros::Time::now().toSec() -  start_time) > running_forward_timeout;
						preempted = as_.isPreemptRequested();
						r.sleep();
					}
				}

				//check if we got a preempt while we were waiting
				if(as_.isPreemptRequested())
				{
					preempted = true;
				}

				//preempt handling: preempt elevator server to freeze the elevator
				if(preempted || timed_out)
				{
					ROS_INFO("Running climber server step 1 preempt/timeout handling - preempting elevator server");
					ae_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

			}

			//log state of action and set result of action

			// TODO : timed_out is never set
			if(timed_out)
			{
				result_.timed_out = true;
				result_.success = false;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				result_.timed_out = false;
				result_.success = false;
				as_.setPreempted(result_);

				ROS_INFO("%s: Preempted", action_name_.c_str());

			}
			else //implies succeeded
			{
				result_.timed_out = false;
				result_.success = true;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			stopped_ = true;
			cmdVelThread.join();

			return;
		}

		/*
		// Function to be called whenever the subscriber for the joint states topic receives a message
		// Grabs various info from hw_interface using
		// dummy joint position values
		void jointStateCallback(const sensor_msgs::JointState &joint_state)
		{
		//get index of linebreak sensor for this actionlib server
		static size_t linebreak_idx = std::numeric_limits<size_t>::max();
		if ((linebreak_idx >= joint_state.name.size()))
		{
		for (size_t i = 0; i < joint_state.name.size(); i++)
		{
		if (joint_state.name[i] == "cargo_intake_line_break") //TODO: define this in the hardware interface
		linebreak_idx = i;
		}
		}

		//update linebreak counts based on the value of the linebreak sensor
		if (linebreak_idx < joint_state.position.size())
		{
		bool linebreak_true = (joint_state.position[linebreak_idx] != 0);
		if(linebreak_true)
		{
		linebreak_true_count += 1;
		linebreak_false_count = 0;
		}
		else
		{
		linebreak_true_count = 0;
		linebreak_false_count += 1;
		}
		}
		else
		{
		static int count = 0;
		if(count % 100 == 0)
		{
		ROS_WARN("intake line break sensor not found in joint_states");
		}
		count++;
		linebreak_true_count = 0;
		linebreak_false_count += 1;
		}
		}
		*/
		void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
		{
			match_time_remaining_ = msg.matchTimeRemaining;
		}
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "climb_server");

	//create the cargo intake actionlib server
	ClimbAction climb_action("climber_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_lift_params(n, "climber_server");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
	{
		ROS_ERROR("Could not read wait_for_server_timeout in climber_server");
		wait_for_server_timeout = 10;
	}

	if (!n_lift_params.getParam("deploy_timeout", elevator_deploy_timeout))
	{
		ROS_ERROR("Could not read elevator_deploy_timeout in climber_server");
		elevator_deploy_timeout = 6;
	}

	if (!n_lift_params.getParam("climb_timeout", elevator_climb_timeout))
	{
		ROS_ERROR("Could not read elevator_climb_timeout in climber_server");
		elevator_climb_timeout = 20;
	}

	if (!n_lift_params.getParam("running_forward_timeout", running_forward_timeout))
	{
		ROS_ERROR("Could not read running_forward_timeout in climber_server");
		running_forward_timeout= 2;
	}

	if (!n_lift_params.getParam("climb_low_timeout", elevator_climb_low_timeout))
	{
		ROS_ERROR("Could not read climb_low_timeout in climber_server");
		elevator_climb_low_timeout = 6;
	}

	if (!n_lift_params.getParam("match_time_lock", match_time_lock))
	{
		ROS_ERROR("Could not read match_time_lock in climber_server");
		match_time_lock = 135;
	}

	if (!n_lift_params.getParam("drive_forward_speed", drive_forward_speed))
	{
		ROS_ERROR("Could not read drive_forward_speed in climber_server");
		drive_forward_speed = 0.2;
	}

	ros::spin();
	return 0;
}
