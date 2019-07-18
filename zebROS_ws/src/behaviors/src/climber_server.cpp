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
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "cargo_intake_controller/CargoIntakeSrv.h"
#include <talon_state_controller/TalonState.h>

//define global variables that will be defined based on config values

//durations and timeouts
double elevator_deploy_timeout;
double elevator_climb_timeout;
double running_forward_timeout;
double elevator_climb_low_timeout;
double match_time_lock;
double wait_for_server_timeout;
double wait_at_top_for_engage;
double pull_leg_up_pause_time;

//other config variables
double drive_forward_speed;
double delay_before_engage;
double delay_before_continue_retract;

int linebreak_debounce_iterations;
double climber_engage_pos;
double climb_raise_position;

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

	    std::atomic<int> linebreak_true_count; //counts how many times in a row the linebreak reported there's a cargo since we started trying to intake/outtake
	    std::atomic<int> linebreak_false_count; //same, but how many times in a row no cargo
		behaviors::ClimbResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		ros::Subscriber match_data_sub_;
		ros::Subscriber joint_states_sub_;
		ros::Subscriber navX_sub_;
		ros::Subscriber talon_states_sub_;

		std::atomic<double> cmd_vel_forward_speed_;
		std::atomic<bool> stopped_;

		// Data from subscribers
		double match_time_remaining_;
		double navX_roll_;
		double navX_pitch_;
		double elev_cur_position_;
		bool climber_engaged_;

		ros::ServiceClient cargo_intake_controller_client_; //create a ros client to send requests to the controller
		// Try to safely wait for an elevator action to succeed
		void waitForElevator(bool &timed_out, bool &preempted, int step, ros::Rate &r, double timeout)
		{
			bool waiting_for_elevator = true;
			const double start_time = ros::Time::now().toSec();
			while (waiting_for_elevator && ros::ok())
			{
				auto state = ae_.getState();
				if ((state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) ||
						(state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED))
				{
					waiting_for_elevator = false;
					if (state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED)
					{
						ROS_INFO_STREAM("elevator " << step << " state returned preempted");
						preempted = true;
					}
					if (!ae_.getResult()->success)
					{
						ROS_INFO_STREAM("elevator " << step << " did not succeed -- climber_server");
						preempted = true;
					}
				}
				else if(as_.isPreemptRequested())
				{
					waiting_for_elevator = false;
					preempted = true;
				}
				else if((ros::Time::now().toSec() - start_time) > timeout)
				{
					ROS_ERROR_STREAM("climber server step " << step << " : timed out");
					timed_out = true;
					waiting_for_elevator = false;
				}
				else
				{
					ros::spinOnce();
					r.sleep();
				}
			}
		}

		/*
		void elevatorClimbConnectThread()
		{
			if(elev_cur_position_ <= climb_raise_position)
			{
				climber_engaged_ = true;
				ROS_WARN_STREAM("elevator connected with climber");
			}
		}*/

		// Basic thread which spams cmd_vel to the drive base to
		// continually drive forward during the climb
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
			 * Raise elevator to right height via elevator actionlib server
			 * Climber Server Step 1
			 * Deploy foot via climber controller
			 * Deploy climber engagement piston via elevator controller
			 * Start driving forward
			 * Lower elevator to right height to make the robot climb (this should be slow... change pid somehow?)
			 * Retract foot to drop robot onto platform
			 * Climber Server Step 2
			 * Pull climber leg up a bit, then pause
			 * Pull climber leg the rest of the way up
			 */
			ros::Rate r(20);

			std::thread cmdVelThread(std::bind(&ClimbAction::cmdVelThread, this));
			//std::thread elevatorClimbConnectThread; //will be initialized right before moving the elevator down to make robot rise in air
			cmd_vel_forward_speed_ = 0; //Make sure it doesn't start moving

			//define variables that will be reused for each controller call/actionlib server call
			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;

			if(goal->step == 0)
			{
				ROS_INFO("Running climber server step 0");

				//pull cargo mech up ---------------------------------------------------
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 0: raising cargo mech up");
					cargo_intake_controller::CargoIntakeSrv cargo_srv;
					cargo_srv.request.power = 0;
					cargo_srv.request.intake_arm = false;
					//send request to controller
					if(!cargo_intake_controller_client_.call(cargo_srv))
					{
						ROS_ERROR("Climber server step 0: cargo intake controller call failed to lift arm up");
						preempted = true;
					}
				}
				// raise elevator to right height so we can engage the climber ------------------------------------------------
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 0: raising elevator before climber is engaged");
					//call the elevator actionlib server
					//define the goal to send
					behaviors::ElevatorGoal elevator_goal;
					elevator_goal.setpoint_index = ELEVATOR_DEPLOY;
					elevator_goal.place_cargo = 0; //doesn't actually do anything
					//send the elevator_goal
					ae_.sendGoal(elevator_goal);
					waitForElevator(timed_out, preempted, goal->step, r, elevator_deploy_timeout);
				} //end of raise elevator to right height before engaging
				//preempt handling
				if(preempted || timed_out || !ros::ok())
				{
					ROS_WARN_STREAM("Climber server timed out or was preempted in step 0");
					ae_.cancelGoalsAtAndBeforeTime(ros::Time::now());
					cmd_vel_forward_speed_ = 0;
				}

			}
			else if(goal->step == 1)
			{
				//TODO make sure elevator is in the correct location before continuing
				ROS_INFO("Running climber server step 1");
				cmd_vel_forward_speed_ = 0; //Don't move yet

				//deploy foot using climber controller -----------------------------------------------
				ROS_INFO("climber server step 0: ensure initial foot state");
				std_srvs::SetBool srv;
				srv.request.data = false; //shouldn't do anything, this is default
				//call controller
				if(!climber_controller_client_.call(srv))
				{
					ROS_ERROR("Foot deploy failed in climber controller");
					preempted = true;
				}

				// pop out pin to engage climber with elevator -----------------------------------------------------------------
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 1: engaging the climber with the elevator");

					//call the elevator controller
					//define the goal to send
					std_srvs::SetBool srv;
					srv.request.data = true;
					//call controller
					if(!climber_engage_client_.call(srv))
					{
						ROS_ERROR("climber server step 1: Climber engage failed in climber controller");
						preempted = true;
					}
				}


				//start driving forward as we lift off the ground --------------------------
				ROS_INFO("climber server step 1: starting to drive forward");
				cmd_vel_forward_speed_ = drive_forward_speed;

				//std::thread elevatorClimbConnectThread(std::bind(&ClimbAction::elevatorClimbConnectThread, this));
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 1: lowering elevator to make robot climb");

					behaviors::ElevatorGoal elevator_goal;
					elevator_goal.setpoint_index = ELEVATOR_CLIMB;
					elevator_goal.place_cargo = 0; //doesn't actually do anything
					//send the elevator_goal
					ae_.sendGoal(elevator_goal);
					waitForElevator(timed_out, preempted, goal->step, r, elevator_deploy_timeout);
				} //end of lowering elevator to make robot climb

				//ROS_INFO_STREAM("preempted = " << preempted);

				//handle preempting/timed out for step 1, prior to falling ------------------------------------------------------
				if(preempted || timed_out || !ros::ok())
				{
					ROS_WARN_STREAM("Climber server timed out or was preempted in step 1 prior to falling");
					//ae_.cancelGoalsAtAndBeforeTime(ros::Time::now());
					cmd_vel_forward_speed_ = 0;
				}
				else
				{
					//retract climber foot to make robot fall ---------------------------------------
					if(!preempted && !timed_out && ros::ok()) //TODO we don't need this if statement as we just checked these cases above
					{
						ROS_INFO_STREAM("Climber server step 1: Retracting foot to make robot fall");
						std_srvs::SetBool srv;
						srv.request.data = true;
						if(!climber_controller_client_.call(srv))
						{
							ROS_ERROR("climber server step 1: Foot retract failed in climber controller");
							preempted = true;
						}
					}

					//keep driving forward until preempted or timed out -------------------------------------
					ROS_INFO_STREAM("Climber step 1: Driving forward after fall, until timeout or preempt");
					double start_time = ros::Time::now().toSec();
					while(ros::ok() && !preempted && !timed_out)
					{
						timed_out = (ros::Time::now().toSec() -  start_time) > running_forward_timeout;
						preempted = as_.isPreemptRequested();
						r.sleep();
					}
					if(timed_out)
						ROS_WARN_STREAM("Driving forward has timed out");
					if(preempted)
						ROS_WARN_STREAM("Driving forward was preempted");

					//preempt handling after falling -----------------------------------------------------
					if(preempted || timed_out || !ros::ok())
					{
						ROS_INFO("Climber server step 1 timed out or preempted after falling");
						//ae_.cancelGoalsAtAndBeforeTime(ros::Time::now());
						cmd_vel_forward_speed_ = 0;
					}
				}
			}
			if(goal->step == 2)
			{
				cmd_vel_forward_speed_ = drive_forward_speed;

				//raise climber leg up a bit --------------------------------------------------------
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 2: raising climber leg up a bit, ensure cargo intake is out");

					cargo_intake_controller::CargoIntakeSrv srv;
					srv.request.power = 0;
					srv.request.intake_arm = true;
					//send request to controller
					if(!cargo_intake_controller_client_.call(srv))
					{
						ROS_ERROR("Climber server step 2: cargo intake controller call failed to put cargo mech down");
						preempted = true;
					}

					//pull the climber leg up a bit
					behaviors::ElevatorGoal elevator_goal;
					elevator_goal.setpoint_index = ELEVATOR_CLIMB_LOW;
					elevator_goal.place_cargo = 0; //doesn't actually do anything
					//send the elevator_goal
					ae_.sendGoal(elevator_goal);
					waitForElevator(timed_out, preempted, goal->step, r, elevator_climb_timeout);
				}


				//wait a little before continuing to retract leg all the waay up and make sure linebreaks are triggered
				ROS_INFO("climber server step 2: Pausing before finishing retracting leg");
				double start_time = ros::Time::now().toSec();
				while(ros::ok() && !preempted && !timed_out) {
					timed_out = (ros::Time::now().toSec() -  start_time) > delay_before_continue_retract;
					if(as_.isPreemptRequested())
					{
						ROS_INFO_STREAM("climber server step 2: preempt requested before 2nd segment of retracting leg");
						preempted = true;
					}
					ros::spinOnce();
					r.sleep();

				}
				//pull climber leg all the way up ------------------------------------------------------
				if(linebreak_true_count > linebreak_debounce_iterations && !preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 2: raising elevator to pull climber all the way back up");
					//call the elevator actionlib server
					//define the goal to send
					behaviors::ElevatorGoal elevator_goal;
					elevator_goal.setpoint_index = ELEVATOR_RAISE;
					elevator_goal.place_cargo = 0; //doesn't actually do anything
					//send the elevator_goal
					ae_.sendGoal(elevator_goal);
					waitForElevator(timed_out, preempted, goal->step, r, elevator_climb_timeout);

					//determine the outcome of the goal
					if (!preempted)
					{
						//Drive forward until drive forward timeout at end of game
						ROS_INFO_STREAM("Driving forward at end of climb");
						const double start_time = ros::Time::now().toSec();

						while(ros::ok() && !preempted && !timed_out)
						{
							timed_out = (ros::Time::now().toSec() -  start_time) > running_forward_timeout;
							preempted = as_.isPreemptRequested();
							r.sleep();
						}
					}
				}
				else {
					ROS_WARN("climber server step 2: linebreak sensors not triggered. NOT CONTINUING CLIMB");
				}
				//preempt handling: preempt elevator server to freeze the elevator
				if(preempted || timed_out || !ros::ok())
				{
					ROS_INFO("Running climber server step 2 preempt/timeout handling - preempting elevator server and stopping drive forward");
					//ae_.cancelGoalsAtAndBeforeTime(ros::Time::now());
					cmd_vel_forward_speed_ = 0;
				}
			}
			if(goal->step == 3) {
				ROS_WARN("Overriding linebreak sensor automation! Continuing climb manually!");
				ROS_WARN("climber server step 3: raising elevator to pull climber all the way back up");
				//call the elevator actionlib server
				//define the goal to send
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.setpoint_index = ELEVATOR_RAISE;
				elevator_goal.place_cargo = 0; //doesn't actually do anything
				//send the elevator_goal
				ae_.sendGoal(elevator_goal);
				waitForElevator(timed_out, preempted, goal->step, r, elevator_climb_timeout);

				//determine the outcome of the goal
				if (!preempted)
				{
					//Drive forward until drive forward timeout at end of game
					ROS_INFO_STREAM("Driving forward at end of climb");
					const double start_time = ros::Time::now().toSec();

					while(ros::ok() && !preempted && !timed_out)
					{
						timed_out = (ros::Time::now().toSec() -  start_time) > running_forward_timeout;
						preempted = as_.isPreemptRequested();
						r.sleep();
					}
				}
			}

			//log state of action and set result of action

			// TODO : timed_out is never set
			behaviors::ClimbResult result; //variable to store result of the actionlib action
			if(timed_out)
			{
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);

				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else //implies succeeded
			{
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);

				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			stopped_ = true;

			ROS_INFO("Before join");
			cmdVelThread.join();
			ROS_INFO("Between join");
			//elevatorClimbConnectThread.join();
			ROS_INFO("After join");
			return;
		}

		void navXCallback(const sensor_msgs::Imu &navXState)
		{
			const tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
			double roll;
			double pitch;
			double yaw;
			tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);

			if (roll == roll) // ignore NaN results
				navX_roll_ = roll;

			if (pitch == pitch) // ignore NaN results
				navX_pitch_ = pitch;
		}

		/*
		// Function to be called whenever the subscriber for the joint states topic receives a message
		// Grabs various info from hw_interface using
		// dummy joint position values
		// */
		void jointStateCallback(const sensor_msgs::JointState &joint_state)
		{
			//get index of linebreak sensor for this actionlib server
			static size_t linebreak_idx_1 = std::numeric_limits<size_t>::max();
			static size_t linebreak_idx_2 = std::numeric_limits<size_t>::max();
			if ((linebreak_idx_1 >= joint_state.name.size()) && (linebreak_idx_2 >= joint_state.name.size()) )
			{
				for (size_t i = 0; i < joint_state.name.size(); i++)
				{
					if (joint_state.name[i] == "climber_linebreak_1") //TODO: define this in the hardware interface
						linebreak_idx_1 = i;
					if (joint_state.name[i] == "climber_linebreak_2") //TODO: define this in the hardware interface
						linebreak_idx_2 = i;
				}
			}

			//update linebreak counts based on the value of the linebreak sensor
			if (linebreak_idx_1 < joint_state.position.size() && linebreak_idx_2 < joint_state.position.size())
			{
				bool linebreak_true = (joint_state.position[linebreak_idx_1] != 0) && (joint_state.position[linebreak_idx_2] != 0);
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
			ROS_WARN_THROTTLE(2.0, "climber_server : intake line break sensor not found in joint_states");
			linebreak_true_count = 0;
			linebreak_false_count += 1;
			}
		}

		void talonStateCallback(const talon_state_controller::TalonState &talon_state)
		{
			static size_t elevator_master_idx = std::numeric_limits<size_t>::max();
			if (elevator_master_idx >= talon_state.name.size())
			{
				for (size_t i = 0; i < talon_state.name.size(); i++)
				{
					if (talon_state.name[i] == "elevator_master")
					{
						elevator_master_idx = i;
						break;
					}
				}
			}
			else {
				elev_cur_position_ = talon_state.position[elevator_master_idx];
			}
		}


		void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
		{
			match_time_remaining_ = msg.matchTimeRemaining;
		}

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
		match_data_sub_ = nh_.subscribe("/frcrobot_rio/match_data", 1, &ClimbAction::matchStateCallback,this);
		//initialize the client being used to call the climber controller
		climber_controller_client_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/climber_controller/climber_feet_retract", false, service_connection_header);
		//initialize the client being used to call the climber controller to engage the climber
		climber_engage_client_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/climber_controller/climber_release_endgame", false, service_connection_header);

		navX_sub_ = nh_.subscribe("/frcrobot_rio/navx_mxp", 1, &ClimbAction::navXCallback, this);
		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states",1,&ClimbAction::talonStateCallback, this);

		cargo_intake_controller_client_ = nh_.serviceClient<cargo_intake_controller::CargoIntakeSrv>("/frcrobot_jetson/cargo_intake_controller/cargo_intake_command", false, service_connection_header);

		//initialize the publisher used to send messages to the drive base
		cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
		//start subscribers subscribing
		joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &ClimbAction::jointStateCallback, this);
	}
	~ClimbAction(void)
	{
	}

};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "climb_server");

	//create the cargo intake actionlib server
	ClimbAction climb_action("climber_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_climb_params(n, "climber_server");

	if (!n.getParam("/action_lift_params/climber3/climb_raise_position", climb_raise_position))
	{
		ROS_ERROR("could not read climb_raise_position");
		climb_raise_position = 0.79;
	}

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
	{
		ROS_ERROR("Could not read wait_for_server_timeout in climber_server");
		wait_for_server_timeout = 10;
	}

	if (!n_climb_params.getParam("wait_at_top_for_engage", wait_at_top_for_engage))
	{
		ROS_ERROR("Could not read wait_at_top_for_engage in climber_server");
		wait_at_top_for_engage = 1;
	}

	if (!n_climb_params.getParam("deploy_timeout", elevator_deploy_timeout))
	{
		ROS_ERROR("Could not read elevator_deploy_timeout in climber_server");
		elevator_deploy_timeout = 6;
	}

	if (!n_climb_params.getParam("climb_timeout", elevator_climb_timeout))
	{
		ROS_ERROR("Could not read elevator_climb_timeout in climber_server");
		elevator_climb_timeout = 20;
	}

	if (!n_climb_params.getParam("running_forward_timeout", running_forward_timeout))
	{
		ROS_ERROR("Could not read running_forward_timeout in climber_server");
		running_forward_timeout= 2;
	}

	if (!n_climb_params.getParam("climb_low_timeout", elevator_climb_low_timeout))
	{
		ROS_ERROR("Could not read climb_low_timeout in climber_server");
		elevator_climb_low_timeout = 6;
	}

	if (!n_climb_params.getParam("match_time_lock", match_time_lock))
	{
		ROS_ERROR("Could not read match_time_lock in climber_server");
		match_time_lock = 135;
	}

	if (!n_climb_params.getParam("pull_leg_up_pause_time", pull_leg_up_pause_time))
	{
		ROS_ERROR("Could not read pull_leg_up_pause_time in climber_server");
		pull_leg_up_pause_time = 0.5;
	}

	if (!n_climb_params.getParam("drive_forward_speed", drive_forward_speed))
	{
		ROS_ERROR("Could not read drive_forward_speed in climber_server");
		drive_forward_speed = 0.2;
	}
	if (!n_climb_params.getParam("delay_before_engage", delay_before_engage))
	{
		ROS_ERROR("Could not read delay_before_engage in climber_server");
		delay_before_engage = 0.2;
	}
	if (!n_climb_params.getParam("delay_before_continue_retract", delay_before_continue_retract))
	{
		ROS_ERROR("Could not read delay_before_continue_retract in climber_server");
		delay_before_continue_retract = 0.2;
	}
	if (!n_climb_params.getParam("linebreak_debounce_iterations", linebreak_debounce_iterations))
	{
		ROS_ERROR("Could not read linebreak_debounce_iterations in climber_server");
		linebreak_debounce_iterations = 5;
	}

	ros::spin();
	return 0;
}
