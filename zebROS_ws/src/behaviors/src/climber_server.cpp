#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/ClimbAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/ElevatorGoal.h"
#include "elevator_controller/ElevatorEngageSrv.h"
#include "std_srvs/SetBool.h" //for the climber controller
#include "geometry_msgs/Twist.h" //for the drivebase
#include <atomic>
#include <ros/console.h>

//define global variables that will be defined based on config values

double elevator_deploy_setpoint; //where the elevator needs to go before engaging the climber
double elevator_climb_setpoint; //where the elevator needs to go after engaging the climber


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
		ros::Publisher cmd_vel_pub_;
/*
		std::atomic<int> linebreak_true_count; //counts how many times in a row the linebreak reported there's a cargo since we started trying to intake/outtake
		std::atomic<int> linebreak_false_count; //same, but how many times in a row no cargo
*/		behaviors::ClimbResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		//ros::Subscriber joint_states_sub_;

	public:
		//make the executeCB function run every time the actionlib server is called
		// TODO - ae_ is never set?
		ClimbAction(const std::string &name) :
			as_(nh_, name, boost::bind(&ClimbAction::executeCB, this, _1), false),
			action_name_(name)
	{
		as_.start(); //start the actionlib server

		// TODO : probably want a call to ae_.waitForServer() with a reasonable timeout, either here
		// or in the executeCB function. Check the return code to make sure the actionlib server
		// exists before calling it.

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		// TODO : call waitForExistence with a reasonable timeout here, again to make
		// sure that the requested services exist. Useful for initially hooking things
		// up to the correct names
		
		//initialize the client being used to call the climber controller
		climber_controller_client_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/climber_controller/climber_service", false, service_connection_header);
		//initialize the client being used to call the elevator controller to engage the climber
		climber_engage_client_ = nh_.serviceClient<elevator_controller::EngageClimberSrv>("/frcrobot_jetson/elevator_controller/climber_engage_service", false, service_connection_header);

		// TODO : consider just using 1 instead of 10.
		//initialize the publisher used to send messages to the drive base
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/frcrobot_jetson/swerve_drive_controller/cmd_vel",10);

		//start subscribers subscribing
		//joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &ClimbAction::jointStateCallback, this);
	}

		~ClimbAction(void) 
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::ClimbGoalConstPtr &goal) {
			/* Steps to climb:
			 * 1. Deploy foot via climber controller
			 * 2. Raise elevator to right height via elevator actionlib server
			 * 3. Deploy climber engagement piston via elevator controller
			 * 4. Lower elevator to right height to make the robot climb (this should be slow... change pid somehow?)
			 *
			 * Following steps will not be run by this server
			 * 5. Driver presses button that calls elevator controller to retract climber foot to fall onto platform
			 * 6. Driver preempts this server to stop drivebase rolling forward, and to pull the climber leg up a bit - this is final state of this server
			*/
			ros::Rate r(10);

			//define variables that will be reused for each controller call/actionlib server call
			double start_time;
			bool success; //if controller/actionlib server call succeeded, the actionlib server will output the latest value of this at the end - if the last action succeeded, we infer everything did

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;
			
			//deploy foot using climber controller -----------------------------------------------
			success = false; //didn't succeed yet
			//define service to send
			ROS_ERROR("climber server: deploying foot");
			std_msgs::SetBool srv;
			srv.data = true; //TODO: check this
			//call controller
			if(!climber_controller_client_.call(srv))
			{
				ROS_ERROR("Foot deploy failed in climber controller");
				preempted = true;
			}
			else {
				success = true;
			}
			ros::spinOnce();

			// raise elevator to right height so we can engage the climber ------------------------------------------------
			if(!preempted && !timed_out)
			{
				// TODO : don't use ROS_ERROR for info messages, use ROS_INFO instead
				ROS_ERROR("climber server: raising elevator before climber is engaged");
				
				success = false;

				//call the elevator actionlib server
				//define the goal to send
				behaviors::ElevatorGoal goal;
				goal.elevator_setpoint = elevator_deploy_setpoint; //elevator_deploy_setpoint is defined via config values
				//send the goal
				ae_.sendGoal(goal);
				// TODO : probably should pass in a (configurable) timeout, then
				// check the return code to see if that has expired. If so, error out
				ae_.waitForResult(); //wait until the action finishes, whether it succeeds, times out, or is preempted

				//determine the outcome of the goal
				behaviors::ElevatorResult elev_result;
				elev_result = *ae_.getResult();
				if(elev_result.success)
				{
					success = true;
				}
				else {
					preempted = true;
				}

				//check if we got a preempt while we were waiting
				if(as_.isPreemptRequested())
				{
					preempted = true;
				}
			} //end of raise elevator to right height before engaging

			//engage climber with elevator controller -----------------------------------------------------------------
			if(!preempted && !timed_out)
			{
				ROS_ERROR("climber server: engaging the climber");

				success = false;

				//call the elevator controller
				//define the goal to send
				elevator_controller::ClimberEngageSrv srv;
				srv.engage = true;
				//call controller
				if(!climber_engage_client_.call(srv))
				{
					ROS_ERROR("Climber engage failed in climber controller");
					preempted = true;
				}
				else {
					success = true;
				}
				ros::spinOnce();
			}
			// TODO : any need for a delay between deploying feet and 
			// running elevator?

			//lower elevator to make robot rise off ground
			if(!preempted && !timed_out)
			{
				ROS_ERROR("climber server: lowering elevator to make robot climb");
				
				success = false;

				//call the elevator actionlib server
				//define the goal to send
				behaviors::ElevatorGoal goal;
				goal.elevator_setpoint = elevator_climb_setpoint; //elevator_climb_setpoint is defined via config values
				//send the goal
				ae_.sendGoal(goal);
				ae_.waitForResult(); //wait until the action finishes, whether it succeeds, times out, or is preempted

				//determine the outcome of the goal
				// TODO : could just replace these 3 lines with
				// if (ae_.getResult()->success)
				behaviors::ElevatorResult elev_result;
				elev_result = *ae_.getResult();
				if(elev_result.success)
				{
					success = true;
				}
				else {
					preempted = true;
				}

				//check if we got a preempt while we were waiting
				if(as_.isPreemptRequested())
				{
					preempted = true;
				}
			} //end of lowering elevator to make robot climb

			//Start the wheels turning slowly at this point
			//The driver will determine when to make the robot fall onto the platform. That's handled outside this server.
			//When the robot has driven forward sufficiently, the driver will preempt this server to stop the driving and to pull the climber up
			
			//spin wheels forward slowly and wait for the driver to preempt this actionlib server
			while(!preempted)
			{
				if(as_.isPreemptRequested() || !ros::ok())
				{
					preempted = true;
				}
				else
				{
					//publish move wheels forward slowly msg
					geometry_msgs::Twist msg;
					/*TODO: define msg*/
					// The speed should probably be a config item?
					cmd_vel_pub_.publish(msg);

					// TODO : reverse these.
					r.sleep();
					ros::spinOnce();
				}
			}

			//the same button that preempts this actionlib server will cause the climber to be retracted - handled in a teleop joystick file somewhere


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
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "climb_server");

	//create the cargo intake actionlib server
	ClimbAction climb_action("climb_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_lift_params(n, "actionlib_lift_params");

	if (!n_lift_params.getParam("elevator_deploy_setpoint", elevator_deploy_setpoint))
		ROS_ERROR("Could not read elevator_deploy_setpoint in climber_server");

	if (!n_lift_params.getParam("elevator_climb_setpoint", elevator_climb_setpoint))
		ROS_ERROR("Could not read elevator_climb_setpoint in climber_server");

	ros::spin();
	return 0;
}
