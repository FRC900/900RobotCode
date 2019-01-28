#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/ClimbAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/ElevatorGoal.h"
#include "elevator_controller/ElevatorSrv.h"
#include "geometry_msgs/Twist.h" //for the drivebase
#include <atomic>
#include <ros/console.h>

//define global variables that will be defined based on config values

double elevator_start_setpoint; //where the elevator needs to go before engaging the climber
double elevator_finish_setpoint; //where the elevator needs to go after engaging the climber
double elevator_timeout; //used for all elevator movements


class ClimbAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::ClimbAction> as_; //create the actionlib server
		std::string action_name_;

		//create clients to call other actionlib servers
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ae_; //to call the elevator

		//create clients/subscribers to activate controllers
		ros::ServiceClient elevator_controller_client_; //create a ros client to send requests to the climber controller (piston in the leg)
		ros::Publisher cmd_vel_pub_;
/*
		std::atomic<int> linebreak_true_count; //counts how many times in a row the linebreak reported there's a cargo since we started trying to intake/outtake
		std::atomic<int> linebreak_false_count; //same, but how many times in a row no cargo
*/		behaviors::ClimbResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		//ros::Subscriber joint_states_sub_;

	public:
		//make the executeCB function run every time the actionlib server is called
		ClimbAction(const std::string &name) :
			as_(nh_, name, boost::bind(&ClimbAction::executeCB, this, _1), false),
			action_name_(name)
	{
		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the climber controller
		elevator_controller_client_ = nh_.serviceClient<elevator_intake_controller::ElevatorSrv>("/frcrobot_jetson/elevator_controller/elevator_service", false, service_connection_header);

		//initialize the publisher used to send messages to the drive base
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/frcrobot_jetson/swerve_drive_controller/cmd_vel",10);

		//start subscribers subscribing
		//joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &CargoIntakeAction::jointStateCallback, this);
	}

		~CargoIntakeAction(void) 
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::IntakeGoalConstPtr &goal) {
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
			
			//deploy foot using climber controller
			success = false; //didn't succeed yet
			//define service to send
			ROS_ERROR("climber server: deploying foot");
			std_msgs::SetBool srv;
			srv.data = true; //TODO: check this
			//call controller
			if(!elevator_controller_client_.call(srv))
			{
				ROS_ERROR("Foot deploy failed in climber controller");
				as_.setPreempted();
				preempted = true;
			}
			else {
				success = true;
			}
			ros::spinOnce();

			// raise elevator to right height so we can engage the climber
			if(!preempted && !timed_out)
			{
				ROS_ERROR("climber server: raising elevator before climber is engaged");
				
				success = false;

				//call the elevator actionlib server
				//define the goal to send
				behaviors::ElevatorGoal goal;
				goal.elevator_setpoint = elevator_start_setpoint; //elevator_start_setpoint is defined via config values
				//send the goal
				ae_.sendGoal(goal);
				ae_.waitForResult(); //wait until the action finishes, whether it succeeds, times out, or is preempted

				//determine the outcome of the goal
				behaviors::ElevatorResult elev_result;
				elev_result = *ae_.getResult();
				if(elev_result.success)
				{
					success = true;
				}
				else {
					as_.setPreempted();
					preempted = true;
				}

				//check if we got a preempt while we were waiting
				if(as_isPreemptRequested())
				{
					as_.setPreempted();
					preempted = true;
				}
			} //end of raise elevator to right height before engaging

			//engage climber
			//I STOPPED WORKING HERE. BELOW IS COPY PASTED -----------------------------------------------------------------------------------

			//set ending state of controller no matter what happened: arm up, roller motors stopped
			//define command to send to cargo intake controller
			cargo_intake_controller;:CargoIntakeSrv srv;
			srv.request.power = 0;
			srv.request.intake_arm = true; //TODO: Double check
			//send request to controller
			if(!controller_client_.call(srv))
			{
				ROS_ERROR("Srv intake call failed in cargo intake server");
			}
			//update everything by doing spinny stuff
			ros::spinOnce();


			//log state of action and set result of action
			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else //implies succeeded
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			result_.timed_out = timed_out; //timed_out refers to last controller call, but applies for whole action
			result_.success = success; //success refers to last controller call, but applies for whole action
			as_.setSucceeded(result_); //pretend it succeeded no matter what, but tell what actually happened with the result - helps with SMACH
			return;
		}

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
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "cargo_intake_server");

	//create the cargo intake actionlib server
	CargoIntakeAction cargo_intake_action("cargo_intake_server");
/*
	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "actionlib_cargo_intake_params");

	if (!n_params.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_intake_server");

	if (!n_params.getParam("intake_timeout", intake_timeout))
		ROS_ERROR("Could not read intake_timeout in cargo_intake_server");

	if (!n.getParam("actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_sever");
*/
	ros::spin();
	return 0;
}
