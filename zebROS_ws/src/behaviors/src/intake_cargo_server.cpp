#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/IntakeAction.h"
#include "behaviors/ElevatorAction.h"
#include "cargo_intake_controller/CargoIntakeSrv.h"
#include "sensor_msgs/JointState.h"
#include <atomic>
#include <ros/console.h>
#include "behaviors/enumerated_elevator_indices.h"
#include "cargo_outtake_controller/CargoOuttakeSrv.h"

//define global variables that will be defined based on config values

double roller_power;
double intake_timeout;
double linebreak_debounce_iterations;
double wait_for_server_timeout;
double pause_before_running_motor = 0;

class CargoIntakeAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::IntakeAction> as_; //create the actionlib server
		std::string action_name_;
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::ServiceClient cargo_intake_controller_client_; //create a ros client to send requests to the controller
		ros::ServiceClient cargo_outtake_controller_client_; //create a ros client to send requests to the controller
		std::atomic<int> linebreak_true_count; //counts how many times in a row the linebreak reported there's a cargo since we started trying to intake/outtake
		std::atomic<int> linebreak_false_count; //same, but how many times in a row no cargo
		behaviors::IntakeResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		ros::Subscriber joint_states_sub_;
	public:
		//make the executeCB function run every time the actionlib server is called
		CargoIntakeAction(const std::string &name) :
			as_(nh_, name, boost::bind(&CargoIntakeAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true) //TODO make sure this is linked up correctly
	{
		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		cargo_intake_controller_client_ = nh_.serviceClient<cargo_intake_controller::CargoIntakeSrv>("/frcrobot_jetson/cargo_intake_controller/cargo_intake_command", false, service_connection_header);
		cargo_outtake_controller_client_ = nh_.serviceClient<cargo_outtake_controller::CargoOuttakeSrv>("/frcrobot_jetson/cargo_outtake_controller/cargo_outtake_command", false, service_connection_header);
		//start subscribers subscribing
		joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &CargoIntakeAction::jointStateCallback, this);
	}

		~CargoIntakeAction(void)
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::IntakeGoalConstPtr &goal) {
			ROS_INFO("%s: Running callback", action_name_.c_str());

			//wait for all actionlib servers we need
			if(!ac_elevator_.waitForServer(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Cargo intake server couldn't find elevator server");
				as_.setPreempted();
				return;
			}

			//wait for all controller services we need
			if(! cargo_intake_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Intake cargo server can't find cargo_intake_controller");
				as_.setPreempted();
				return;
			}


			ros::Rate r(100);

			//define variables that will be reused for each controller call/actionlib server call
			double start_time = ros::Time::now().toSec();

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool success = false; //if controller/actionlib server call succeeded
			bool preempted = false;
			bool timed_out = false;

			if(!preempted && !timed_out) {
				ROS_WARN("cargo intake server: sending elevator to intake setpoint");
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.setpoint_index = INTAKE;
				elevator_goal.raise_intake_after_success = false;
				ac_elevator_.sendGoal(elevator_goal);
				bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(intake_timeout - (ros::Time::now().toSec() - start_time))); //Wait for server to finish or until timeout is reached
				if(finished_before_timeout) {
					actionlib::SimpleClientGoalState state = ac_elevator_.getState();
					if(state.toString() != "SUCCEEDED") {
						ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
						preempted = true;
				}
				else {
					ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
				}
			}
			else {
				ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
				timed_out = true;
			}

			//test if we got a preempt while waiting
			if(as_.isPreemptRequested())
			{
				preempted = true;
			}

			//raise cargo clamp
			if(!preempted && !timed_out && ros::ok())
			{
				ROS_WARN("%s: raising cargo clamp", action_name_.c_str());

				cargo_outtake_controller::CargoOuttakeSrv outtake_srv;
				outtake_srv.request.kicker_in = true;
				outtake_srv.request.clamp_release = true;
				cargo_outtake_controller_client_.call(outtake_srv);
				ros::Duration(pause_before_running_motor).sleep();
			}

			//send command to lower arm and run roller to the cargo intake controller ------
			if(!preempted && !timed_out && ros::ok())
			{
				ROS_WARN("%s: lowering arm and spinning roller in",action_name_.c_str());

				//reset variables
				linebreak_true_count = 0; //when this gets higher than linebreak_debounce_iterations, we'll consider the gamepiece intooketh
				success = false;

				//define request to send to cargo intake controller
				cargo_intake_controller::CargoIntakeSrv srv;
				srv.request.power = roller_power;
				srv.request.intake_arm = true; //TODO: double check

				//send request to controller
				if(!cargo_intake_controller_client_.call(srv))
				{
					ROS_ERROR("%s: Srv intake call failed", action_name_.c_str());
					preempted = true;
				}

				//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
				while(!success && !timed_out && !preempted && ros::ok()) {
					success = linebreak_true_count > linebreak_debounce_iterations;
					timed_out = (ros::Time::now().toSec()-start_time) > intake_timeout;

					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_WARN(" %s: Preempted", action_name_.c_str());
						preempted = true;
					}
					else if(!success) 
					{
						r.sleep();
					}
				}
			}


			//end of code for sending something to a controller ----------------------------------

			//set ending state of controller no matter what happened: arm up and roller motors stopped
			//define command to send to cargo intake controller
			cargo_intake_controller::CargoIntakeSrv srv;
			srv.request.power = 0;
			srv.request.intake_arm = false; //TODO: Double check
			//send request to controller
			if(!cargo_intake_controller_client_.call(srv))
			{
				ROS_ERROR("Srv intake call failed in cargo intake server");
			}

			if(success)
			{
				cargo_outtake_controller::CargoOuttakeSrv outtake_srv;
				outtake_srv.request.kicker_in = true;
				outtake_srv.request.clamp_release = false;
				cargo_outtake_controller_client_.call(outtake_srv);
				//call the cargo outtake controller to clamp DOWN
			}

			//log state of action and set result of action
			result_.timed_out = timed_out; //timed_out refers to last controller call, but applies for whole action
			if(timed_out)
			{
				ROS_WARN("%s: Timed Out", action_name_.c_str());
				result_.success = false;
				as_.setSucceeded(result_);
			}
			else if(preempted)
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				result_.success = false;
				as_.setSucceeded(result_);
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result_.success = true;
				as_.setSucceeded(result_);
			}

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
					if (joint_state.name[i] == "cargo_intake_linebreak_1") //TODO: define this in the hardware interface
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

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params_intake(n, "actionlib_cargo_intake_params");

	if (!n.getParam("/actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_sever");
	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in intake_sever");

	if (!n_params_intake.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_intake_server");
	if (!n_params_intake.getParam("intake_timeout", intake_timeout))
		ROS_ERROR("Could not read intake_timeout in cargo_intake_server");
	if (!n_params_intake.getParam("pause_before_running_motor", pause_before_running_motor))
		ROS_ERROR("Could not read pause_before_running_motor in cargo_intake_server");

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}

