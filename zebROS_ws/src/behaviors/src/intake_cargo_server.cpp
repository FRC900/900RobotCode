#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/IntakeAction.h"
#include "behaviors/ElevatorAction.h"
#include "cargo_intake_controller/CargoIntakeSrv.h"
#include "elevator_controller/ElevatorSrv.h"
#include "sensor_msgs/JointState.h"
#include <atomic>
#include <ros/console.h>

//define global variables that will be defined based on config values

double roller_power;
double intake_timeout;
double linebreak_debounce_iterations;
double intake_setpoint;
double cargo_ship_setpoint;
double low_rocket_setpoint;
double mid_rocket_setpoint;
double high_rocket_setpoint;


class CargoIntakeAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::IntakeAction> as_; //create the actionlib server
		std::string action_name_;
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::ServiceClient controller_client_; //create a ros client to send requests to the controller
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
			ac_elevator_("elevator_server", true) //TODO make sure this is linked up correctly
	{
		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize action client to call actionlib server

		//initialize the client being used to call the controller
		ros::ServiceClient cargo_intake_controller_client_ = nh_.serviceClient<cargo_intake_controller::CargoIntakeSrv>("/frcrobot_jetson/cargo_intake_controller/cargo_intake_command", false, service_connection_header);
		ros::ServiceClient elevator_controller_client_ = nh_.serviceClient<elevator_controller::ElevatorSrv>("/frcrobot_jetson/elevator_controller/elevator_service", false, service_connection_header);
		//start subscribers subscribing
		joint_states_sub_ = nh_.subscribe("/frcrobot/joint_states", 1, &CargoIntakeAction::jointStateCallback, this);
	}

		~CargoIntakeAction(void) 
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::IntakeGoalConstPtr &goal) {
			ROS_INFO("%s: Running callback", action_name_.c_str());
			ros::Rate r(10);

			//define variables that will be reused for each controller call/actionlib server call
			double start_time = ros::Time::now().toSec();

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool success = false; //if controller/actionlib server call succeeded
			bool preempted = false;
			bool timed_out = false;

			if(!preempted && !timed_out) {
				ROS_WARN("cargo intake server: sending elevator to intake config");
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.setpoint_index = intake_setpoint;
				ac_elevator_.sendGoal(elevator_goal);
				bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(intake_timeout - (ros::Time::now().toSec() - start_time))); //Wait for server to finish or until timeout is reached
				if(finished_before_timeout) {
					actionlib::SimpleClientGoalState state = ac_elevator_.getState();
					if(state.toString().c_str() != "SUCCEEDED") {
						ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
						as_.setPreempted();
						preempted = true;
					}
					else {
						ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
					}
				}
				else {
					ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
					timed_out = true;
					as_.setPreempted();
				}
			}

			//send command to lower arm and run roller to the cargo intake controller ------
			if(!preempted && !timed_out)
			{
				ROS_WARN("%s: lowering arm and spinning roller in",action_name_.c_str());

				//reset variables
				linebreak_true_count = 0; //when this gets higher than linebreak_debounce_iterations, we'll consider the gamepiece intooketh
				success = false;

				//define request to send to cargo intake controller
				cargo_intake_controller::CargoIntakeSrv srv;
				srv.request.power = roller_power;
				srv.request.intake_arm = false; //TODO: double check

				//send request to controller
				if(!controller_client_.call(srv))
				{
					ROS_ERROR("%s: Srv intake call failed", action_name_.c_str());
					as_.setPreempted();
					preempted = true;
				}
				//update everything by doing spinny stuff
				ros::spinOnce();

				//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
				while(!success && !timed_out && !preempted) {
					success = linebreak_true_count > linebreak_debounce_iterations;
					timed_out = (ros::Time::now().toSec()-start_time) > intake_timeout;

					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_WARN(" %s: Preempted", action_name_.c_str());
						as_.setPreempted();
						preempted = true;
					}
					else {
						r.sleep();
						ros::spinOnce();
					}
				}
			}


			//end of code for sending something to a controller ----------------------------------

			//set ending state of controller no matter what happened: arm up and roller motors stopped
			//define command to send to cargo intake controller
			cargo_intake_controller::CargoIntakeSrv srv;
			srv.request.power = 0;
			srv.request.intake_arm = true; //TODO: Double check
			//send request to controller
			if(!controller_client_.call(srv))
			{
				ROS_ERROR("Srv intake call failed in cargo intake server");
			}
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
			as_.setSucceeded(result_, "intake_cargo_server: testing send text result from actionlib server... SPOOKY"); //pretend it succeeded no matter what, but tell what actually happened with the result - helps with SMACH
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

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params_intake(n, "actionlib_cargo_intake_params");
	ros::NodeHandle n_params_lift(n, "actionlib_lift_params");

	if (!n.getParam("actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_sever");

	if (!n_params_intake.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_intake_server");
	if (!n_params_intake.getParam("intake_timeout", intake_timeout))
		ROS_ERROR("Could not read intake_timeout in cargo_intake_server");

	if(!n_params_lift.getParam("cargo_ship", cargo_ship_setpoint))
		ROS_ERROR("Coult not read cargo_ship in cargo_intake_server");
	if(!n_params_lift.getParam("low_rocket", low_rocket_setpoint))
		ROS_ERROR("Coult not read low_rocket in cargo_intake_server");
	if(!n_params_lift.getParam("high_rocket", high_rocket_setpoint))
		ROS_ERROR("Coult not read high_rocket in cargo_intake_server");
	if(!n_params_lift.getParam("mid_rocket", mid_rocket_setpoint))
		ROS_ERROR("Coult not read mid_rocket in cargo_intake_server");
	if(!n_params_lift.getParam("intake", intake_setpoint))
		ROS_ERROR("Coult not read intake in cargo_intake_server");


	ros::spin();
	return 0;
}
