#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/PlaceAction.h"
#include "controllers_2019/CargoIntakeSrv.h"
#include "sensor_msgs/JointState.h"
#include "behaviors/enumerated_elevator_indices.h"

//define global variables that will be defined based on config values
double outtake_timeout; //timeout for the entire action
double elevator_timeout; //timeout for the elevator call
double linebreak_debounce_iterations;

double pause_time_between_pistons;
double wait_for_server_timeout;
double roller_power;
double holding_power;
double pause_before_elevator_lower; //after the outtake

int linebreak_true_count = 0;
int linebreak_false_count = 0;


	/* Place
	 * request is SETPOINT (cargo, rocket1, rocket2, rocket3) and PLACE_CARGO (hatch if false)
	 */

	class CargoOuttakeAction {
		protected:
			ros::NodeHandle nh_;
			actionlib::SimpleActionServer<behaviors::PlaceAction> as_; //create the actionlib server
			std::string action_name_;

			actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

			ros::ServiceClient cargo_intake_controller_client_; //create a ros client to send requests to the controller

		public:
			//make the executeCB function run every time the actionlib server is called
			CargoOuttakeAction(const std::string &name) :
			as_(nh_, name, boost::bind(&CargoOuttakeAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true) //TODO make sure this is linked up correctly
	{
		as_.start(); //start the actionlib server
			//do networking stuff?
			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";

			//initialize the client being used to call the controller
			cargo_intake_controller_client_ = nh_.serviceClient<controllers_2019::CargoIntakeSrv>("/frcrobot_jetson/cargo_intake_controller/cargo_intake_command", false, service_connection_header);
	}

		~CargoOuttakeAction(void)
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::PlaceGoalConstPtr &goal) {
			ROS_WARN_STREAM("Running callback " << action_name_.c_str());

			//wait for all actionlib servers we need
			if(!ac_elevator_.waitForServer(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Cargo outtake server couldn't find elevator server");
				as_.setPreempted();
				return;
			}

			//wait for all controller services we need
			if(! cargo_intake_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Outtake cargo server can't find cargo_intake_controller");
				as_.setPreempted();
				return;
			}


			ros::Rate r(10);

			//define variables that will be reused for each controller call/actionlib server call
			double start_time = ros::Time::now().toSec();

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool success = false; //if controller/actionlib server call succeeded
			bool preempted = false;
			bool timed_out = false;

			//make sure cargo mech is in the up position
			//define request
			ROS_WARN_STREAM("Setting initial state to cargo mech in outtake server");
			controllers_2019::CargoIntakeSrv srv;
			srv.request.power = holding_power;
			srv.request.intake_arm = false;
			//send request to controller
			if(!cargo_intake_controller_client_.call(srv))
			{
				ROS_ERROR("Cargo outtake server: controller call failed to lift arm up");
				preempted = true;
			}

			//send elevator to outtake position
			if(!preempted && !timed_out)
			{
				ROS_WARN_STREAM("cargo outtake server: sending elevator to outtake setpoint");
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.setpoint_index = goal->setpoint_index;
				elevator_goal.place_cargo = true;
				ac_elevator_.sendGoal(elevator_goal);

				bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(std::max(elevator_timeout - (ros::Time::now().toSec() - start_time), 0.001))); //Wait for server to finish or until timeout is reached
				if(finished_before_timeout) {
					actionlib::SimpleClientGoalState state = ac_elevator_.getState();
					if(state.toString() != "SUCCEEDED") {
						ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
						//preempted = true;
					}
					else {
						ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
					}
				}
				else {
					ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
					//timed_out = true;
				}
			}
			if(goal->setpoint_index == CARGO_SHIP && !preempted && !timed_out) {
				ROS_WARN("Moving arm joint down before launching for cargo ship");
				srv.request.power = holding_power;
				srv.request.intake_arm = true;
				if(!cargo_intake_controller_client_.call(srv))
				{
					ROS_ERROR("Cargo outtake server: could not move arm before launching");
					preempted = true;
				}
				ros::Duration(pause_time_between_pistons).sleep();
			}

			//send command to launch cargo ----
			ROS_WARN_STREAM("Cargo outtake server: launching cargo");
			if(!preempted && !timed_out)
			{
				srv.request.power = roller_power;
				srv.request.intake_arm = true;

				if(!cargo_intake_controller_client_.call(srv))
				{
					ROS_ERROR("Cargo outtake server: could not roll out");
					preempted = true;
				}
				//update everything by doing spinny stuff
                while(!timed_out && !preempted && ros::ok()) {
					ROS_WARN_STREAM_THROTTLE(.25, "Cargo outtake server: continuing to launch");
                    timed_out = (ros::Time::now().toSec() - start_time) > outtake_timeout;
                    if(as_.isPreemptRequested() || !ros::ok()) {
                        ROS_WARN(" %s: Preempted", action_name_.c_str());
                        preempted = true;
                    }
					r.sleep();
                }
				ros::spinOnce(); // TODO - maybe inside the while() loop?
			}

			//make sure outtake really happened; pause before lowering elevator
			double time_before_pause = ros::Time::now().toSec();
			while(pause_before_elevator_lower - (ros::Time::now().toSec() - time_before_pause) > 0 && !preempted && ros::ok()) {
				ROS_WARN_STREAM_THROTTLE(.25, "Cargo outtake server: waiting before lowering elevator");
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_WARN(" %s: Preempted", action_name_.c_str());
					preempted = true;
				}
				r.sleep();
			}

			if(!preempted) {
				//move elevator down to end setpoint
				ROS_WARN_STREAM("cargo outtake server: elevator down after placing");
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.setpoint_index = goal->end_setpoint_index;
				elevator_goal.place_cargo = true;
				ac_elevator_.sendGoal(elevator_goal);

				bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(std::max(outtake_timeout - (ros::Time::now().toSec() - start_time), 0.001))); //Wait for server to finish or until timeout is reached
				if(finished_before_timeout) {
					actionlib::SimpleClientGoalState state = ac_elevator_.getState();
					if(state.toString() != "SUCCEEDED") {
						ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
						//preempted = true;
					}
					else {
						ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
					}
				}
				else {
					ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
					//timed_out = true;
				}
			}

			//set ending state of controller no matter what happened: not rolling and arm up

			ROS_WARN_STREAM("Reseting cargo" << action_name_.c_str());

			//define request to send to controller
			srv.request.power = 0;
			srv.request.intake_arm = false;

			//send request to controller
            if(!cargo_intake_controller_client_.call(srv))
			{
				ROS_ERROR("%s: Srv outtake call failed", action_name_.c_str());
				preempted = true;
			}

			//log state of action and set result of action
			behaviors::PlaceResult result; //variable to store result of the actionlib action
			result.timed_out = timed_out; //timed_out refers to last controller call, but applies for whole action
			result.success = success; //success refers to last controller call, but applies for whole action

			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
				as_.setSucceeded(result);
			}
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				as_.setPreempted(result);
			}
			else //implies succeeded
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				as_.setSucceeded(result);
			}

			return;
		}

};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "cargo_outtake_server");

	//create the cargo intake actionlib server
	CargoOuttakeAction cargo_outtake_action("cargo_outtake_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params_outtake(n, "actionlib_cargo_outtake_params");
	ros::NodeHandle n_params_lift(n, "actionlib_lift_params");

	if (!n.getParam("/actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_sever");
	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in intake_sever");

	if (!n.getParam("/cargo_intake/actionlib_cargo_intake_params/holding_power", holding_power))
        ROS_ERROR("Could not read holding_power in cargo_outtake_server");
	ROS_WARN("Holding power in cargo outtake server: %f", holding_power);

	if (!n_params_outtake.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_outtake_server");
	if (!n_params_outtake.getParam("outtake_timeout", outtake_timeout))
		ROS_ERROR("Could not read outtake_timeout in cargo_outtake_server");
	if (!n_params_outtake.getParam("elevator_timeout", elevator_timeout))
		ROS_ERROR("Could not read elevator_timeout in cargo_elevator_server");
	if (!n_params_outtake.getParam("pause_time_between_pistons", pause_time_between_pistons))
		ROS_ERROR("Could not read  pause_time_between_pistons in cargo_outtake_server");

	ros::spin();
	return 0;
}
