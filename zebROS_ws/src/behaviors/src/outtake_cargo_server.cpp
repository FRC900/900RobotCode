#ifndef cargo_outtake_server
#define cargo_outtake_server

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/PlaceAction.h"
#include "cargo_intake_controller/CargoIntakeSrv.h"
#include "elevator_controller/ElevatorSrv.h"
#include "sensor_msgs/JointState.h"
#include <atomic>
#include <ros/console.h>
#include "behaviors/enumerated_elevator_indices.h"

//define global variables that will be defined based on config values

double outtake_timeout; //timeout for the elevator call
double linebreak_debounce_iterations;
double pause_time_between_pistons;
double roller_power;

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
		behaviors::PlaceResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		//ros::Subscriber joint_states_sub_;

	public:
		//make the executeCB function run every time the actionlib server is called
		CargoOuttakeAction(const std::string &name) :
			as_(nh_, name, boost::bind(&CargoOuttakeAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("elevator_server", true) //TODO make sure this is linked up correctly
	{
		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";


		//initialize the client being used to call the controller
		ros::ServiceClient cargo_intake_controller_client_ = nh_.serviceClient<cargo_intake_controller::CargoIntakeSrv>("/frcrobot_jetson/cargo_intake_controller/cargo_intake_command", false, service_connection_header);
        cargo_intake_controller_client_.waitForExistence();
		
		//start subscribers subscribing
		//joint_states_sub_ = nh_.subscribe("/frcrobot/joint_states", 1, &CargoOuttakeAction::jointStateCallback, this);
	}

		~CargoOuttakeAction(void)
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::PlaceGoalConstPtr &goal) {
			ROS_INFO_STREAM("%s: Running callback" << action_name_.c_str());
			ros::Rate r(10);

			//define variables that will be reused for each controller call/actionlib server call
			double start_time = ros::Time::now().toSec();

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool success = false; //if controller/actionlib server call succeeded
			bool preempted = false;
			bool timed_out = false;
			
			//send elevator to outtake position
			if(!preempted && !timed_out) {
				ROS_WARN_STREAM("cargo outtake server: sending elevator to outtake config");
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.setpoint_index = goal->setpoint_index;
				elevator_goal.place_cargo = true;
				ac_elevator_.sendGoal(elevator_goal);

				bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(std::max(outtake_timeout - (ros::Time::now().toSec() - start_time), 0.001))); //Wait for server to finish or until timeout is reached
				if(finished_before_timeout) {
					actionlib::SimpleClientGoalState state = ac_elevator_.getState();
					if(state.toString().c_str() != "SUCCEEDED") {
						ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
						preempted = true;
					}
					else {
						ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
						success = true;
					}
				}
				else {
					ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
					//timed_out = true;
					//preempted = true;
				}
			}
			
			//send command to launch cargo ----
			if(!preempted && !timed_out)
			{
				ROS_WARN("%s: unclamping cargo", action_name_.c_str());

				//define request to send to controller
				cargo_intake_controller::CargoIntakeSrv srv;
                srv.request.power = roller_power;
                srv.request.intake_arm = false;

				//send request to controller
				if(!cargo_intake_controller_client_.call(srv))
				{
					ROS_ERROR("%s: Srv outtake call failed", action_name_.c_str());
					preempted = true;
				}
				//update everything by doing spinny stuff
                while(!timed_out && !preempted && ros::ok()) {
                    timed_out = (ros::Time::now().toSec() - start_time) > outtake_timeout;
                    if(as_.isPreemptRequested() || !ros::ok()) {
                        ROS_WARN(" %s: Preempted", action_name_.c_str());
                        preempted = true;
                    }
					r.sleep();
                }
				ros::spinOnce();
			}
			

			//set ending state of controller no matter what happened: unclamped and kicker not in kicking position
			cargo_intake_controller::CargoIntakeSrv srv;
			srv.request.power = 0;
			srv.request.intake_arm = false;

			//send request to controller
        if(!cargo_intake_controller_client_.call(srv))
			{
				ROS_ERROR("%s: Srv outtake call failed", action_name_.c_str());
				preempted = true;
			}

			//log state of action and set result of action
			
			result_.timed_out = timed_out; //timed_out refers to last controller call, but applies for whole action
			result_.success = success; //success refers to last controller call, but applies for whole action

			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
				as_.setSucceeded(result_);
			}
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				as_.setPreempted(result_);
			}
			else //implies succeeded
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				as_.setSucceeded(result_);
			}
			
			return;
		}

		// Function to be called whenever the subscriber for the joint states topic receives a message
		// Grabs various info from hw_interface using
		// dummy joint position values
		/*void jointStateCallback(const sensor_msgs::JointState &joint_state)
		{
			//get index of linebreak sensor for this actionlib server
			static size_t linebreak_idx = std::numeric_limits<size_t>::max();
			if ((linebreak_idx >= joint_state.name.size()))
			{
				for (size_t i = 0; i < joint_state.name.size(); i++)
				{
					if (joint_state.name[i] == "cargo_outtake_line_break") //TODO: define this in the hardware interface
						linebreak_idx = i;
				}
			}

			//update linebreak counts based on the value of the linebreak sensor
			if (linebreak_idx < joint_state.position.size())
			{
				bool linebreak_true = (joint_state.position[linebreak_idx] != 0);
				if(linebreak_true)
				{
					linebreak_true_count_ += 1;
					linebreak_false_count_ = 0;
				}
				else
				{
					linebreak_true_count_ = 0;
					linebreak_false_count_ += 1;
				}
			}
			else
			{
				static int count = 0;
				if(count % 100 == 0)
				{
					ROS_WARN("outtake line break sensor not found in joint_states");
				}
				count++;
				linebreak_true_count_ = 0;
				linebreak_false_count_ += 1;
			}
		}*/
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "cargo_outtake_server");

	//create the cargo intake actionlib server
	CargoOuttakeAction cargo_outtake_action("cargo_outtake_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params_outtake(n, "actionlib_cargo_outtake_params");
	ros::NodeHandle n_params_intake(n, "actionlib_cargo_intake_params");
	ros::NodeHandle n_params_lift(n, "actionlib_lift_params");

	if (!n.getParam("/teleop/teleop_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_server");

	if (!n_params_outtake.getParam("outtake_timeout", outtake_timeout))
		ROS_ERROR("Could not read outtake_timeout in cargo_outtake_server");
	
	if (!n_params_outtake.getParam("pause_time_between_pistons", pause_time_between_pistons))
		ROS_ERROR("Could not read  pause_time_between_pistons in cargo_outtake_server");

    if(!n_params_outtake.getParam("roller_power", roller_power))
        ROS_ERROR("Could not read roller_power in cargo_outtakeserver");



	ros::spin();
	return 0;
}

#endif
