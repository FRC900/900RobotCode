#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/PlaceAction.h"
#include "cargo_intake_controller/CargoIntakeSrv.h"
#include "elevator_controller/ElevatorSrv.h"
#include "sensor_msgs/JointState.h"
#include "cargo_outtake_controller/CargoOuttakeSrv.h"
#include <atomic>
#include <ros/console.h>

//define global variables that will be defined based on config values

double outtake_timeout;
double linebreak_debounce_iterations;

/* Place
 * request is SETPOINT (cargo, rocket1, rocket2, rocket3) and PLACE_CARGO (hatch if false)
 */

class CargoOuttakeAction {
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::PlaceAction> as_; //create the actionlib server
		std::string action_name_;

		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::ServiceClient controller_client_; //create a ros client to send requests to the controller
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
		ros::ServiceClient cargo_outtake_controller_client_ = nh_.serviceClient<cargo_outtake_controller::CargoOuttakeSrv>("/frcrobot_jetson/cargo_outtake_controller/cargo_outtake_command", false, service_connection_header);
		
		//start subscribers subscribing
		joint_states_sub_ = nh_.subscribe("/frcrobot/joint_states", 1, &CargoOuttakeAction::jointStateCallback, this);
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

				bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(max(outtake_timeout - (ros::Time::now().toSec() - start_time), 0.001))); //Wait for server to finish or until timeout is reached
				if(finished_before_timeout) {
					actionlib::SimpleClientGoalState state = ac_elevator_.getState();
					if(state.toString().c_str() != "SUCCEEDED") {
						ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
						as_.setPreempted();
						preempted = true;
					}
					else {
						ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
						success = true;
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
				ROS_WARN("%s: unclamping cargo", action_name_.c_str());

				//define request to send to cargo intake controller
				cargo_outtake_controller::CargoOuttakeSrv srv;
				srv.request.kicker_in = true; 
				srv.request.clamp_in = true;

				//send request to controller
				if(!controller_client_.call(srv))
				{
					ROS_ERROR("%s: Srv outtake call failed", action_name_.c_str());
					as_.setPreempted();
					preempted = true;
				}
				//update everything by doing spinny stuff
				ros::spinOnce();

				//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
				while(!success && !timed_out && !preempted) {
					success = linebreak_true_count_ > linebreak_debounce_iterations;
					timed_out = (ros::Time::now().toSec()-start_time) > outtake_timeout;

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
			cargo_outtake_controller::CargoOuttakeSrv srv;
			srv.request.power = 0;
			srv.request.kicker_in = false;
			srv.request.clamp_in = true;
			//send request to controller
			if(!controller_client_.call(srv))
			{
				ROS_ERROR("Srv outtake call failed in cargo outtake server");
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
			//as_.setSucceeded(result_, "intake_cargo_server: testing send text result from actionlib server... SPOOKY"); //pretend it succeeded no matter what, but tell what actually happened with the result - helps with SMACH
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

	if (!n.getParam("actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_sever");

	if (!n_params_outtake.getParam("outtake_timeout", outtake_timeout))
		ROS_ERROR("Could not read outtake_timeout in cargo_outtake_server");

	ros::spin();
	return 0;
}
