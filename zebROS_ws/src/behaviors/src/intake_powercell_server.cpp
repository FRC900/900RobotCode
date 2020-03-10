#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behavior_actions/IntakeAction.h"
#include "behavior_actions/IndexerAction.h"
#include "controllers_2020_msgs/IntakeArmSrv.h"
#include "controllers_2020_msgs/IntakeRollerSrv.h"
#include "sensor_msgs/JointState.h"
#include <atomic>

#include "behavior_actions/enumerated_indexer_actions.h"

#include "behaviors/linebreak.h"

//define global variables that will be defined based on config values

class PowerCellIntakeAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::IntakeAction> as_; //create the actionlib server
		std::string action_name_;

		//create ros clients to send requests to the controller
		ros::ServiceClient intake_arm_controller_client_;
		ros::ServiceClient intake_roller_controller_client_;

		//create actionlib clients
		actionlib::SimpleActionClient<behavior_actions::IndexerAction> ac_indexer_;

		//intake linebreak (true if ball in the intake)
		Linebreak intake_linebreak_{"intake_linebreak"};

		//define variables that will be set true if the actionlib action is to be ended
		//this will cause subsequent controller calls to be skipped
		bool preempted_ = false;
		bool timed_out_ = false;
		double start_time_;

	public:
		//config variables
		double roller_percent_out_;
		double server_timeout_;
		double wait_for_server_timeout_;

		//make the executeCB function run every time the actionlib server is called
		PowerCellIntakeAction(const std::string &name) :
			as_(nh_, name, boost::bind(&PowerCellIntakeAction::executeCB, this, _1), false),
			action_name_(name),
			ac_indexer_("/indexer/indexer_server", true)
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		intake_arm_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IntakeArmSrv>("/frcrobot_jetson/intake_controller/intake_arm_command", false, service_connection_header);
		intake_roller_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IntakeRollerSrv>("/frcrobot_jetson/intake_controller/intake_roller_command", false, service_connection_header);
	}

		~PowerCellIntakeAction(void)
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::IntakeGoalConstPtr &/*goal*/)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());
			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;


			//wait for all actionlib servers we need
			if(!ac_indexer_.waitForServer(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " couldn't find indexer actionlib server");
				as_.setPreempted();
				return;
			}

			//wait for all controller services we need
			if(! intake_arm_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM("Intake powercell server can't find powercell intake controller's arm ROS server");
				as_.setPreempted();
				return;
			}
			if(! intake_roller_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM("Intake powercell server can't find powercell intake controller's roller ROS server");
				as_.setPreempted();
				return;
			}


			//tell indexer actionlib server to move to position intake
			behavior_actions::IndexerGoal indexer_goal;
			indexer_goal.action = POSITION_INTAKE;
			ac_indexer_.sendGoal(indexer_goal);
			//don't wait for it to finish, we need to start intaking immediately - moving to position intake is basically a 2nd priority thing. If it fails it's (hopefully) ok, we were going to cancel it anyways when we're ready to call the indexer server to intake a ball



			//send command to lower arm and run roller to the powercell intake controller ------
			ROS_WARN("%s: lowering arm and spinning roller in",action_name_.c_str());
			//define requests to send to powercell intake controller
			controllers_2020_msgs::IntakeArmSrv arm_srv;
			arm_srv.request.intake_arm_extend = true;

			controllers_2020_msgs::IntakeRollerSrv roller_srv;
			roller_srv.request.percent_out = roller_percent_out_;

			//send requests to controller
			if(!intake_arm_controller_client_.call(arm_srv))
			{
				ROS_ERROR("%s: powercell intake controller call to arm failed when starting the intake", action_name_.c_str());
				preempted_ = true;
			}
			if(!intake_roller_controller_client_.call(roller_srv))
			{
				ROS_ERROR("%s: powercell intake controller call to roller failed when starting the intake", action_name_.c_str());
				preempted_ = true;
			}

			//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
			ros::Rate r(10);
			while(!timed_out_ && !preempted_ && ros::ok())
			{
				if(intake_linebreak_.triggered_)
				{
					//call indexer actionlib server to intake the ball
					//sending a new call to the indexer actionlib will cancel it if it is already running
					behavior_actions::IndexerGoal indexer_goal;
					indexer_goal.action = INTAKE_ONE_BALL;
					ac_indexer_.sendGoal(indexer_goal);
					waitForActionlibServer(ac_indexer_, 10, "waiting for indexer actionlib server to intake a ball");
					//if it finishes successfully, this while loop will keep running, allowing multiple balls to be processed
				}

				timed_out_ = (ros::Time::now().toSec()-start_time_) > server_timeout_;

				if(as_.isPreemptRequested() || !ros::ok()) { //intended stopping mechanism is the driver preempting this actionlib server
					ROS_WARN(" %s: preempted durng main while loop", action_name_.c_str());
					preempted_ = true;
				}
				else
				{
					r.sleep();
				}
			}

			//set ending state of controller: roller motors stopped
			/*arm_srv.request.intake_arm_extend = false;
			if(!preempted_)
			{
				if(!intake_arm_controller_client_.call(arm_srv))
				{
					ROS_ERROR("%s: powercell intake controller call to arm failed when setting final state", action_name_.c_str());
				}
			}*/
			roller_srv.request.percent_out = 0;
			if(!intake_roller_controller_client_.call(roller_srv))
			{
				ROS_ERROR("%s: powercell intake controller call to roller failed when setting final state", action_name_.c_str());
			}

			//log state of action and set result of action
			behavior_actions::IntakeResult result; //variable to store result of the actionlib action
			result.timed_out = timed_out_; //timed_out_ refers to last controller call, but applies for whole action
			if(timed_out_)
			{
				ROS_WARN("%s: Timed Out", action_name_.c_str());
				result.success = true; //timed out is encoded as succeeded b/c actionlib has no state for timed out
				as_.setSucceeded(result);
			}
			else if(preempted_)
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				result.success = false;
				as_.setPreempted(result);
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result.success = true;
				as_.setSucceeded(result);
			}

			return;
		}


		void waitForActionlibServer(auto &action_client, double timeout, const std::string &activity)
			//activity is a description of what we're waiting for, e.g. "waiting for mechanism to extend" - helps identify where in the server this was called (for error msgs)
		{
			double request_time = ros::Time::now().toSec();
			ros::Rate r(10);

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
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
					preempted_ = true;
				}
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out during " << activity);
					timed_out_ = true;
					action_client.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
				else { //if didn't succeed and nothing went wrong, keep waiting
					ros::spinOnce();
					r.sleep();
				}
			}
		}
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "powercell_intake_server");

	//create the powercell intake actionlib server
	PowerCellIntakeAction powercell_intake_action("powercell_intake_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params_intake(n, "actionlib_powercell_intake_params");

	if (!n_params_intake.getParam("wait_for_server_timeout", powercell_intake_action.wait_for_server_timeout_)) {
		ROS_ERROR("Could not read wait_for_server_timeout in intake_sever");
		powercell_intake_action.wait_for_server_timeout_ = 10;
	}
	if (!n_params_intake.getParam("roller_percent_out", powercell_intake_action.roller_percent_out_)) {
		ROS_ERROR("Could not read roller_percent_out in powercell_intake_server");
		powercell_intake_action.roller_percent_out_ = 0.6;
	}
	if (!n_params_intake.getParam("server_timeout", powercell_intake_action.server_timeout_)) {
		ROS_ERROR("Could not read server_timeout in powercell_intake_server");
		powercell_intake_action.server_timeout_ = 50;
	}

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
