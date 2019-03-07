#ifndef PANEL_INTAKE_SERVER
#define PANEL_INTAKE_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <panel_intake_controller/PanelIntakeSrv.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/ElevatorAction.h>
#include "behaviors/enumerated_elevator_indices.h"

//define global variables that will be defined based on config values
double elevator_timeout;
double pause_time_after_extend;
double pause_time_after_clamp;
double dist_up_after;
double wait_for_server_timeout;

class IntakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
		std::string action_name_;

		behaviors::IntakeFeedback feedback_;
		behaviors::IntakeResult result_;

		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::ServiceClient panel_controller_client_;

		ros::Subscriber GoalDetectSub_;

	public:
		IntakeHatchPanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&IntakeHatchPanelAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true)
	{
		//GoalDetectSub_ = nh_.subscribe("goal_detect_msg",1, &IntakeHatchPanelAction::goalDetectCallback, this) //TODO make sure this is linked up correctly
		/* std::map<std::string, std::string> service_connection_header;
		   service_connection_header["tcp_nodelay"] = "1";
		   ElevatorSrv_ = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
		 */
		as_.start();

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		panel_controller_client_ = nh_.serviceClient<panel_intake_controller::PanelIntakeSrv>("/frcrobot_jetson/panel_intake_controller/panel_command", false, service_connection_header);
	}

		~IntakeHatchPanelAction(void) {}

		void executeCB(const behaviors::IntakeGoalConstPtr &/*goal*/)
		{
			ROS_WARN("hatch panel intake server running");

			//make sure the elevator server exists
			bool elevator_server_found = ac_elevator_.waitForServer(ros::Duration(wait_for_server_timeout));

			if(!elevator_server_found)
			{
				ROS_ERROR_STREAM("The elevator server was not loaded before the panel intake server needed it");
				as_.setPreempted();
				return;
			}

			ros::Rate r(10);
			//define variables that will be re-used for each call to a controller
			double start_time = ros::Time::now().toSec();

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;

			//move elevator to intake location
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = INTAKE;
			elev_goal.place_cargo = false;
			elev_goal.raise_intake_after_success = true;
			ac_elevator_.sendGoal(elev_goal);

			bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(elevator_timeout - (ros::Time::now().toSec() - start_time)));
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

			//send commands to panel_intake_controller to grab the panel ---------------------------------------
			if(!preempted && !timed_out && ros::ok())
			{
				//extend panel mechanism
				panel_intake_controller::PanelIntakeSrv srv;
				srv.request.claw_release = true;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv))
				{
					ROS_ERROR("Panel controller call failed in panel intake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything


				//pause for a bit
				ros::Duration(pause_time_after_extend).sleep();


				//grab the panel - we can reuse the srv variable
				srv.request.claw_release = false;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Panel controller call failed in panel intake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything


				//pause for a bit
				ros::Duration(pause_time_after_clamp).sleep();
				//
				//move elevator up a bit after
				behaviors::ElevatorGoal elev_goal;
				elev_goal.setpoint_index = CARGO_SHIP; //TODO fix this add to enum in include file
				elev_goal.place_cargo = false;
				elev_goal.raise_intake_after_success = true;
				ac_elevator_.sendGoal(elev_goal);


				finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(elevator_timeout - (ros::Time::now().toSec() - start_time)));
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

			}

			//Set final state - retract the panel mechanism and clamp (to stay within frame perimeter)
			//it doesn't matter if preempted or timed out, do this anyway
			panel_intake_controller::PanelIntakeSrv srv;
			srv.request.claw_release = false;
			srv.request.push_extend = false;
			//send request to controller
			if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
			{
				ROS_ERROR("Panel controller call failed in panel intake server");
				preempted = true;
			}
			ros::spinOnce(); //update everything

			//log state of action and set result of action
			result_.timed_out = timed_out;

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
				as_.setPreempted(result_);
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result_.success = true;
				as_.setSucceeded(result_);
			}
			return;

		}
		/*
		//TODO: get message type
		goalDetectCallback(msg type here)
		{

		}
		 */
};


int main(int argc, char** argv)
{
	//create node
	ros::init(argc, argv, "panel_intake_server");

	IntakeHatchPanelAction intake_hatch_panel_server("intake_hatch_panel_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_intake_params");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in panel_intake_sever");
	if (!n_panel_params.getParam("elevator_timeout", elevator_timeout))
		ROS_ERROR("Could not read elevator_timeout in panel_intake_sever");
	if (!n_panel_params.getParam("pause_time_after_extend", pause_time_after_extend))
		ROS_ERROR("Could not read pause_time_after_extend in panel_intake_sever");
	if (!n_panel_params.getParam("pause_time_after_clamp", pause_time_after_clamp))
		ROS_ERROR("Could not read pause_time_after_clamp in panel_intake_sever");
	if (!n_panel_params.getParam("dist_up_after", dist_up_after))
		ROS_ERROR("Could not read dist_up_after in panel_intake_sever");

	ros::spin();

	return 0;
}

#endif
