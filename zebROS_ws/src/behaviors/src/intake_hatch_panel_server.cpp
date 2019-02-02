#ifndef CARGO_INTAKE_SERVER
#define CARGO_INTAKE_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <panel_intake_controller/PanelIntakeSrv.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/PathAction.h>
#include <behaviors/ElevatorAction.h>
#include "behaviors/enumerated_elevator_indices.h"

//define global variables that will be defined based on config values
double linebreak_debounce_iterations;
double intake_timeout;
double pause_time_between_pistons

class IntakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
		std::string action_name_;

		behaviors::IntakeFeedback feedback_;
		behaviors::IntakeResult result_;

		actionlib::SimpleActionClient<behaviors::ElevatorAction> ae_;
		actionlib::SimpleActionClient<behaviors::PathAction> ap_;

		ros::Subscriber GoalDetectSub_;

	public:
		IntakeHatchPanel(std::string name) :
			as_(nh_, name, boost::bind(&IntakeHatchPanel::executeCB, this, _1), false),
			action_name_(name),
			ae_("move_elevator_server", true),
			ap_("path_to_goal", true)
	{
		GoalDetectSub_ = nh_.subscribe("goal_detect_msg",1, &IntakeHatchPanelAction::goalDetectCallback, this) //TODO make sure this is linked up correctly
			/* std::map<std::string, std::string> service_connection_header;
			   service_connection_header["tcp_nodelay"] = "1";
			   ElevatorSrv_ = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
			   IntakeSrv_ = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
			   ClampSrv_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
			   HighCube_ = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeStateCallback, this);
			   BottomLimit_ = nh_.subscribe("/frcrobot/elevator_controller/bottom_limit_pivot", 1, &autoAction::bottomLimitCallback, this);*/

			as_.start();
	}

		~IntakeHatchPanelAction(void) {}

		void executeCB(const behaviors::IntakeHatchPanelGoalConstPtr &goal)
		{
			ros::Rate r(10);
			//define variables that will be re-used for each call to a controller
			double start_time;
			bool success; //if controller/actionlib server call succeeded

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;

			//call the path server to path slightly back from the panel, and the elevator server to go to the correct height for intaking
			if(!preempted && !timed_out)
			{
				//path actionlib server
				behaviors::PathGoal path_goal;
				/*TODO: define path_goal here*/
				ap_.sendGoal(path_goal);
				
				//elevator server
				behaviors::ElevatorGoal elev_goal;
				elev_goal.setpoint_index = goal->setpoint_index; //TODO: figure this out
				elev_goal.place_cargo = false;
				ae_.sendGoal(elev_goal);
				
				ap_.waitForResult(); //waits until the goal finishes, whether a success, timeout, or preempt
				ae_.waitForResult();

				//test if we got a preempt while waiting
				if(as_.isPreemptRequested())
				{
					as_.setPreempted();
					preempted = true;
				}
				else
				{
					
				}
			}

			//call the path server to path to the panel
			if(!preempted && !timed_out)
			{
				behaviors::PathGoal path_goal;
				/*TODO: define path_goal here. Also figure out how we're going to use goal detection to accurately path*/
				ap_.sendGoal(path_goal);
				
				ap_.waitForResult(); //waits until the goal finishes, whether a success, timeout, or preempt
				
				//test if we got a preempt while waiting
				if(as_.isPreemptRequested())
				{
					as_.setPreempted();
					preempted = true;
				}
			}

			//send command to panel_intake_controller to grab the panel ---------------------------------------
			if(!preempted && !timed_out)
			{
				ROS_ERROR("hatch panel intake server running");

				//reset variables
				//linebreak_true_count = 0; //when this gets higher than linebreak_debounce_iterations, we'll consider the gamepiece intaked
				start_time = ros::Time::now().toSec();
				success = false;

				//define request to send to cargo intake controller
				panel_intake_controller::PanelIntakeSrv srv;
				srv.request.claw_in = false; //TODO: make sure this means grab the panel
				srv.request.push_in = true; //this too
				//srv.request.wedge_in = false; //this too
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Srv intake call failed in auto interpreter server intake");
				}
				//update everything by doing spinny stuff
				ros::spinOnce();

				//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
				//
				ros::Duration(0.5).sleep();//sleeps for 0.5

					/*	while(!success && !timed_out && !preempted) {
					success = linebreak_true_count > linebreak_debounce_iterations;
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						preempted = true;
					}
					if (!preempted) {
						r.sleep()r
						ros::spinOnce();
						timed_out = (ros::Time::now().toSec()-start_time) > goal->timeout;
					}
				}
			} */
			//end of code for sending something to a controller --------------------------------------------------------------------------------

			//call another actionlib server
			/*
			   behaviors::ThingGoal goal;
			   goal.property = value;
			   thing_actionlib_client_.sendGoal(goal);
			   */

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
		//TODO: get message type
		goalDetectCallback(/*message type here*/)
		{
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "intake_hatch_panel_server");

	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "actionlib_hatch_panel_params");

	if(!n.getParam("actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
	{
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_hatch_panel_server");
	}
	if(!n_params.getParam("pause_time_between_pistons", pause_time_between_pistons_))
	{
		ROS_ERROR("Could not read pause_time_between_pistons in intake_hatch_panel_server");
	}

	if(!n_params.getParam("intake_timeout", intake_timeout_))
	{
		ROS_ERROR("Could not read intake_timeout in intake_hatch_panel_server");
	}
	/*
	   if(!n_params_fake.getParam("help_me", intake_timeout));
	   ROS_ERROR("Could not read intake_timeout in intake_hatch_panel_server");
	   if(!n_params_fake.getParam("help_me2", arm_timeout));
	   ROS_ERROR("Could not read arm_timeout in intake_hatch_panel_server");
	   */

	//create the actionlib server from the class defined above
	IntakeHatchPanel intake_hatch_panel_server("intake_hatch_panel_server");
	ros::spin();

	return 0;
}

#endif
