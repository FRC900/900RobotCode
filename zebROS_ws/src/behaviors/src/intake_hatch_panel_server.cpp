#ifndef PANEL_INTAKE_SERVER
#define PANEL_INTAKE_SERVER

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
double pause_time_between_pistons;
double wait_for_server_timeout;

class IntakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
		std::string action_name_;

		behaviors::IntakeFeedback feedback_;
		behaviors::IntakeResult result_;

		actionlib::SimpleActionClient<behaviors::ElevatorAction> ae_;

		ros::Subscriber GoalDetectSub_;

	public:
		IntakeHatchPanel(std::string name) :
			as_(nh_, name, boost::bind(&IntakeHatchPanel::executeCB, this, _1), false),
			action_name_(name),
			ae_("move_elevator_server", true)
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
			bool elevator_server_found = ae_.waitForServer(ros::Duration(wait_for_server_timeout));
			ros::Rate r(10);
			//define variables that will be re-used for each call to a controller
			double start_time;
			bool success; //if controller/actionlib server call succeeded

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;

				//elevator server
				behaviors::ElevatorGoal elev_goal;
				ElevatorLocation setpoint=INTAKE;
				elev_goal.setpoint_index = setpoint; //TODO: figure this out
				elev_goal.place_cargo = false;
				ae_.sendGoal(elev_goal);

				ae_.waitForResult();

				if(!elevator_server_found)
				{
					ROS_ERROR_STREAM("The elevator server was not loaded before the panel intake server needed it");
					as_.setPreempted();
					return;
				}

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
				success = false;

				//define request to send to cargo intake controller
				panel_intake_controller::PanelIntakeSrv srv;
				srv.request.claw_in = true; //TODO: make sure this means grab the panel
				srv.request.push_in = false; //this too
				//srv.request.wedge_in = false; //this too
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Srv intake call failed in auto interpreter server intake");
					preempted = true;
				}
				success = true;

				panel_intake_controller::PanelIntakeSrv srv;
				srv.request.claw_in = false; //TODO: make sure this means grab the panel
				srv.request.push_in = false; //this too
				//srv.request.wedge_in = false; //this too
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Srv intake call failed in auto interpreter server intake");
					preempted = true;
				}
				success = true;

				panel_intake_controller::PanelIntakeSrv srv;
				srv.request.claw_in = false; //TODO: make sure this means grab the panel
				srv.request.push_in = true; //this too
				//srv.request.wedge_in = false; //this too
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Srv intake call failed in auto interpreter server intake");
					preempted = true;
				}
				success = true;

/update everything by doing spinny stuff
				ros::spinOnce();

				//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
				//
				ros::Duration(pause_time_between_pistons).sleep();//sleeps for 0.5

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
	IntakeHatchPanel intake_hatch_panel_server("intake_hatch_panel_server");
	ros::spin();

	return 0;
}

#endif
