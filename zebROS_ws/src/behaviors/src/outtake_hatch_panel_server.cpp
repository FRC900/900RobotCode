//#ifndef PANEL_OUTTAKE_SERVER
//#define PANEL_OUTTAKE_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <panel_outtake_controller/PanelOuttakeSrv.h>
#include <behaviors/PlaceAction.h>
#include <behaviors/PathAction.h>
#include <behaviors/ElevatorAction.h>
#include "behaviors/enumerated_elevator_indices.h"

//define global variables that will be defined based on config values
//double linebreak_debounce_iterations;
double panel_outtake_timeout;//this is the same variable as used in cargo_outtake, should it be a different one for the hatch panel?
double cargo_ship_setpoint;
double low_rocket_setpoint;
double mid_rocket_setpoint;
double high_rocket_setpoint;

class OuttakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::OuttakeAction> as_;
		std::string action_name_;

		behaviors::OuttakeFeedback feedback_;
		behaviors::PlaceResult result_;

		actionlib::SimpleActionClient<behaviors::ElevatorAction> ae_;
		actionlib::SimpleActionClient<behaviors::PathAction> ap_;

		ros::ServiceClient controller_client_; //create a ros client to send requests to the controller
		ros::Subscriber GoalDetectSub_;

	public:
		OuttakeHatchPanel(std::string name) :
			as_(nh_, name, boost::bind(&OuttakeHatchPanel::executeCB, this, _1), false),
			action_name_(name),
			ae_("move_elevator_server", true),
			ap_("path_to_goal", true)
			ac_elevator_("elevator_server", true) //TODO make sure this is linked up correctly
	{
		GoalDetectSub_ = nh_.subscribe("goal_detect_msg",1, &OuttakeHatchPanelAction::goalDetectCallback, this) //TODO make sure this is linked up correctly
			/* std::map<std::string, std::string> service_connection_header;
			   service_connection_header["tcp_nodelay"] = "1";
			   ElevatorSrv_ = nh_.serviceClient<elevator_controller::ElevatorControlS>("/frcrobot/elevator_controller/cmd_posS", false, service_connection_header);
			   IntakeSrv_ = nh_.serviceClient<elevator_controller::Intake>("/frcrobot/elevator_controller/intake", false, service_connection_header);
			   ClampSrv_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot/elevator_controller/clamp", false, service_connection_header);
			   HighCube_ = nh_.subscribe("/frcrobot/elevator_controller/cube_state", 1, &autoAction::cubeStateCallback, this);
			   BottomLimit_ = nh_.subscribe("/frcrobot/elevator_controller/bottom_limit_pivot", 1, &autoAction::bottomLimitCallback, this);*/

			as_.start();
		//initialize the client being used to call the controller
		ros::ServiceClient panel_outtake_controller_client_ = nh_.serviceClient<panel_outtake_controller::PanelOuttakeSrv>("/frcrobot_jetson/cargo_outtake_controller/panel_outtake_command", false, service_connection_header);
		ros::ServiceClient elevator_controller_client_ = nh_.serviceClient<elevator_controller::ElevatorSrv>("/frcrobot_jetson/elevator_controller/elevator_service", false, service_connection_header);
		//start subscribers subscribing
		joint_states_sub_ = nh_.subscribe("/frcrobot/joint_states", 1, &PanelIntakeAction::jointStateCallback, this);
}

~OuttakeHatchPanelAction(void) {}

void executeCB(const behaviors::PlaceHatchPanelGoalConstPtr &goal)
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
		ROS_ERROR("hatch panel outtake server running");

		//reset variables
		//linebreak_true_count = 0; //when this gets higher than linebreak_debounce_iterations, we'll consider the gamepiece intaked
		start_time = ros::Time::now().toSec();
		success = false;

		//define request to send to panel outtake controller
		panel_outtake_controller::PanelOuttakeSrv srv;
		srv.request.claw_in = false; //TODO: make sure this means grab the panel
		srv.request.push_in = true; //this too
		srv.request.wedge_in = false; //this too
		//send request to controller
		if(!controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
		{
			ROS_ERROR("Srv outtake call failed in auto interpreter server outtake");
		}
		//update everything by doing spinny stuff
		ros::spinOnce();

		//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
		while(!success && !timed_out && !preempted) {
			success = linebreak_true_count > linebreak_debounce_iterations;
			if(as_.isPreemptRequested() || !ros::ok()) {
				ROS_WARN("%s: Preempted", action_name_.c_str());
				as_.setPreempted();
				preempted = true;
			}
			if (!preempted) {
				r.sleep();
				ros::spinOnce();
				timed_out = (ros::Time::now().toSec()-start_time) > goal->timeout;
			}
		}
	}
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
	ros::init(argc, argv, "outtake_hatch_panel_server");

	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "actionlib_hatch_panel_outtake_params");

	//if(!n.getParam("actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
	//{
	//	ROS_ERROR("Could not read linebreak_debounce_iterations in outtake_hatch_panel_server");
	//}

	if(!n_params.getParam("outtake_timeout", outtake_timeout))
	{
		ROS_ERROR("Could not read outtake_timeout in outtake_hatch_panel_server");
	}

	if(!n_params_lift.getParam("cargo_ship", cargo_ship_setpoint))
	{
		ROS_ERROR("Coult not read cargo_ship in outtake_hatch_panel_server");
	}
	if(!n_params_lift.getParam("low_rocket", low_rocket_setpoint))
	{
		ROS_ERROR("Coult not read low_rocket in outtake_hatch_panel_server");
	}
	if(!n_params_lift.getParam("high_rocket", high_rocket_setpoint))
	{
		ROS_ERROR("Coult not read high_rocket in outtake_hatch_panel_server");
	}
	if(!n_params_lift.getParam("mid_rocket", mid_rocket_setpoint))
	{
		ROS_ERROR("Coult not read mid_rocket in outtake_hatch_panel_server");
	}



	/*
	   if(!n_params_fake.getParam("help_me", intake_timeout));
	   ROS_ERROR("Could not read intake_timeout in intake_hatch_panel_server");
	   if(!n_params_fake.getParam("help_me2", arm_timeout));
	   ROS_ERROR("Could not read arm_timeout in intake_hatch_panel_server");
	   */

	//create the actionlib server from the class defined above
	OuttakeHatchPanel outtake_hatch_panel_server("outtake_hatch_panel_server");
	ros::spin();

	return 0;
}

#endif
