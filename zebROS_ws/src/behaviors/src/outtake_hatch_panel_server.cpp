#ifndef PANEL_OUTTAKE_SERVER
#define PANEL_OUTTAKE_SERVER

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <panel_intake_controller/PanelIntakeSrv.h>
#include <behaviors/PlaceAction.h>
#include <behaviors/ElevatorAction.h>

//define global variables that will be defined based on config values
double elevator_timeout;
double pause_time_between_pistons;
double wait_for_server_timeout;

class OuttakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::PlaceAction> as_;
		std::string action_name_;

		behaviors::PlaceFeedback feedback_;
		behaviors::PlaceResult result_;

		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::ServiceClient panel_controller_client_;

		ros::Subscriber GoalDetectSub_;

	public:
		OuttakeHatchPanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&OuttakeHatchPanelAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("move_elevator_server", true)
	{
		//GoalDetectSub_ = nh_.subscribe("goal_detect_msg",1, &OuttakeHatchPanelAction::goalDetectCallback, this) //TODO make sure this is linked up correctly
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

		~OuttakeHatchPanelAction(void) {}

		void executeCB(const behaviors::PlaceGoalConstPtr &goal)
		{
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

			//move elevator to outtake location
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = goal->setpoint_index;
			elev_goal.place_cargo = false;
			ac_elevator_.sendGoal(elev_goal);

			bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(elevator_timeout - (ros::Time::now().toSec() - start_time)));
			if(!finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac_elevator_.getState();
				if(state.toString().c_str() != "SUCCEEDED")
				{
					preempted = true;
				}
			}
			else {
				timed_out = true;
			}

			//test if we got a preempt while waiting
			if(as_.isPreemptRequested())
			{
				preempted = true;
			}

			//send commands to panel_intake_controller to grab the panel ---------------------------------------
			if(!preempted && !timed_out)
			{
				ROS_ERROR("hatch panel outtake server running");

				//extend panel mechanism
				panel_intake_controller::PanelIntakeSrv srv;
				srv.request.claw_release = false;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv))
				{
					ROS_ERROR("Panel controller call failed in panel outtake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything


				//pause for a bit
				ros::Duration(pause_time_between_pistons).sleep();

				//release the panel - we can reuse the srv variable
				srv.request.claw_release = true;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Panel controller call failed in panel outtake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything


				//pause for a bit
				ros::Duration(pause_time_between_pistons).sleep();

				//retract the panel mechanism; we can reuse the srv variable
				srv.request.claw_release = true;
				srv.request.push_extend = false;
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Panel controller call failed in panel outtake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything

			}

			//log state of action and set result of action
			result_.timed_out = timed_out;

			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
				result_.success = false;
				as_.setSucceeded(result_);
			}
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				result_.success = false;
				as_.setPreempted(result_);
			}
			else //implies succeeded
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
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
	ros::init(argc, argv, "panel_outtake_server");

	OuttakeHatchPanelAction outtake_hatch_panel_server("outtake_hatch_panel_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_params");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in panel_outtake_sever");
	if (!n_panel_params.getParam("elevator_timeout", elevator_timeout))
		ROS_ERROR("Could not read elevator_timeout in panel_outtake_sever");
	if (!n_panel_params.getParam("pause_time_between_pistons", pause_time_between_pistons))
		ROS_ERROR("Could not read pause_time_between_pistons in panel_outtake_sever");

	ros::spin();

	return 0;
}

#endif
