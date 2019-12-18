#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <controllers_2019/PanelIntakeSrv.h>
#include <behaviors/PlaceAction.h>
#include <behaviors/ElevatorAction.h>
#include <thread>
#include <geometry_msgs/Twist.h>
#include <atomic>
#include <std_srvs/Empty.h>

//define global variables that will be defined based on config values
double elevator_timeout;
double outtake_timeout;
double wait_for_server_timeout;
double pause_time_after_release;
double pause_time_after_extend;
double pause_time_after_drawback;
double outtake_cmd_vel;

class OuttakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::PlaceAction> as_;
		std::string action_name_;

		std::atomic<bool> continue_outtake;

		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::ServiceClient panel_controller_client_;
		ros::ServiceServer continue_outtake_server_;
		ros::Publisher cmd_vel_publisher_;

		ros::Subscriber GoalDetectSub_;

		std::atomic<double> cmd_vel_forward_speed_;
		std::atomic<bool> stopped_;

	public:
		OuttakeHatchPanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&OuttakeHatchPanelAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true)
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
		panel_controller_client_ = nh_.serviceClient<controllers_2019::PanelIntakeSrv>("/frcrobot_jetson/panel_intake_controller/panel_command", false, service_connection_header);
		bool continueOuttakeCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &response);
		continue_outtake_server_ = nh_.advertiseService("continue_outtake_panel", &OuttakeHatchPanelAction::continueOuttakeCB, this);
		continue_outtake = false;

		cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);

	}

		~OuttakeHatchPanelAction(void) {}

		bool continueOuttakeCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &response) {
			continue_outtake = true;
			return true;
		}

		void cmdVelCallback()
		{
			ROS_INFO_STREAM("the callback is being called");
			geometry_msgs::Twist cmd_vel_msg;
			stopped_ = false;

			ros::Rate r(20);

			while(ros::ok() && !stopped_)
			{
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.linear.y = cmd_vel_forward_speed_;
				cmd_vel_msg.linear.z = 0.0;
				cmd_vel_msg.angular.x = 0.0;
				cmd_vel_msg.angular.y = 0.0;
				cmd_vel_msg.angular.z = 0.0;

				cmd_vel_publisher_.publish(cmd_vel_msg);
				r.sleep();
			}
		}

		void executeCB(const behaviors::PlaceGoalConstPtr &goal)
		{
			ROS_WARN("hatch panel outtake server running");

			cmd_vel_forward_speed_ = 0.0;
			continue_outtake = false; //Set continue to false to wait for B to be released

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
			std::thread cmdVelThread(std::bind(&OuttakeHatchPanelAction::cmdVelCallback, this));

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

			cmd_vel_forward_speed_ = outtake_cmd_vel;

			bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(std::max(elevator_timeout - (ros::Time::now().toSec() - start_time), 0.001)));
			if(finished_before_timeout && !ac_elevator_.getResult()->timed_out) {
				actionlib::SimpleClientGoalState state = ac_elevator_.getState();
				if(state.toString() != "SUCCEEDED") {
					ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
				}
				else {
					ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
				}
			}
			else {
				ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
			}


			//test if we got a preempt while waiting
			if(as_.isPreemptRequested())
			{
				preempted = true;
			}

			//Wait for B button to be released before continuing action
			while(!continue_outtake && !timed_out && !preempted && ros::ok()) {
				timed_out = ros::Time::now().toSec() - start_time > outtake_timeout;
				if(as_.isPreemptRequested()){
					preempted = true;
				}
				ros::spinOnce();
				r.sleep();
			}
			//send commands to panel_intake_controller to grab the panel ---------------------------------------
			if(!preempted && ros::ok())
			{
				//extend panel mechanism
				controllers_2019::PanelIntakeSrv srv;
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
				ros::Duration(pause_time_after_extend).sleep();

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
				ros::Duration(pause_time_after_release).sleep();

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
                                
                                ros::Duration(pause_time_after_drawback).sleep();
			}

			ros::Duration(1).sleep();

			//lower elevator
			elev_goal.setpoint_index = goal->end_setpoint_index;
			elev_goal.place_cargo = false;
			ac_elevator_.sendGoal(elev_goal);

			finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(std::max(elevator_timeout - (ros::Time::now().toSec() - start_time), 0.001)));
			if(finished_before_timeout && !ac_elevator_.getResult()->timed_out) {
				actionlib::SimpleClientGoalState state = ac_elevator_.getState();
				if(state.toString() != "SUCCEEDED") {
					ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
				}
				else {
					ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
				}
			}
			else {
				ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
			}

			//TODO fix this comp change made end state pulled in, and deployed and isn't with in frame perimeter
			//set final state of mechanism - pulled in, clamped (to stay within frame perimeter)
			//it doesn't matter if timed out or preempted, do anyways
			//extend panel mechanism
			controllers_2019::PanelIntakeSrv srv;
			srv.request.claw_release = true;
			srv.request.push_extend = false;
			//send request to controller
			if(!panel_controller_client_.call(srv))
			{
				ROS_ERROR("Panel controller call failed in panel outtake server, final state of mechanism call.");
				preempted = true;
			}
			ros::spinOnce(); //update everything

			//log state of action and set result of action
			behaviors::PlaceResult result;
			result.timed_out = timed_out;

			if(timed_out)
			{
				ROS_WARN("%s: Timed Out", action_name_.c_str());
				result.success = false;
			}
			else if(preempted)
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				result.success = false;
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result.success = true;
			}
			as_.setSucceeded(result);

			stopped_ = true;
			cmdVelThread.join();

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
	ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_outtake_params");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in panel_outtake_sever");
	if (!n_panel_params.getParam("elevator_timeout", elevator_timeout))
		ROS_ERROR("Could not read elevator_timeout in panel_outtake_sever");
	if (!n_panel_params.getParam("outtake_timeout", outtake_timeout))
		ROS_ERROR("Could not read outtake_timeout in panel_outtake_sever");
	if (!n_panel_params.getParam("pause_time_after_release", pause_time_after_release))
		ROS_ERROR("Could not read pause_time_after_release in panel_outtake_sever");
	if (!n_panel_params.getParam("pause_time_after_drawback", pause_time_after_drawback))
		ROS_ERROR("Could not read pause_time_after_drawback in panel_outtake_sever");
	if (!n_panel_params.getParam("pause_time_after_extend", pause_time_after_extend))
		ROS_ERROR("Could not read pause_time_after_extend in panel_outtake_sever");
	if (!n_panel_params.getParam("outtake_cmd_vel", outtake_cmd_vel))
		ROS_ERROR("Could not read outtake_cmd_vel in panel_outtake_sever");

	ros::spin();

	return 0;
}
