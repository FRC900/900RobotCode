//Import other files that are used
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include <behavior_actions/RotatePanelAction.h>
#include <controllers_2020_msgs/ControlPanelSrv.h>
#include <controllers_2020_msgs/ClimberSrv.h>

//Import files used for controlling the drivebase
//#include "geometry_msgs/Twist.h"
//#include "talon_state_msgs/TalonState.h"

class RotatePanelAction {

	protected:

		//Make variables for ros and the actionlib server
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behavior_actions::RotatePanelAction> as_;
		std::string action_name_;

		//Make variables for the clients, publishers, and subscribers
		ros::ServiceClient rotate_panel_client_;
		ros::ServiceClient climber_client_;

		//These are only used if we make the server control the drivebase
		//ros::Subscriber talon_states_sub_;
		//ros::Publisher cmd_vel_publisher_;


	private:

		//Make variables for config values
		double timeout;
		double rotations;
		double wait_for_server_timeout;

		//These are only used if we make the server control the drivebase
		//int cmd_speed;
		//double minimum_current

	public:

		//Construct the class instance
		RotatePanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&RotatePanelAction::executeCB, this, _1), false),
			action_name_(name)

	{
		//Import config values
		if (!nh_.getParam("rotations", rotations))
		{
			ROS_ERROR("Could not read rotations in rotateHatchPanel");
		}
		if (!nh_.getParam("timeout", timeout))
		{
			ROS_ERROR("Could not read timeout in rotateHatchPanel");
		}
		if (!nh_.getParam("wait_for_server_timeout", wait_for_server_timeout))
		{
			ROS_ERROR("Could not read wait_for_server_timeout in rotateHatchPanel");
		}

		//Start the actionlib server
		as_.start();

		//Networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//Initialize the clients used to move the control panel mechanism and climber mechanism
		rotate_panel_client_ = nh_.serviceClient<controllers_2020_msgs::ControlPanelSrv>("/frcrobot_jetson/control_panel_controller/control_panel_command", false, service_connection_header);
		climber_client_ = nh_.serviceClient<controllers_2020_msgs::ClimberSrv>("/frcrobot_jetson/climber_controller/climber_command", false, service_connection_header);

		//Subscribe to the state of the motors and publish messages to the drive base. Only used if we control the drivebase.
		//talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &RotatePanelAction::TalonStateCallback, this);
		//cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel",1);

	}

		~RotatePanelAction(void)
		{
		}

		//This is the callback function that runs whenever the actionlib server is called
		void executeCB(const behavior_actions::RotatePanelGoalConstPtr &goal)
		{
			//Set the rate ros should refresh at
			ros::Rate r(10);

			//Make a variable for using the result section of the action file
			behavior_actions::RotatePanelResult result_;

			//Make variables specific to each call of the actionlib server
			double start_time;
			bool preempted = false;
			bool timed_out = false;
			bool success = false;

			//If the server hasn't preempted or timed out
			if(!preempted && !timed_out)
			{

				//Wait for the server to contact the control panel controller. If it doesn't respond, send an error and preempt
				if(!rotate_panel_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
				{
					ROS_ERROR_STREAM("The control_panel_controller was not ready in time for the rotate_panel_server");
					as_.setPreempted();
					return;
				}

				//Wait for the climber controller to respond. If it doesn't respond, send an error and preempt
				if(!climber_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
				{
					ROS_ERROR_STREAM("The climber_controller was not ready in time for the rotate_panel_server");
					as_.setPreempted();
					return;
				}

				//Send a message indicating the server has started
				ROS_INFO("Rotate Panel Server: Rotating Panel");

				//Set the start time of the server call, and say that it hasn't been successful yet
				start_time = ros::Time::now().toSec();
				success = false;

				//Make a variable for controlling the drive base
				//geometry_msgs::Twist cmd_vel_msg;

				//Make the srv variables for making requests to the climber and control panel controllers
				controllers_2020_msgs::ClimberSrv climber_srv;
				controllers_2020_msgs::ControlPanelSrv srv;

				//Request that the climber extend, stay at the 0 setpoint, and engage the brake
				climber_srv.request.winch_percent_out = 0;
				climber_srv.request.climber_deploy = true;
				climber_srv.request.climber_elevator_brake = true;

				//Send the request. If it doesn't work, send an error
				if (!climber_client_.call(climber_srv))
				{
					ROS_ERROR("Srv failed in raising the climber");
				}

				//Make a message to move the drive base in the x direction, and publish the message
				//cmd_vel_msg.linear.x = cmd_speed;
				//cmd_vel_msg.linear.y = 0.0;
				//cmd_vel_msg.linear.z = 0.0;
				//cmd_vel_msg.angular.x = 0.0;
				//cmd_vel_msg.angular.y = 0.0;
				//cmd_vel_msg.angular.z = 0.0;
				//cmd_vel_publisher_.publish(cmd_vel_msg);

				//Request that the control panel controller moves the control panel a certain number of times
				srv.request.control_panel_rotations = rotations;

				//Send the request. If it doesn't work, send an error
				if (!rotate_panel_client_.call(srv))
				{
					ROS_ERROR("Srv failed in rotating the panel");
				}

				//Request the control panel mechanism to stop rotating and the climber to go back down
				srv.request.control_panel_rotations = 0;
				climber_srv.request.climber_deploy = false;

				//Send both requests. If they don't go through, send errors.
				if (!rotate_panel_client_.call(srv))
				{
					ROS_ERROR("Srv failed in stopping panel rotation");
				}
				if (!climber_client_.call(climber_srv))
				{
					ROS_ERROR("Srv failed in lowering the climber");
				}

				//While the server hasn't succeeded, failed, or timed out
				while(!success && !timed_out && !preempted)
				{
					//If the server has requested to be preempted, or ros isn't okay, send a warning and preempt the server
					if(as_.isPreemptRequested() || !ros::ok())
					{
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						preempted = true;
					}

					//Otherwise if the server is fine, wait, spin ros, and check if the server has timed out
					if (!preempted)
					{
						r.sleep();
						ros::spinOnce();
						timed_out = (ros::Time::now().toSec()-start_time) > timeout;
					}
				}
			}

			//If the server has timed out, send a message
			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}

			//If the server is preempted, send a message
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}

			//If the server has succeeded, send a message
			else
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			//Send the state of the server to the action file
			result_.preempted = preempted;
			result_.timed_out = timed_out;
			result_.success = success;

			//Set the reult of the actionlib server
			as_.setSucceeded(result_);
			return;
		}
};

int main(int argc, char** argv)
{
	//Initialize ros
	ros::init(argc, argv, "rotate_panel_server");

	//Create the server instance
	RotatePanelAction rotate_panel_action("rotate_panel_server");

	//Import config values used to move the drivebase
	/*if (!n_params.getParam("cmd_speed", rotate_panel_action.cmd_speed))
	  {
	  ROS_ERROR_STREAM("Could not read cmd_speed in rotateHatchPanel");
	  }
	  if (!n_params.getParam("minimum_current", rotate_panel_action.minimum_current))
	  {
	  ROS_ERROR_STREAM("Could not read minimum_current in rotateHatchPanel");
	  }*/

	//Refresh ros
	ros::spin();
	return 0;
}
