#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behavior_actions/RotatePanelAction.h"
#include "behavior_actions/rotate_panel_override.h"
#include "sensor_msgs/JointState.h"
#include "talon_state_msgs/TalonState.h"
#include "controllers_2020_msgs/ControlPanelSrv"

class RotatePanelAction {

	protected:


		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behavior_actions::RotatePanelAction> as_;
		std::string action_name_;
		ros::ServiceClient controller_client_;
		behavior_actions::RotatePanelFeedback feedback_;
		behavior_actions::RotatePanelResult result_;
		ros::Subscriber talon_states_sub_;

	public:

		double timeout;
		double rotations;
		double wait_for_server_timeout;
		bool preempted;
		bool timed_out;

		RotatePanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&RotatePanelAction::executeCB, this, _1), false),
			action_name_(name)

	{

		as_.start();

		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		controller_client_ = nh_.serviceClient<controllers_2020_msgs::ControlPanelSrv>("/frcrobot_jetson/controllers_2020/control_panel_controller", false, service_connection_header);

		talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states", 1, &RotatePanelAction::TalonStateCallback, this);

	}

		~RotatePanelAction(void)
		{
		}

		void TalonStateCallback(const talon_state_msgs::TalonState &talon_state)
		{
			int x = 1;
		}

		void executeCB(const behavior_actions::RotatePanelGoalConstPtr &goal) {
			ros::Rate r(10);
			double start_time;
			bool success;
			result_.success = true;
			preempted = false;
			timed_out = false;
			success = false;

			if(!preempted && !timed_out)
			{
				ROS_INFO("Rotate Panel Server: Rotating Panel");

				//reset variables
				start_time = ros::Time::now().toSec();
				success = false;
				control_panel_controller::RotatePanelSrv srv;
				srv.request.rotations = rotations;

				if (!controller_client_.call(srv))
				   {
				   ROS_ERROR("Srv intake call failed in auto interpreter server intake");
				   }

				ros::spinOnce();

				while(!success && !timed_out && !preempted) {
					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_WARN("%s: Preempted", action_name_.c_str());
						as_.setPreempted();
						preempted = true;
					}
					if (!preempted) {
						r.sleep();
						ros::spinOnce();
						timed_out = (ros::Time::now().toSec()-start_time) > timeout;
					}
				}
			}
			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			result_.timed_out = timed_out;
			result_.success = success;
			as_.setSucceeded(result_);
			return;
		}

		bool DS_Override(behavior_actions::rotate_panel_override::Request &req,
				behavior_actions::rotate_panel_override::Response &res)
		{
			if (req.override == true)
			{
				preempted = true;
				res.success = true;
				res.override = true;
				return true;
			}

			else
			{
				return false;
			}
		}
};
int main(int argc, char** argv) {
	ros::init(argc, argv, "rotate_panel_server");
	RotatePanelAction rotate_panel_action("rotate_panel_server");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "rotateControlPanelParams");
	if(!n_params.getParam("rotations", rotate_panel_action.rotations))
	{
		ROS_ERROR_STREAM("Could not read rotations in rotateHatchPanel");
	}
	if(!n_params.getParam("timeout", rotate_panel_action.timeout))
	{
		ROS_ERROR_STREAM("Could not read timeout in rotateHatchPanel");
	}

	ros::spin();
	return 0;
}
