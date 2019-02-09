#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "behaviors/AlignGoal.h"
#include "behaviors/AlignAction.h"

double align_timeout;
double orient_error_threshhold;
double x_error_threshhold;

class AlignAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignAction> as_; //create the actionlib server
		std::string action_name_;
		behaviors::AlignResult result_; //variable to store result of the actionlib action

		ros::Publisher enable_navx_pub_;
		ros::Publisher enable_x_pub_;
		ros::Publisher enable_y_pub_;

		ros::Subscriber navx_error_sub_;
		ros::Subscriber x_error_sub_;
		ros::Subscriber y_error_sub_;

		bool x_aligned_;
		bool y_aligned_;
		bool orient_aligned_;

	public:
		//make the executeCB function run every time the actionlib server is called
		AlignAction(const std::string &name) :
			as_(nh_, name, boost::bind(&AlignAction::executeCB, this, _1), false),
			action_name_(name)
		{
			as_.start(); //start the actionlib server

			enable_navx_pub_ = nh_.advertise<std_msgs::Bool>("/navX_snap_to_goal_pid/pid_enable", 1);
			enable_x_pub_ = nh_.advertise<std_msgs::Bool>("pid_enable", 1);
			enable_y_pub_ = nh_.advertise<std_msgs::Bool>("enable_y_pub", 1);

			navx_error_sub_ = nh_.subscribe("/navX_snap_to_goal_pid/pid_debug", 1, &AlignAction::navx_error_cb, this);
			x_error_sub_ = nh_.subscribe("distance_pid_state", 1, &AlignAction::x_error_cb, this);
			y_error_sub_ = nh_.subscribe("y_aligned", 1, &AlignAction::y_error_cb, this);
		}

		~AlignAction(void) 
		{
		}

		void navx_error_cb(const std_msgs::Float64 &msg)
		{
			orient_aligned_ = (msg.data > orient_error_threshhold);
		}

		void x_error_cb(const std_msgs::Float64 &msg)
		{
			x_aligned_ = (msg.data > x_error_threshhold);
		}

		void y_error_cb(const std_msgs::Bool &msg)
		{
			y_aligned_ = msg.data;
		}
		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::AlignGoalConstPtr &goal) {
			ros::Rate r(10);

			double start_time = ros::Time::now().toSec();
			bool success = false; 
			bool preempted = false;
			bool timed_out = false;

			/*CALL NAVX STUFF HERE*/

			start_time = ros::Time::now().toSec();
			bool aligned = false;
			while(!aligned && !preempted && !timed_out)
			{
				ros::spinOnce();

				std_msgs::Bool x_msg;
				x_msg.data = !x_aligned_;
				enable_x_pub_.publish(x_msg);

				std_msgs::Bool y_msg;
				y_msg.data = !y_aligned_;
				enable_y_pub_.publish(y_msg);

				timed_out = (ros::Time::now().toSec() - start_time) > align_timeout;
				preempted = as_.isPreemptRequested();
				aligned = x_aligned_ && y_aligned_;
			}
			
			if(timed_out)
			{
				result_.timed_out = true;
				result_.success = false;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				result_.timed_out = false;
				result_.success = false;
				as_.setPreempted(result_);
				
				ROS_INFO("%s: Preempted", action_name_.c_str());
				
			}
			else //implies succeeded
			{
				result_.timed_out = false;
				result_.success = true;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			return;
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "align_server");

	AlignAction align_action("align_server");

	ros::NodeHandle n;

	if(!n.getParam("align_timeout", align_timeout))
		ROS_ERROR_STREAM("Could not read align_timeout in align_server");

	if(!n.getParam("x_error_threshhold", x_error_threshhold))
		ROS_ERROR_STREAM("Could not read x_error_threshhold in align_server");

	if(!n.getParam("orient_error_threshhold", orient_error_threshhold))
		ROS_ERROR_STREAM("Could not read orient_error_threshhold in align_server");

	ros::spin();
	return 0;
}
