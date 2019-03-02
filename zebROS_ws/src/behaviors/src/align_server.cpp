#include "ros/ros.h"
#include <atomic>
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "behaviors/AlignGoal.h"
#include "behaviors/AlignAction.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

double align_timeout;
double orient_timeout;
double orient_error_threshold;
double x_error_threshold;
double cargo_error_threshold;


bool startup = true; //disable all pid nodes on startup
class AlignAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignAction> as_; //create the actionlib server
		std::string action_name_;
		behaviors::AlignResult result_; //variable to store result of the actionlib action


		std::shared_ptr<ros::Publisher> enable_navx_pub_;
		std::shared_ptr<ros::Publisher> enable_x_pub_;
		std::shared_ptr<ros::Publisher> enable_y_pub_;
		std::shared_ptr<ros::Publisher> enable_align_hatch_pub_;
		std::shared_ptr<ros::Publisher> enable_align_cargo_pub_;
		std::shared_ptr<ros::Publisher> enable_cargo_pub_;

		ros::Subscriber navx_error_sub_;
		ros::Subscriber x_error_sub_;
		ros::Subscriber y_error_sub_;
		ros::Subscriber cargo_error_sub_;

        ros::ServiceClient BrakeSrv;

		bool x_aligned_ = false;
		bool y_aligned_ = false;
		bool cargo_aligned_ = false;
		bool orient_aligned_ = false;
		bool distance_aligned_ = false;

	public:
		//make the executeCB function run every time the actionlib server is called
		AlignAction(const std::string &name,
			const std::shared_ptr<ros::Publisher>& enable_navx_pub_,
			const std::shared_ptr<ros::Publisher>& enable_x_pub_,
			const std::shared_ptr<ros::Publisher>& enable_y_pub_,
			const std::shared_ptr<ros::Publisher>& enable_align_hatch_pub_,
			const std::shared_ptr<ros::Publisher>& enable_align_cargo_pub_,
			const std::shared_ptr<ros::Publisher>& enable_cargo_pub_):
			as_(nh_, name, boost::bind(&AlignAction::executeCB, this, _1), false),
			action_name_(name),
			enable_navx_pub_(enable_navx_pub_),
			enable_x_pub_(enable_x_pub_),
			enable_y_pub_(enable_y_pub_),
			enable_align_hatch_pub_(enable_align_hatch_pub_),
			enable_align_cargo_pub_(enable_align_cargo_pub_),
			enable_cargo_pub_(enable_cargo_pub_)
		{
            as_.start();

            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";
			BrakeSrv = nh_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);

			navx_error_sub_ = nh_.subscribe("navX_pid/pid_debug", 1, &AlignAction::navx_error_cb, this);
			x_error_sub_ = nh_.subscribe("distance_pid/pid_debug", 1, &AlignAction::x_error_cb, this);
			cargo_error_sub_ = nh_.subscribe("cargo_pid/pid_debug", 1, &AlignAction::cargo_error_cb, this);
			y_error_sub_ = nh_.subscribe("align_with_terabee/y_aligned", 1, &AlignAction::y_error_cb, this);


		}

		~AlignAction(void)
		{
		}

		//Error callbacks
		void navx_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			orient_aligned_ = (fabs(msg.data[0]) < orient_error_threshold);
			ROS_WARN_STREAM_THROTTLE(1, "navX error: " << fabs(msg.data[0]));
		}
		void x_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			x_aligned_ = (fabs(msg.data[0]) < x_error_threshold);
			ROS_WARN_STREAM_THROTTLE(1, "distance error: " << msg.data[0]);
		}
		void y_error_cb(const std_msgs::Bool &msg)
		{
			y_aligned_ = msg.data;
		}
		void cargo_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			cargo_aligned_ = (fabs(msg.data[0]) < cargo_error_threshold);
			ROS_WARN_STREAM_THROTTLE(1, "cargo error: " << msg.data[0]);
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::AlignGoalConstPtr &goal) {
			ROS_INFO_STREAM("align server callback called");
			ros::Rate r(10);
			startup = false;

			double start_time = ros::Time::now().toSec();
			bool preempted = false;
			bool timed_out = false;
			bool orient_timed_out = false;
			bool aligned = false;

			orient_aligned_ = false;
			distance_aligned_ = false;
			x_aligned_ = false;
			y_aligned_ = false;
			while(!aligned && !preempted && !timed_out && ros::ok())
			{
				ros::spinOnce();
				r.sleep();

				//Define enable messages for pid nodes
				std_msgs::Bool orient_msg;
				std_msgs::Bool distance_msg;
				std_msgs::Bool terabee_msg;
				std_msgs::Bool enable_align_msg;

				orient_msg.data = true && !orient_timed_out;							//Publish true to the navX pid node throughout the action until orient_timed_out
				distance_msg.data = orient_aligned_ || orient_timed_out;				//Enable distance pid once orient is aligned or timed out
				terabee_msg.data = (orient_aligned_ || orient_timed_out) && x_aligned_; //Enable terabee node when distance is aligned and  orient aligns or orient times out
				enable_align_msg.data = !aligned;										//Enable publishing pid vals until aligned

				//CARGO VS HATCH PANEL
				//only difference is enable_cargo_pub_ vs enable_y_pub_
				//
				//Publish enable messages
				enable_navx_pub_->publish(orient_msg);
				enable_x_pub_->publish(distance_msg);
				if(goal->has_cargo) {
					enable_cargo_pub_->publish(terabee_msg);
					enable_align_cargo_pub_->publish(enable_align_msg);
					aligned = x_aligned_ && cargo_aligned_;	 //Check aligned
				}
				else {
					enable_y_pub_->publish(terabee_msg);
					enable_align_hatch_pub_->publish(enable_align_msg);
					aligned = x_aligned_ && y_aligned_;		//Check aligned
				}

				//Check timed out
				timed_out = (ros::Time::now().toSec() - start_time) > align_timeout;
				orient_timed_out = (ros::Time::now().toSec() - start_time) > orient_timeout;
				//Check preempted
				preempted = as_.isPreemptRequested();

				//Non-spammy debug statements
				if(!orient_aligned_)
					ROS_INFO_THROTTLE(1, "Orienting");
				else
					ROS_INFO_STREAM_THROTTLE(1, "Translating + Orienting. Distance aligned: " << x_aligned_ << " Cutout aligned: " << y_aligned_);
				if(orient_timed_out)
					ROS_ERROR_STREAM_THROTTLE(1, "Orient timed out!");
			}
			//Stop robot after aligning
			std_srvs::Empty empty;
			BrakeSrv.call(empty);
			ros::spinOnce();


			//Stop all PID after aligning
			std_msgs::Bool false_msg;
			false_msg.data = false;

			enable_navx_pub_->publish(false_msg);
			enable_x_pub_->publish(false_msg);
			enable_y_pub_->publish(false_msg);
			enable_cargo_pub_->publish(false_msg);
			enable_align_hatch_pub_->publish(false_msg);

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

	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "align_server_params");

	if(!n_params.getParam("align_timeout", align_timeout))
		ROS_ERROR_STREAM("Could not read align_timeout in align_server");
	if(!n_params.getParam("orient_timeout", orient_timeout))
		ROS_ERROR_STREAM("Could not read orient_timeout in align_server");
	if(!n_params.getParam("x_error_threshold", x_error_threshold))
		ROS_ERROR_STREAM("Could not read x_error_threshold in align_server");
	if(!n_params.getParam("orient_error_threshold", orient_error_threshold))
		ROS_ERROR_STREAM("Could not read orient_error_threshold in align_server");
	if(!n_params.getParam("cargo_error_threshold", cargo_error_threshold))
		ROS_ERROR_STREAM("Could not read cargo_error_threshold in align_server");

	std::shared_ptr<ros::Publisher> enable_navx_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_x_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_y_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_hatch_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_cargo_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_cargo_pub_ = std::make_shared<ros::Publisher>();

	*enable_navx_pub_ = n.advertise<std_msgs::Bool>("navX_pid/pid_enable", 1);
	*enable_x_pub_ = n.advertise<std_msgs::Bool>("distance_pid/pid_enable", 1);
	*enable_y_pub_ = n.advertise<std_msgs::Bool>("align_with_terabee/enable_y_pub", 1);
	*enable_cargo_pub_ = n.advertise<std_msgs::Bool>("cargo_pid/pid_enable", 1);
	*enable_align_hatch_pub_ = n.advertise<std_msgs::Bool>("align_hatch_pid/pid_enable", 1);
	*enable_align_cargo_pub_ = n.advertise<std_msgs::Bool>("align_cargo_pid/pid_enable", 1);

	AlignAction align_action("align_server", enable_navx_pub_, enable_x_pub_, enable_y_pub_, enable_align_hatch_pub_, enable_align_cargo_pub_, enable_cargo_pub_);

	ros::Rate r(20);
	while(ros::ok()) {
		//Stop PID nodes from defaulting true
		if(startup) {
			std_msgs::Bool false_msg;
			false_msg.data = false;
			enable_navx_pub_->publish(false_msg);
			enable_x_pub_->publish(false_msg);
			enable_y_pub_->publish(false_msg);
			enable_cargo_pub_->publish(false_msg);
			enable_align_hatch_pub_->publish(false_msg);
			enable_align_cargo_pub_->publish(false_msg);
		}

		ros::spinOnce();
	}
	ros::spin();
	return 0;
}
