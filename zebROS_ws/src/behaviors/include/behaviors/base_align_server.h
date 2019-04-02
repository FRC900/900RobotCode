#pragma once
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "behaviors/AlignGoal.h"
#include "behaviors/AlignAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/enumerated_elevator_indices.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

// TODO : These probably need to be moved into the base class, along
// with some defaults and a way to set them

extern bool debug;

// TODO
//    One way to clean this up is to create a base AlignAction class
//    that then gets specialized into different derived classes based on the 
//    thing being aligned
//    As it stands now, you'd want one derived class for Terabee hatch (currently only auto align)
//    and a second for cargo on the rocket.  We'd add additional derived classes for
//    ZED hatch and C920 cargo ship cargo
//    The base class would have everything shared between the processes.  Basically, everything
//    at this point minus a few exceptions
//    The point where the class is checking if(goal->has_cargo) would turn into an
//    unconditional call to a member function. This function would be different in each
//    derived class - it would be customized for each derived type.
//    Then to make this function work, you'd combine the goal->has_cargo specific vars
//    (e.g. hatch_panel_enable_distance pub and cargo_enable_distance_pub) into a single
//    var. That var would be initialized to the correct value at init time for each of the
//    separate classes.  So when constructing the derived TerabeeCargoAlignAction class,
//    set the now-combined enable_distance_pub var to publish to "cargo_distance_pid/pid_enable"
//    Then the specialized function in the derived TerabeeCargoAlignAction will use
//    distance_pub, and it will be set to the correct publisher for that.
//    After doing this, continue to simplify by moving code which is only needed
//    in the derived classes into those classed (e.g. the callback for their particular
//    enable_distance_pub)
//    The main idea is to make the base class code generic to the "align angle, then distance, then y"
//    problem and putting anything specific into derived classes. This makes it easier to add
//    a new e.g. Zed class - you just have to fill in the Zed-specific parts while still having
//    the Base framework for the generic stuff.
//    This would move the decision between cargo vs hatch up to the teleop, but it kinda makes
//    sense - it would call a variety of AlignActions based on input it has already
//    then make the specific Align Action only do one thing well

//bool startup = true; //disable all pid nodes on startup
class BaseAlignAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignAction> as_; //create the actionlib server
		std::string action_name_;
		// TODO this result should be a local var
		behaviors::AlignResult result_; //variable to store result of the actionlib action
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;

		ros::Publisher enable_align_pub_;
		ros::Publisher enable_orient_pub_;
		ros::Publisher enable_x_pub_;
		ros::Publisher enable_y_pub_;

		ros::Subscriber orient_error_sub_;
		ros::Subscriber x_error_sub_;
		ros::Subscriber y_error_sub_;

        ros::ServiceClient BrakeSrv_;

		bool aligned_ = false;
		bool orient_aligned_ = false;
		bool x_aligned_ = false;
		bool y_aligned_ = false;
	
		double orient_error_ = 0.0;
		double x_error_ = 0.0;
		double y_error_ = 0.0;

		bool orient_timed_out_ = false;
		bool x_timed_out_ = false;
		bool y_timed_out_ = false;

		double align_timeout_ = 0.0;
		double orient_timeout_ = 0.0;
		double x_timeout_ = 0.0;
		double y_timeout_ = 0.0;

		double orient_error_threshold_ = 0.0;
		double x_error_threshold_ = 0.0;
		double y_error_threshold_ = 0.0;

		bool preempted_ = false;

		double start_time_ = -1.0;

	public:
		//make the executeCB function run every time the actionlib server is called
		BaseAlignAction(const std::string &name,

						const std::string &enable_align_topic_,
						const std::string &enable_orient_topic_,
						const std::string &enable_x_topic_,
						const std::string &enable_y_topic_,

						const std::string &orient_error_topic_,
						const std::string &x_error_topic_,
						const std::string &y_error_topic_,

						const std::string &align_timeout_param_name_,
						const std::string &orient_timeout_param_name_,
						const std::string &x_timeout_param_name_,
						const std::string &y_timeout_param_name_,

						const std::string &orient_error_threshold_param_name_,
						const std::string &x_error_threshold_param_name_,
						const std::string &y_error_threshold_param_name_):

			as_(nh_, name, boost::bind(&BaseAlignAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true),
			enable_align_pub_(nh_.advertise<std_msgs::Bool>(enable_align_topic_, 1, true)),
			enable_orient_pub_(nh_.advertise<std_msgs::Bool>(enable_orient_topic_, 1, true)),
			enable_x_pub_(nh_.advertise<std_msgs::Bool>(enable_x_topic_, 1, true)),
			enable_y_pub_(nh_.advertise<std_msgs::Bool>(enable_y_topic_, 1, true)),
			orient_error_sub_(nh_.subscribe(orient_error_topic_, 1, &BaseAlignAction::orient_error_cb, this)),
			x_error_sub_(nh_.subscribe(x_error_topic_, 1, &BaseAlignAction::x_error_cb, this)),
			y_error_sub_(nh_.subscribe(y_error_topic_, 1, &BaseAlignAction::y_error_cb, this))
		{
            as_.start();

            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

			BrakeSrv_ = nh_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);

			if(!nh_.getParam(align_timeout_param_name_, align_timeout_))
				ROS_ERROR_STREAM("Could not read align_timeout_param_name_ in align_server");
			if(!nh_.getParam(orient_timeout_param_name_, orient_timeout_))
				ROS_ERROR_STREAM("Could not read orient_timeout_param_name_ in align_server");
			if(!nh_.getParam(x_timeout_param_name_, x_timeout_))
				ROS_ERROR_STREAM("Could not read x_timeout_param_name_ in align_server");
			if(!nh_.getParam(y_timeout_param_name_, y_timeout_))
				ROS_ERROR_STREAM("Could not read y_timeout_param_name_ in align_server");

			if(!nh_.getParam(orient_error_threshold_param_name_, orient_error_threshold_))
				ROS_ERROR_STREAM("Could not read orient_error_threshold_param_name_ in align_server");
			if(!nh_.getParam(x_error_threshold_param_name_, x_error_threshold_))
				ROS_ERROR_STREAM("Could not read x_error_threshold_param_name_ in align_server");
			if(!nh_.getParam(y_error_threshold_param_name_, y_error_threshold_))
				ROS_ERROR_STREAM("Could not read y_error_threshold_param_name_ in align_server");
		}

		~BaseAlignAction(void)
		{
		}

		//function to check for preempts and timeouts
		bool check_timeout(double start_time, double timeout) {
			bool timeout_var = false;
			if(ros::Time::now().toSec() - start_time > timeout) {
				timeout_var = true;
			}
			return timeout_var;
		}
		bool check_preempted() {
			bool preempted_var = false;
			if(as_.isPreemptRequested()) {
				preempted_var = true;
			}
			return preempted_var;
		}

		//Default error callbacks for pid node
		virtual void orient_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			orient_aligned_ = (fabs(msg.data[0]) < orient_error_threshold_);
			orient_error_ =fabs(msg.data[0]) < orient_error_threshold_;
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "orient error: " << fabs(msg.data[0]));
		}
		virtual void x_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			x_aligned_ = (fabs(msg.data[0]) < x_error_threshold_);
			x_error_ =fabs(msg.data[0]) < x_error_threshold_;
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "x error: " << fabs(msg.data[0]));
		}
		virtual void y_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			y_aligned_ = (fabs(msg.data[0]) < y_error_threshold_);
			y_error_ =fabs(msg.data[0]) < y_error_threshold_;
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "y error: " << fabs(msg.data[0]));
		}

		virtual void enable_align(bool enable=true) {
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_align_pub_.publish(enable_msg);
		}

		//Functions to enable align PID
		virtual void align_orient(ros::Rate r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			ROS_INFO_STREAM("Running align_orient");
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_orient_pub_.publish(enable_msg);

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !orient_aligned_ && !preempted_ && !orient_timed_out_) {
					enable_align();
					orient_timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				enable_orient_pub_.publish(enable_msg);
			}
		}
		virtual void align_x(ros::Rate r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_x_pub_.publish(enable_msg);

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !x_aligned_ && !preempted_ && !x_timed_out_) {
					enable_align();
					x_timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				enable_x_pub_.publish(enable_msg);
			}
		}
		virtual void align_y(ros::Rate r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			ROS_INFO_STREAM("Running align_y");
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_y_pub_.publish(enable_msg);
			start_time_ = ros::Time::now().toSec();

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !y_aligned_ && !preempted_ && !y_timed_out_) {
					enable_align();
					y_timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				enable_y_pub_.publish(enable_msg);
			}
		}

		//TODO ability to define just order of orient, x, and y in config or launch file with timeouts and conditions for continuing to the next step and align in one direction throughout the whole time
		//
		//TODO code for combining joystick input with align output
		//
		//TODO code for using joystick input to affect align output
		//
		//TODO Set state of align server with printouts in executeCB
		//
		//TODO add more debug printouts
		//
		//TODO Make this configurable
		virtual bool wait_for_mech(ros::Rate r, double timeout) {
			bool waiting = true;
			bool timed_out = false;
			while(waiting && ros::ok() &&!preempted_) {
				timed_out = check_timeout(start_time_, timeout);

				auto state = ac_elevator_.getState();
				if ((state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) ||
					(state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED) ||
					timed_out)
				{
					waiting = false;
					if (timed_out)
						ROS_ERROR_STREAM("align_server move_mech timed out");
					if (state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED)
					{
						ROS_INFO_STREAM("align_server move_mech returned preempted_");
						preempted_ = true;
					}
					if (!ac_elevator_.getResult()->success)
					{
						ROS_INFO_STREAM("align_server move_mech did not succeed");
						preempted_ = true;
					}
					else {
						return true;
					}
				}
				if(as_.isPreemptRequested()){
					preempted_ = true;
				}
				else {
					ros::spinOnce();
					r.sleep();
				}
			}
			return false;
		}
		//TODO make this configurable
		//Function to move mech out of the way of sensors
		virtual bool move_mech(ros::Rate r, bool wait_for_result) {
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = CARGO_SHIP;
			elev_goal.place_cargo = false;
			ac_elevator_.sendGoal(elev_goal);
			if(wait_for_result) {
				return wait_for_mech(r, align_timeout_);
			}
			else {
				return true;
			}
		}

		//Disable all PID nodes
		virtual void disable_pid() {
			std_msgs::Bool false_msg;
			false_msg.data = false;

			enable_align_pub_.publish(false_msg);
			enable_orient_pub_.publish(false_msg);
			enable_x_pub_.publish(false_msg);
			enable_y_pub_.publish(false_msg);
		}

		//Example align function
		virtual bool robot_align() {
			ros::Rate r(30);
			ROS_WARN("starting robot_align");

			start_time_ = ros::Time::now().toSec();
			bool timed_out = false;

			orient_timed_out_ = false;
			x_timed_out_ = false;
			y_timed_out_ = false;

			aligned_ = false;
			orient_aligned_ = false;
			x_aligned_ = false;
			y_aligned_ = false;

			//move mech out of the way
			//move_mech(r, false);
			//enable, wait for alignment, TODO change this timeout, keep enabled
			ROS_WARN("Starting orient align");
			align_orient(r, true, true, align_timeout_, false);
			ROS_WARN("Ending orient align");

			//Check if it timed out or preempted while waiting
			timed_out = check_timeout(start_time_, align_timeout_);
			preempted_ = check_preempted();
			if(preempted_ || timed_out) {
				return false;
			}

			//enable, wait for alignment, default timeout, don't keep enabled
			//align_x(r, true, true);

			////Check if it timed out or preempted while waiting
			//timed_out = check_timeout(start_time_, align_timeout_);
			//preempted_ = check_preempted();
			//if(preempted_ || timed_out) {
			//	return false;
			//}
			
			//enable, wait for alignment, default timeout, don't keep enabled
			ROS_WARN("Starting y align");
			align_y(r, true, true);
			ROS_WARN("ending y align");

			//Check if it timed out or preempted while waiting
			timed_out = check_timeout(start_time_, align_timeout_);
			preempted_ = check_preempted();
			if(preempted_ || timed_out) {
				return false;
			}


			ROS_INFO("Base align class: align succeeded");
			return true;
		}

		//define the function to be executed when the actionlib server is called
		virtual void executeCB(const behaviors::AlignGoalConstPtr &goal) {
			disable_pid();
			bool align_succeeded = robot_align();
			disable_pid(); //Disable all align PID after execution

			if(orient_timed_out_ || y_timed_out_ || x_timed_out_)
			{
				result_.timed_out = true;
				result_.success = false;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted_)
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

/*int main(int argc, char** argv) {
	ros::init(argc, argv, "align_server");

	ros::NodeHandle n;
	ros::NodeHandle n_private_params("~");
	ros::NodeHandle n_params(n, "align_server_params");
    ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_intake_params");

	if(!n_params.getParam("align_timeout_", align_timeout_))
		ROS_ERROR_STREAM("Could not read align_timeout_ in align_server");
	if(!n_params.getParam("orient_timeout", orient_timeout))
		ROS_ERROR_STREAM("Could not read orient_timeout in align_server");
	if(!n_params.getParam("x_timeout", x_timeout))
		ROS_ERROR_STREAM("Could not read x_timeout in align_server");
	if(!n_params.getParam("y_timeout", y_timeout))
		ROS_ERROR_STREAM("Could not read y_timeout in align_server");
	if(!n_params.getParam("orient_error_threshold", orient_error_threshold))
		ROS_ERROR_STREAM("Could not read orient_error_threshold in align_server");
	if(!n_params.getParam("x_error_threshold", x_error_threshold))
		ROS_ERROR_STREAM("Could not read x_error_threshold in align_server");
	if(!n_params.getParam("cargo_error_threshold", y_error_threshold))
		ROS_ERROR_STREAM("Could not read cargo_error_threshold in align_server");

	if(!n_private_params.getParam("debug", debug))
		ROS_ERROR_STREAM("Could not read debug in align_server");


	std::shared_ptr<ros::Publisher> enable_navx_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> cargo_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_y_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_hatch_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_cargo_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_cargo_pub_ = std::make_shared<ros::Publisher>();

	*enable_navx_pub_ = n.advertise<std_msgs::Bool>("orient_pid/pid_enable", 1,  true);
	*hatch_panel_enable_distance_pub_ = n.advertise<std_msgs::Bool>("hatch_panel_distance_pid/pid_enable", 1,  true);
	*cargo_enable_distance_pub_ = n.advertise<std_msgs::Bool>("cargo_distance_pid/pid_enable", 1,  true);
	*enable_y_pub_ = n.advertise<std_msgs::Bool>("align_with_camera/enable_y_pub", 1,  true);
	*enable_cargo_pub_ = n.advertise<std_msgs::Bool>("cargo_pid/pid_enable", 1,  true);
	*enable_align_hatch_pub_ = n.advertise<std_msgs::Bool>("align_hatch_pid/pid_enable", 1,  true);
	*enable_align_cargo_pub_ = n.advertise<std_msgs::Bool>("align_cargo_pid/pid_enable", 1,  true);

	BaseAlignAction align_action("align_server", enable_align_hatch_pub_, enable_navx_pub_, hatch_panel_enable_distance_pub_, enable_y_pub_, "orient_pid/pid_debug", "hatch_panel_distance_pid/pid_debug", "align_with_camera/y_aligned"); //TODO make the error topics params?


	//Stop PID nodes from defaulting true
	// TODO : why not just put this call in the BaseAlignAction constructor, then move
	// all of the publishers to be straight member variables in BaseAlignAction
	std_msgs::Bool false_msg;
	false_msg.data = false;
	enable_navx_pub_.publish(false_msg);
	hatch_panel_enable_distance_pub_.publish(false_msg);
	cargo_enable_distance_pub_.publish(false_msg);
	enable_y_pub_.publish(false_msg);
	enable_cargo_pub_.publish(false_msg);
	enable_align_hatch_pub_.publish(false_msg);
	enable_align_cargo_pub_.publish(false_msg);

    ros::spin();
	return 0;
}*/
