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

double align_timeout;
double orient_timeout;
double orient_error_threshold;
double x_error_threshold;
double cargo_error_threshold;


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
class AlignAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignAction> as_; //create the actionlib server
		std::string action_name_;
		// TODO this result should be a local var
		behaviors::AlignResult result_; //variable to store result of the actionlib action
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;
        
		std::shared_ptr<ros::Publisher> enable_navx_pub_;
		std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_;
		std::shared_ptr<ros::Publisher> cargo_enable_distance_pub_;
		std::shared_ptr<ros::Publisher> enable_y_pub_;
		std::shared_ptr<ros::Publisher> enable_align_hatch_pub_;
		std::shared_ptr<ros::Publisher> enable_align_cargo_pub_;
		std::shared_ptr<ros::Publisher> enable_cargo_pub_;

		ros::Subscriber navx_error_sub_;
		ros::Subscriber hatch_panel_distance_error_sub_;
		ros::Subscriber cargo_distance_error_sub_;
		ros::Subscriber y_error_sub_;
		ros::Subscriber cargo_error_sub_;

        ros::ServiceClient BrakeSrv;

		bool hatch_panel_distance_aligned_ = false;
		bool cargo_distance_aligned_ = false;
		bool y_aligned_ = false;
		bool cargo_aligned_ = false;
		bool orient_aligned_ = false;
		bool distance_aligned_ = false;

	public:
		//make the executeCB function run every time the actionlib server is called
		AlignAction(const std::string &name,
			const std::shared_ptr<ros::Publisher>& enable_navx_pub_,
			const std::shared_ptr<ros::Publisher>& hatch_panel_enable_distance_pub_,
			const std::shared_ptr<ros::Publisher>& cargo_enable_distance_pub_,
			const std::shared_ptr<ros::Publisher>& enable_y_pub_,
			const std::shared_ptr<ros::Publisher>& enable_align_hatch_pub_,
			const std::shared_ptr<ros::Publisher>& enable_align_cargo_pub_,
			const std::shared_ptr<ros::Publisher>& enable_cargo_pub_):
			as_(nh_, name, boost::bind(&AlignAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true),
			enable_navx_pub_(enable_navx_pub_),
			hatch_panel_enable_distance_pub_(hatch_panel_enable_distance_pub_),
			cargo_enable_distance_pub_(cargo_enable_distance_pub_),
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
			hatch_panel_distance_error_sub_ = nh_.subscribe("hatch_panel_distance_pid/pid_debug", 1, &AlignAction::hatch_panel_distance_error_cb, this);
			cargo_distance_error_sub_ = nh_.subscribe("cargo_distance_pid/pid_debug", 1, &AlignAction::cargo_distance_error_cb, this);
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
			//ROS_WARN_STREAM_THROTTLE(1, "navX error: " << fabs(msg.data[0]));
		}
		void hatch_panel_distance_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			hatch_panel_distance_aligned_ = (fabs(msg.data[0]) < x_error_threshold);
			//ROS_WARN_STREAM("distance error: " << msg.data[0]);
		}
		void cargo_distance_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			cargo_distance_aligned_ = (fabs(msg.data[0]) < x_error_threshold);
			ROS_ERROR_STREAM("distance error: " << msg.data[0]);
		}
		void y_error_cb(const std_msgs::Bool &msg)
		{
			y_aligned_ = msg.data;
		}
		void cargo_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			cargo_aligned_ = (fabs(msg.data[0]) < cargo_error_threshold);
			//ROS_WARN_STREAM_THROTTLE(1, "cargo error: " << msg.data[0]);
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::AlignGoalConstPtr &goal) {
			ROS_INFO_STREAM("align server callback called");
			ros::Rate r(30);

			double start_time = ros::Time::now().toSec();
			bool preempted = false;
			bool timed_out = false;
			bool orient_timed_out = false;
			bool aligned = false;

			orient_aligned_ = false;
			distance_aligned_ = false;
			cargo_aligned_ = false;
			hatch_panel_distance_aligned_ = false;
			cargo_distance_aligned_ = false;
			y_aligned_ = false;

			//move elevator up a bit before aligning(terabees r sad :( )
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = CARGO_SHIP; //TODO fix this add to enum in include file
			elev_goal.place_cargo = false;
			ac_elevator_.sendGoal(elev_goal);


			//test if we got a preempt while waiting
			if(as_.isPreemptRequested())
			{
				preempted = true;
			}

			while(!aligned && !preempted && !timed_out && ros::ok())
			{
				ros::spinOnce();
				r.sleep();

				//Define enable messages for pid nodes
				std_msgs::Bool orient_msg;
				std_msgs::Bool distance_msg;
				std_msgs::Bool terabee_msg;
				std_msgs::Bool enable_align_msg;

				orient_msg.data = true && !orient_timed_out;							    //Publish true to the navX pid node throughout the action until orient_timed_out


				enable_align_msg.data = !aligned;										    //Enable publishing pid vals until aligned

				//CARGO VS HATCH PANEL
				//only difference is enable_cargo_pub_ vs enable_y_pub_
				//
				//Publish enable messages
				enable_navx_pub_->publish(orient_msg);

				if(goal->has_cargo) {
					enable_align_cargo_pub_->publish(enable_align_msg);
                }
                else {
					enable_align_hatch_pub_->publish(enable_align_msg);
                }
				/*
				if(goal->has_cargo) {
					distance_msg.data = (orient_aligned_ || orient_timed_out) && !cargo_distance_aligned_;	//Enable distance pid once orient is aligned or timed out
					cargo_enable_distance_pub_->publish(distance_msg);
					terabee_msg.data = (orient_aligned_ || orient_timed_out) && cargo_distance_aligned_;     //Enable terabee node when distance is aligned and  orient aligns or orient times out
					ROS_ERROR_STREAM("Terabee_msg cargo: " << ((orient_aligned_ || orient_timed_out) && cargo_distance_aligned_));
					enable_cargo_pub_->publish(terabee_msg);
					enable_align_cargo_pub_->publish(enable_align_msg);
					aligned = cargo_distance_aligned_ && cargo_aligned_;	 //Check aligned
				}
				else {
					distance_msg.data = (orient_aligned_ || orient_timed_out) && !hatch_panel_distance_aligned_;	//Enable distance pid once orient is aligned or timed out
					hatch_panel_enable_distance_pub_->publish(distance_msg);
					terabee_msg.data = (orient_aligned_ || orient_timed_out) && hatch_panel_distance_aligned_;     //Enable terabee node when distance is aligned and  orient aligns or orient times out
					enable_y_pub_->publish(terabee_msg);
					enable_align_hatch_pub_->publish(enable_align_msg);
					aligned = hatch_panel_distance_aligned_ && y_aligned_;		//Check aligned
				}
				*/

				//Check timed out
				timed_out = (ros::Time::now().toSec() - start_time) > align_timeout;
				orient_timed_out = (ros::Time::now().toSec() - start_time) > orient_timeout;
				//Check preempted
				preempted = as_.isPreemptRequested();

				//Non-spammy debug statements
				if(!orient_aligned_)
					ROS_INFO_THROTTLE(1, "Orienting");
				else
					ROS_INFO_STREAM_THROTTLE(1, "Translating + Orienting. Hatch panel distance aligned: " << hatch_panel_distance_aligned_ << " Cargo distance aligned: " << cargo_distance_aligned_ << " Cutout aligned: " << y_aligned_);
				if(orient_timed_out)
					ROS_ERROR_STREAM_THROTTLE(1, "Orient timed out!");
			}
			//Stop robot after aligning
			bool success = true;
			std_srvs::Empty empty;
			if (!BrakeSrv.call(empty))
			{
				ROS_ERROR("brakeSrv call failed in align_server");
				success = false;
			}

			//Stop all PID after aligning
			// TODO : just set starting back to true?
			std_msgs::Bool false_msg;
			false_msg.data = false;

			enable_navx_pub_->publish(false_msg);
			hatch_panel_enable_distance_pub_->publish(false_msg);
			cargo_enable_distance_pub_->publish(false_msg);
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
			// Only for the case where the BrakeSrv call failed
			// Does this make sense?
			else if (!success)
			{
				result_.timed_out = false;
				result_.success = false;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Not Successful", action_name_.c_str());
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
    ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_intake_params");

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
	std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> cargo_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_y_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_hatch_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_cargo_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_cargo_pub_ = std::make_shared<ros::Publisher>();

	*enable_navx_pub_ = n.advertise<std_msgs::Bool>("navX_pid/pid_enable", 1,  true);
	*hatch_panel_enable_distance_pub_ = n.advertise<std_msgs::Bool>("hatch_panel_distance_pid/pid_enable", 1,  true);
	*cargo_enable_distance_pub_ = n.advertise<std_msgs::Bool>("cargo_distance_pid/pid_enable", 1,  true);
	*enable_y_pub_ = n.advertise<std_msgs::Bool>("align_with_terabee/enable_y_pub", 1,  true);
	*enable_cargo_pub_ = n.advertise<std_msgs::Bool>("cargo_pid/pid_enable", 1,  true);
	*enable_align_hatch_pub_ = n.advertise<std_msgs::Bool>("align_hatch_pid/pid_enable", 1,  true);
	*enable_align_cargo_pub_ = n.advertise<std_msgs::Bool>("align_cargo_pid/pid_enable", 1,  true);

	AlignAction align_action("align_server", enable_navx_pub_, hatch_panel_enable_distance_pub_, cargo_enable_distance_pub_, enable_y_pub_, enable_align_hatch_pub_, enable_align_cargo_pub_, enable_cargo_pub_);

	ros::Rate r(20); // TODO : not used

	//Stop PID nodes from defaulting true
	// TODO : why not just put this call in the AlignAction constructor, then move
	// all of the publishers to be straight member variables in AlignAction
	std_msgs::Bool false_msg;
	false_msg.data = false;
	enable_navx_pub_->publish(false_msg);
	hatch_panel_enable_distance_pub_->publish(false_msg);
	cargo_enable_distance_pub_->publish(false_msg);
	enable_y_pub_->publish(false_msg);
	enable_cargo_pub_->publish(false_msg);
	enable_align_hatch_pub_->publish(false_msg);
	enable_align_cargo_pub_->publish(false_msg);

    ros::spin();
	return 0;
}
