#pragma once
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "behaviors/align_axis_config.h"
#include "behaviors/align_axis_state.h"
#include "behaviors/AlignGoal.h"
#include "behaviors/AlignAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/PlaceGoal.h"
#include "behaviors/PlaceAction.h"
#include "behaviors/enumerated_elevator_indices.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

extern bool debug;

//bool startup = true; //disable all pid nodes on startup
class BaseAlignAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignAction> as_; //create the actionlib server
		std::string action_name_;
		// TODO this result should be a local var
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_; //Action client for controlling the elevato
		actionlib::SimpleActionClient<behaviors::PlaceAction> ac_outtake_hatch_panel_;

		//Publishers for enabling PID loops and cmd_vel combiner
		ros::Publisher enable_align_pub_;	//Enables the cmd_vel combiner
		ros::Subscriber elevator_setpoint_sub_;

		//Service client for forcefully stopping the robot
        ros::ServiceClient BrakeSrv_;

		std::map<std::string, AlignActionAxisState> axis_states_;
		//Set default state for align variables
		//
		//Whether we are aligned variables
		bool aligned_ = false;				//overall aligned

		//timeouts for overall align and specific axes
		double align_timeout_ = 0.0;		//overall align timeout
		//is align preempted
		bool preempted_ = false;

		//start time of align
		double start_time_ = -1.0;
		bool placed_ = false;
		int elevator_cur_setpoint_ = 0;
		bool place_after_align_ = false;
		double min_error_to_place_ = 0.1;

	public:
		//make the executeCB function run every time the actionlib server is called
		BaseAlignAction(const std::string &name,

						//Topics that enable align PID loops
						const std::string &enable_align_topic_, // global - turns on / off the publish_pid_cmd_vel node's output
						const std::string &place_after_align_name_,
						const std::string &min_error_to_place_name_,

						//Paramater name with align timeouts
						const std::string &align_timeout_param_name_ // global timeout for entire server CB
		)

			: as_(nh_, name, boost::bind(&BaseAlignAction::executeCB, this, _1), false)
			, action_name_(name)
			, ac_elevator_("/elevator/elevator_server", true)		//TODO maybe make this a generic part of the robot
			, ac_outtake_hatch_panel_("/hatch_outtake/outtake_hatch_panel_server", true)

			//Create publishers on the enable PID loop topics
			, enable_align_pub_(nh_.advertise<std_msgs::Bool>(enable_align_topic_, 1, true))
			, elevator_setpoint_sub_(nh_.subscribe("/teleop/elevator_setpoint", 1, &BaseAlignAction::elevatorSetpointCB, this))
		{
            as_.start();

            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

			//Create service client for abruptly stopping the robot
			BrakeSrv_ = nh_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);

			//Get the timeout parameters
			if(!nh_.getParam(align_timeout_param_name_, align_timeout_))
				ROS_ERROR_STREAM("Could not read align_timeout_param_name_ in align_server");
			if(!nh_.getParam(place_after_align_name_, place_after_align_))
				ROS_ERROR_STREAM("Could not read place_after_align in align_server");
			if(!nh_.getParam(min_error_to_place_name_, min_error_to_place_))
				ROS_ERROR_STREAM("Could not read min_error_to_place in align_server");
		}

		~BaseAlignAction(void)
		{
		}

		bool addAxis(const AlignActionAxisConfig &axis_config)
		{
			double timeout;
			if(!nh_.getParam(axis_config.timeout_param_, timeout))
			{
				ROS_ERROR_STREAM("Could not read param "
						<< axis_config.timeout_param_
						<< " in align_server");
				return false;
			}
			double error_threshold;
			if(!nh_.getParam(axis_config.error_threshold_param_, error_threshold))
			{
				ROS_ERROR_STREAM("Could not read param "
						<< axis_config.error_threshold_param_
						<< " in align_server");
				return false;
			}

			axis_states_.emplace(std::make_pair(axis_config.name_,
				AlignActionAxisState(axis_config.name_,
						nh_,
						axis_config.enable_pub_topic_,
						axis_config.error_sub_topic_,
						boost::bind(&BaseAlignAction::error_term_cb, this, _1, axis_config.name_),
						timeout,
						error_threshold)));
			return true;
		}


#if 0 // TODO - not used
		bool get_axis_timed_out(const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in get_axis_timed_out");
				return false;
			}
			return axis_it->second.timed_out_;
		}
#endif
		double get_axis_error(const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in get_axis_error");
				return std::numeric_limits<double>::max();
			}
			return axis_it->second.error_;
		}

		//functions to check for preempts and timeouts
		bool check_timeout(double start_time, double timeout) {
			if(ros::Time::now().toSec() - start_time > timeout) {
				return true;
			}
			return false;
		}
		bool check_preempted() {
			if(preempted_ || as_.isPreemptRequested()) {
				return true;
			}
			return false;
		}

		//Function to dynamically load a new set of PID on a ros PID node
		//TODO make this work
		virtual void load_new_pid(std::string reconfigure_topic, double p_, double i_, double d_) {
			dynamic_reconfigure::ReconfigureRequest srv_req;
			dynamic_reconfigure::ReconfigureResponse srv_resp;
			dynamic_reconfigure::Config conf;

			dynamic_reconfigure::DoubleParameter p;
			dynamic_reconfigure::DoubleParameter i;
			dynamic_reconfigure::DoubleParameter d;
			dynamic_reconfigure::DoubleParameter p_scale;
			dynamic_reconfigure::DoubleParameter i_scale;
			dynamic_reconfigure::DoubleParameter d_scale;

			p_scale.name = "Kp_scale";
			p_scale.value = 100.0;
			i_scale.name = "Ki_scale";
			i_scale.value = 10.0;
			d_scale.name = "Kd_scale";
			d_scale.value = 10.0;

			p.name = "Kp";
			p.value = p_ / p_scale.value;
			i.name = "Ki";
			i.value = i_ / i_scale.value;
			d.name = "Kd";
			d.value = d_ / d_scale.value;

			conf.doubles.push_back(p_scale);
			conf.doubles.push_back(p);
			conf.doubles.push_back(i_scale);
			conf.doubles.push_back(i);
			conf.doubles.push_back(d_scale);
			conf.doubles.push_back(d);

			srv_req.config = conf;
			ros::service::call(reconfigure_topic, srv_req, srv_resp);
		}

		virtual void error_term_cb(const std_msgs::Float64MultiArrayConstPtr &msg,
			const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in error_term_cb");
				return;
			}
			auto &axis = axis_it->second;
			//Check if error less than threshold
			axis.aligned_ = (fabs(msg->data[0]) < axis.error_threshold_);
			axis.error_ = msg->data[0];
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, name << " error: " << axis.error_ << " aligned: " << axis.aligned_);
		}

		void elevatorSetpointCB(std_msgs::Int8 msg)
		{
			elevator_cur_setpoint_ = msg.data;
		}
		//Functions to enable align PID
		virtual void enable_align(bool enable=true) {
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_align_pub_.publish(enable_msg);
		}

		virtual void do_align(const std::string &name, ros::Rate &r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			ROS_INFO_STREAM("Running " << name << " do_align");
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.cend())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in do_align");
				return;
			}
			auto &axis = axis_it->second;
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			axis.enable_pub_.publish(enable_msg);
			if (debug)
				ROS_INFO_STREAM("do_align(" << name  << ") : set enable_pub = " << enable);

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !axis.aligned_ && !preempted_ && !axis.timed_out_) {
					enable_align(); // TODO - move outside loop?
					axis.timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					if(place_after_align_ && name == "x")
					{
						double error = get_axis_error(name);
						if(error < min_error_to_place_ && !placed_ && error != 0.0) {
							ROS_INFO_STREAM("Placing because axis " << name << " has error " << error);
							place_game_piece();
							placed_ = true;
						}
					}
					if (debug)
						ROS_INFO_STREAM("do_align(" << name
								<< ") : axis.aligned_ = " << axis.aligned_
								<< " axis.timed_out_ = " << axis.timed_out_
								<< " start_time_ = " << start_time_
								<< " timeout = " << timeout
								<< " reempted_ = " << preempted_);
					if (!axis.timed_out_ && !preempted_)
						r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				axis.enable_pub_.publish(enable_msg);
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
			while(waiting && ros::ok() &&!preempted_) {
				const bool timed_out = check_timeout(start_time_, timeout);

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
		//TODO : probably best to do as the start of a
		// robot_align in a derived class, which then calls
		// the base class robot_align
		//
		// Or maybe add a generic pre_robot_align method
		// which can be overriden in the derived classes
		// It can be empty for the base class and then
		// modified as needed by other conditions
		//Function to move mech out of the way of sensors
		virtual bool move_mech(ros::Rate r, bool wait_for_result) {
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = INTAKE;
			elev_goal.place_cargo = false;
			ac_elevator_.sendGoal(elev_goal);
			if(wait_for_result) {
				return wait_for_mech(r, align_timeout_);
			}
			else {
				return true;
			}
		}

		virtual void place_game_piece()
		{
			behaviors::PlaceGoal goal;
			goal.setpoint_index = elevator_cur_setpoint_;
            goal.end_setpoint_index = INTAKE;
			ac_outtake_hatch_panel_.sendGoal(goal);
		}


		//Disable all PID nodes
		virtual void disable_pid() {
			std_msgs::Bool false_msg;
			false_msg.data = false;

			enable_align_pub_.publish(false_msg);
			for (auto &axis : axis_states_)
				axis.second.enable_pub_.publish(false_msg);
		}

		// Example align function
		// Probably best to make this pure virtual and require
		// any classes using this to define a custom one
		virtual bool robot_align()
		{
			ros::Rate r(60);
			ROS_WARN("starting robot_align");

			start_time_ = ros::Time::now().toSec();
			bool timed_out = false;

			for (auto &axis : axis_states_)
			{
				axis.second.timed_out_ = false;
				axis.second.aligned_ = false;
			}

			aligned_ = false;

			//move mech out of the way
			//move_mech(r, false);
			//enable, wait for alignment, TODO change this timeout, keep enabled
			ROS_WARN("Starting orient align");
			do_align("orient", r, true, true, align_timeout_, true);
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
			//enable,don't wait for alignment, default timeout, don't keep enabled
			ROS_WARN("Starting y align");
			do_align("y", r, true);
			do_align("x", r, true, true);
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
			double start_time = ros::Time::now().toSec();
			disable_pid();
			ros::Rate r(20);
			bool align_succeeded = robot_align();
			bool timed_out = false;
			preempted_ = false;
			while(!timed_out && ros::ok()) {
				timed_out = ros::Time::now().toSec() - start_time > 0.1;
				r.sleep();
			}
			disable_pid(); //Disable all align PID after execution

			behaviors::AlignResult result; //variable to store result of the actionlib action

			if (std::any_of(axis_states_.cbegin(), axis_states_.cend(),
						[](const auto &axis) { return axis.second.timed_out_; } ))
			{
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted_)
			{
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else //implies succeeded
			{
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);

				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			return;
		}
};

