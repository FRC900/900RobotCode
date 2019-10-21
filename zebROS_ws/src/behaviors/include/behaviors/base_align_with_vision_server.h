#include "behaviors/base_align_server.h"
#include <thread>
#include <std_msgs/Float64.h>

class BaseAlignVisionAction : public BaseAlignAction {
	protected:
		std::thread ratioThread;
		ros::Publisher ratio_xy_pub_;
		ros::Publisher constant_vel_pub_;
		//bool ratio_imposed_ = false;

		std::string reconfigure_orient_pid_topic_;

		double p0_;
		double i0_;
		double d0_;
		double p1_;
		double i1_;
		double d1_;

		bool update_ratio_ = false;

		//TESTING PARAMS !!DANGER!!
		bool do_orient_ = false;
		bool track_target_ = false;
		bool do_pid_ = false;
		bool hold_orient_ = false;
		bool do_align_flag_ = false;

		double constant_vel_ = 0.0;
		double after_orient_delay_ = 1.0;
	public:
		BaseAlignVisionAction(const std::string &name,

							const std::string &enable_align_topic_,
#if 0
							const std::string &enable_orient_topic_,
							const std::string &enable_x_topic_,
							const std::string &enable_y_topic_,

							const std::string &orient_error_topic_,
							const std::string &x_error_topic_,
							const std::string &y_error_topic_,
#endif

							const std::string &align_timeout_param_name_,
#if 0
							const std::string &orient_timeout_param_name_,
							const std::string &x_timeout_param_name_,
							const std::string &y_timeout_param_name_,

							const std::string &orient_error_threshold_param_name_,
							const std::string &x_error_threshold_param_name_,
							const std::string &y_error_threshold_param_name_,
#endif

							const std::string &ratio_xy_topic_,
							const std::string &constant_vel_topic_,

							const std::string &reconfigure_orient_pid_topic):
			BaseAlignAction(name,
				enable_align_topic_,
#if 0
				enable_orient_topic_,
				enable_x_topic_,
				enable_y_topic_,

				orient_error_topic_,
				x_error_topic_,
				y_error_topic_,
#endif

				align_timeout_param_name_
#if 0
				orient_timeout_param_name_,
				x_timeout_param_name_,
				y_timeout_param_name_,

				orient_error_threshold_param_name_,
				x_error_threshold_param_name_,
				y_error_threshold_param_name_
#endif
					),
			 reconfigure_orient_pid_topic_(reconfigure_orient_pid_topic)
		{
			if(!nh_.getParam("orient_pid/p0", p0_)){
				ROS_ERROR("BaseAlignVision failed to load p0");
			}
			if(!nh_.getParam("orient_pid/i0", i0_)){
				ROS_ERROR("BaseAlignVision failed to load i0");
			}
			if(!nh_.getParam("orient_pid/d0", d0_)){
				ROS_ERROR("BaseAlignVision failed to load d0");
			}
			if(!nh_.getParam("orient_pid/p1", p1_)){
				ROS_ERROR("BaseAlignVision failed to load p1");
			}
			if(!nh_.getParam("orient_pid/i1", i1_)){
				ROS_ERROR("BaseAlignVision failed to load i1");
			}
			if(!nh_.getParam("orient_pid/d1", d1_)){
				ROS_ERROR("BaseAlignVision failed to load d1");
			}
			//TESTING PARAMS !DANGER!
			if(!nh_.getParam("do_orient", do_orient_)){
				ROS_ERROR("BaseAlignVision failed to load do_orient");
			}
			if(!nh_.getParam("track_target", track_target_)){
				ROS_ERROR("BaseAlignVision failed to load track_target");
			}
			if(!nh_.getParam("do_pid", do_pid_)){
				ROS_ERROR("BaseAlignVision failed to load do_pid");
			}
			if(!nh_.getParam("hold_orient", hold_orient_)){
				ROS_ERROR("BaseAlignVision failed to load hold_orient");
			}
			if(!nh_.getParam("constant_vel", constant_vel_)){
				ROS_ERROR("BaseAlignVision failed to load constant_vel");
			}
			if(!nh_.getParam("do_align", do_align_flag_)){
				ROS_ERROR("BaseAlignVision failed to load do_align");
			}
			if(!nh_.getParam("after_orient_delay", after_orient_delay_)){
				ROS_ERROR("BaseAlignVision failed to load after_orient_delay");
			}

			constant_vel_pub_ = nh_.advertise<std_msgs::Float64>(constant_vel_topic_, 1);
			if(!ratio_xy_topic_.empty()) {
				//ratio_imposed_ = true;
				ratio_xy_pub_ = nh_.advertise<std_msgs::Float64>(ratio_xy_topic_, 1);
				ratioThread = std::thread(std::bind(&BaseAlignVisionAction::ratioPub, this));
			}
		}

		void ratioPub() {
			ros::Rate r(60);
			while(ros::ok()) {
				if(update_ratio_) {
					const double y_error = get_axis_error("y");
					const double x_error = get_axis_error("x");
					ROS_WARN_THROTTLE(0.25, "Ratio pub: y_error: %f x_error:%f", y_error, x_error);
					if((y_error != 0.0 && x_error != 0.0) || track_target_) {
						if(x_error > 0.2 || track_target_) {
							std_msgs::Float64 msg;
							msg.data = (y_error)/(x_error);
							ratio_xy_pub_.publish(msg);
						}
						else
						{
							//ROS_INFO_STREAM("Not publishing ratio because x_error < 0.2");
						}
					}
					else {
						//ROS_INFO_STREAM("Not publishing ratio because x_error or y_error is 0");
					}
					update_ratio_ = false;
				}
				else{
					ROS_INFO_THROTTLE(0.25, "Not publishing ratio because update_ratio is false");
				}
				r.sleep();
			}
		}

		bool robot_align() {
			update_ratio_ = false;
            ros::Rate r(60);
            ROS_WARN("starting robot_align");

            start_time_ = ros::Time::now().toSec();
            bool timed_out = false;

			// TODO - make me a base class method
			for (auto &axis : axis_states_)
			{
				axis.second.timed_out_ = false; // TODO - should probably just be cleared at the top of the loop in do_orient()?
				                                //         and if so, then probably could be made a local there?
				axis.second.aligned_ = false; //TODO - doesn't do anything - will be overwritten by erreor_term_cb?
			}

            aligned_ = false; // TODO - never used? If so, remove from here and the base class

            load_new_pid(reconfigure_orient_pid_topic_, p0_, d0_, i0_); //reset pid to stationary pid values
            //move mech out of the way
            //move_mech(r, false);
            //enable, wait for alignment, todo change this timeout, keep enabled
			if(do_orient_) {
				ROS_WARN("starting orient align");
				do_align("orient", r, true, true, align_timeout_, hold_orient_);
				ROS_WARN("ending orient align");
				ros::Duration(after_orient_delay_).sleep();
			}
			else {
				ROS_ERROR("SKIPPING ORIENT DUE TO TESTING PARAM!!!");
				ROS_ERROR("SKIPPING ORIENT DUE TO TESTING PARAM!!!");
				ROS_ERROR("SKIPPING ORIENT DUE TO TESTING PARAM!!!");
				ROS_ERROR("SKIPPING ORIENT DUE TO TESTING PARAM!!!");
			}

            //check if it timed out or preempted while waiting
            timed_out = check_timeout(start_time_, align_timeout_);
            preempted_ = check_preempted();
            if(preempted_ || timed_out) {
				ROS_INFO_STREAM(__FILE__ << ":" << __LINE__ << " robot align returning false : preempted_ = "
						<< preempted_ << " timed_out = " << timed_out);
                return false;
            }

			if(!do_align_flag_) {
				return false;
			}
            //enable, wait for alignment, default timeout, don't keep enabled
            //do_align("x", r, true, true);

            ////check if it timed out or preempted while waiting
            //timed_out = check_timeout(start_time_, align_timeout_);
            //preempted_ = check_preempted();
            //if(preempted_ || timed_out) {
            //  return false;
            //}
            //enable,don't wait for alignment, default timeout, don't keep enabled

			load_new_pid(reconfigure_orient_pid_topic_, p1_, d1_, i1_); //Set pid to in motion pid values
			if(track_target_) {
				// TODO : how to exit this loop successfully?
				while(ros::ok() && !preempted_) {
					ROS_ERROR_THROTTLE(0.2, "CONSTANTLY TRACKING TARGET DUE TO TESTING CONFIG IN ALIGN SERVER!!!!");
					preempted_ = check_preempted();
					if(!do_pid_) {
						ROS_ERROR_THROTTLE(0.2, "RUNNING CONSTANT VEL DUE TO TESTING CONFIG IN ALIGN SERVER!!!!");

                        enable_align(true);
						std_msgs::Float64 constant_vel_msg;
						constant_vel_msg.data = constant_vel_;
						constant_vel_pub_.publish(constant_vel_msg);
					}
					else {
						do_align("y", r, true);
						do_align("x", r, true);
					}
					r.sleep();
					update_ratio_ = true;
				}
			}
			else if(!do_pid_) {
				update_ratio_ = true;
				timed_out = false;
				start_time_ = ros::Time::now().toSec();
				// TODO : how to exit this loop successfully?
				while(ros::ok() && !timed_out && !preempted_) {
					ROS_ERROR_THROTTLE(0.2, "RUNNING CONSTANT VEL DUE TO TESTING CONFIG IN ALIGN SERVER!!!!");
					timed_out = check_timeout(start_time_, align_timeout_);
					preempted_ = check_preempted();

                    enable_align(true);
					std_msgs::Float64 constant_vel_msg;
					constant_vel_msg.data = constant_vel_;
					constant_vel_pub_.publish(constant_vel_msg);
				}
			}
			else {
				update_ratio_ = true;
				ROS_WARN("starting y align");
				do_align("y", r, true);
				do_align("x", r, true, true, align_timeout_, true);
				ROS_WARN("ending y align");
			}
			// TODO : when to set enable_align(false)?

            //check if it timed out or preempted while waiting
			if (!timed_out && !preempted_)
			{
				timed_out = check_timeout(start_time_, align_timeout_);
				preempted_ = check_preempted();
			}
            if(preempted_ || timed_out) {
				ROS_ERROR_STREAM("Base align with vision server timed out or preempted : preempted_ = "
						<< preempted_ << " timed_out = " << timed_out);
                return false;
            }

            ROS_INFO("base align class: align succeeded");
            return true;
        }
};

