#include "behaviors/base_align_with_vision_server.h"
#include <thread>
#include <std_msgs/Float64.h>

bool debug;

/*
class BaseAlignHatchPanelAction : public BaseAlignAction {
	protected:
		std::thread ratioThread;
		ros::Publisher ratio_xy_pub_;
		bool ratio_imposed = false;

		double p0;
		double i0;
		double d0;
		double p1;
		double i1;
		double d1;
	public:
		AlignHatchPanelAction(const std::string &name,

							const std::string &enable_align_topic_,
							const std::string &enable_orient_topic_,
							const std::string &enable_x_topic_,
							const std::string &enable_y_topic_,

							const std::string &orient_error_topic_,
							const std::string &x_error_topic_,
							const std::strout_param_name_,
							const std::string &x_timeout_param_name_,
							const std::string &y_timeout_param_name_,

							const std::string &orient_error_threshold_param_name_,
							const std::string &x_error_threshold_param_name_,
							const std::string &y_error_threshold_param_name_,

							const std::string &ratio_xy_topic_):
			BaseAlignAction(name,
				enable_align_topic_,
				enable_orient_topic_,
				enable_x_topic_,
				enable_y_topic_,

				orient_error_topic_,
				x_error_topic_,
				y_error_topic_,

				align_timeout_param_name_,
				orient_timeout_param_name_,
				x_timeout_param_name_,
				y_timeout_param_name_,

				orient_error_threshold_param_name_,
				x_error_threshold_param_name_,
				y_error_threshold_param_name_)
		{
			if(!nh_.getParam("orient_pid/p0", p0)){
				ROS_ERROR("Align hatch panel failed to load p0");
			}
			if(!nh_.getParam("orient_pid/i0", i0)){
				ROS_ERROR("Align hatch panel failed to load i0");
			}
			if(!nh_.getParam("orient_pid/d0", d0)){
				ROS_ERROR("Align hatch panel failed to load d0");
			}
			if(!nh_.getParam("orient_pid/p1", p1)){
				ROS_ERROR("Align hatch panel failed to load p1");
			}
			if(!nh_.getParam("orient_pid/i1", i1)){
				ROS_ERROR("Align hatch panel failed to load i1");
			}
			if(!nh_.getParam("orient_pid/d1", d1)){
				ROS_ERROR("Align hatch panel failed to load d1");
			}

			if(!ratio_xy_topic_.empty()) {
				ratio_imposed = true;
				ratio_xy_pub_ = nh_.advertise<std_msgs::Float64>(ratio_xy_topic_, 1);
				ratioThread = std::thread(std::bind(&AlignHatchPanelAction::ratioPub, this));
			}
		}
		void ratioPub() {
			ros::Rate r(60);
			while(ros::ok()) {
				std_msgs::Float64 msg;
				if(y_error_ != 0.0 && x_error_ != 0.0) {
					msg.data =-1*y_error_/x_error_;
				}
				else {
					msg.data = 0.0;
				}
				ratio_xy_pub_.publish(msg);
				r.sleep();
			}
		}
<<<<<<< HEAD
=======

>>>>>>> 482153db53edc6698d28c1a5f8ef99306457b22b
		bool robot_align() {
            ros::Rate r(60);
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

            load_new_pid("/align_hatch/orient_pid/set_parameters", p0, d0, i0); //reset pid to stationary pid values
            //move mech out of the way
            //move_mech(r, false);
            //enable, wait for alignment, todo change this timeout, keep enabled
            ROS_WARN("starting orient align");
            align_orient(r, true, true, align_timeout_, true);
            ROS_WARN("ending orient align");

            //check if it timed out or preempted while waiting
            timed_out = check_timeout(start_time_, align_timeout_);
            preempted_ = check_preempted();
            if(preempted_ || timed_out) {
                return false;
            }

            //enable, wait for alignment, default timeout, don't keep enabled
            //align_x(r, true, true);

            ////check if it timed out or preempted while waiting
            //timed_out = check_timeout(start_time_, align_timeout_);
            //preempted_ = check_preempted();
            //if(preempted_ || timed_out) {
            //  return false;
            //}
            //enable,don't wait for alignment, default timeout, don't keep enabled

            load_new_pid("/align_hatch/orient_pid/set_parameters", p1, d1, i1); //Set pid to in motion pid values
            ROS_WARN("starting y align");
            align_y(r, true);
            align_x(r, true, true);
            ROS_WARN("ending y align");

            //check if it timed out or preempted while waiting
            timed_out = check_timeout(start_time_, align_timeout_);
            preempted_ = check_preempted();
            if(preempted_ || timed_out) {
                return false;
            }


            ROS_INFO("base align class: align succeeded");
            return true;
        }
};
*/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_server");

	ros::NodeHandle n;
	ros::NodeHandle n_private_params("~");
	ros::NodeHandle n_params(n, "align_server_params");

	if(!n_private_params.getParam("debug", debug))
		ROS_ERROR_STREAM("Could not read debug in align_server");

	BaseAlignVisionAction align_hatch_action("align_hatch_server",
			"align_hatch_pid/pid_enable",
			"orient_pid/pid_enable",
			"hatch_panel_distance_pid/pid_enable",
			"align_with_camera/enable_pub",

			"orient_pid/pid_debug",
			"hatch_panel_distance_pid/state",
			"align_with_camera/aligned",

			"/align_hatch/align_hatch_params/align_timeout",
			"/align_hatch/align_hatch_params/orient_timeout",
			"/align_hatch/align_hatch_params/x_timeout",
			"/align_hatch/align_hatch_params/y_timeout",

			"/align_hatch/align_hatch_params/orient_error_threshold",
			"/align_hatch/align_hatch_params/x_error_threshold",
			"/align_hatch/align_hatch_params/y_error_threshold",

			"align_hatch_pid/ratio_xy",
			"hatch_panel_distance_pid/y_command",

			"align_hatch/orient_pid/set_parameters");
	ros::spin();
	return 0;
}
