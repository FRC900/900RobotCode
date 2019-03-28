#include "behaviors/base_align_server.h"
#include <thread>
#include <std_msgs/Float64.h>

class AlignHatchPanelAction : public BaseAlignAction {
	protected:
		std::thread ratioThread;
		ros::Publisher ratio_xy_pub_;
		bool ratio_imposed = false;
	public:
		AlignHatchPanelAction(const std::string &name,

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
#endif
};
// TODO : These probably need to be moved into the base class, along
// with some defaults and a way to set them
double align_timeout;
double orient_timeout;
double x_timeout;
double y_timeout;

double orient_error_threshold;
double x_error_threshold;
double y_error_threshold;

bool debug;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_server");

	ros::NodeHandle n;
	ros::NodeHandle n_private_params("~");
	ros::NodeHandle n_params(n, "align_server_params");

	if(!n_private_params.getParam("debug", debug))
		ROS_ERROR_STREAM("Could not read debug in align_server");

	AlignHatchPanelAction align_hatch_action("align_hatch_server",
			"align_hatch_pid/pid_enable",
			"orient_pid/pid_enable",
			"hatch_panel_distance_pid/pid_enable",
			"align_with_camera/enable_pub",

			"orient_pid/pid_debug",
			"hatch_panel_distance_pid/pid_debug",
			"align_with_camera/aligned",

			"/align_server/align_hatch_params/align_timeout",
			"/align_server/align_hatch_params/orient_timeout",
			"/align_server/align_hatch_params/x_timeout",
			"/align_server/align_hatch_params/y_timeout",

			"/align_server/align_hatch_params/orient_error_threshold",
			"/align_server/align_hatch_params/x_error_threshold",
			"/align_server/align_hatch_params/y_error_threshold",
			"align_hatch_pid/ratio_xy");
	ros::spin();
	return 0;
}
