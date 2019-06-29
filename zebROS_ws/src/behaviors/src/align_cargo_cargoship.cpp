#include "behaviors/base_align_with_vision_server.h"

bool debug;
/*
class AlignCargoCargoshipAction : public BaseAlignAction {
	public:
		AlignCargoCargoshipAction(const std::string &name,

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
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_cargo_cargoship_server");

	ros::NodeHandle n;
	ros::NodeHandle n_private_params("~");
	ros::NodeHandle n_params(n, "align_server_params");

	if(!n_private_params.getParam("debug", debug))
		ROS_ERROR_STREAM("Could not read debug in align_server");

	BaseAlignVisionAction align_cargo_cargoship_action("align_cargo_cargoship_server",
			"align_pid/pid_enable",
			"orient_pid/pid_enable",
			"cargo_distance_pid/pid_enable",
			"align_with_camera/enable_y_pub",

			"orient_pid/pid_debug",
			"cargo_distance_pid/state",
			"align_with_camera/y_aligned",

            "/align_cargo/align_cargo_params/align_timeout",
            "/align_cargo/align_cargo_params/orient_timeout",
            "/align_cargo/align_cargo_params/x_timeout",
            "/align_cargo/align_cargo_params/y_timeout",

            "/align_cargo/align_cargo_params/orient_error_threshold",
            "/align_cargo/align_cargo_params/x_error_threshold",
            "/align_cargo/align_cargo_params/y_error_threshold",

			"align_cargo_pid/ratio_xy",
			"cargo_distance_pid/x_command",

			"align_cargo_cargoship/orient_pid/set_parameters");

	ros::spin();
	return 0;
}
