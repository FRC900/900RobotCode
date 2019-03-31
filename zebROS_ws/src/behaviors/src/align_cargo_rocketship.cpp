#include "behaviors/base_align_server.h"

class AlignCargoRocketshipAction : public BaseAlignAction {
	public:
		AlignCargoRocketshipAction(const std::string &name,
							  const std::string &enable_align_topic_,
							  const std::string &enable_orient_topic_,
							  const std::string &enable_x_topic_,
							  const std::string &enable_y_topic_,
							  const std::string &orient_error_topic_,
							  const std::string &x_error_topic_,
							  const std::string &y_error_topic_) :
			BaseAlignAction(name,
				enable_align_topic_,
				enable_orient_topic_,
				enable_x_topic_,
				enable_y_topic_,
				orient_error_topic_,
				x_error_topic_,
				y_error_topic_)
		{
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_server");

	ros::NodeHandle n;
	ros::NodeHandle n_private_params("~");
	ros::NodeHandle n_params(n, "align_server_params");

	if(!n_params.getParam("align_timeout", align_timeout))
		ROS_ERROR_STREAM("Could not read align_timeout in align_server");
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

	AlignCargoRocketshipAction align_cargo_rocketship_action("align_cargo_rocketship_server",
			"align_pid/pid_enable",
			"orient_pid/pid_enable",
			"cargo_distance_pid/pid_enable",
			"align_with_camera/enable_y_pub",
			"orient_pid/pid_debug",
			"cargo_distance_pid/pid_debug",
			"align_with_camera/y_aligned");

	ros::spin();
	return 0;
}
