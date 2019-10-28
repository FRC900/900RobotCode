// File which creates an align hatch panel actionlib service
// Most of the interesting code is in base_align_with_vision_server.h
// This mainly just creates that class with the correct topics
// for aligning with the cargoship using the ZED camera
//
#include "behaviors/base_align_with_vision_server.h"
#include <thread>
#include <std_msgs/Float64.h>

// TODO - make a member of the align class
bool debug = true;

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
			"/align_hatch/align_hatch_params/align_timeout",
			"align_hatch_pid/ratio_xy",
			"hatch_panel_distance_pid/y_command",
			"/align_hatch/orient_pid/set_parameters");

	// X-distance is hooked up to terabee distance
	// coming from the align_with_terabee node
	if (!align_hatch_action.addAxis(AlignActionAxisConfig("x",
				"hatch_panel_distance_pid/pid_enable",
				"hatch_panel_distance_pid/state",
				"/align_hatch/align_hatch_params/x_timeout",
				"/align_hatch/align_hatch_params/x_error_threshold"
				)))
	{
		ROS_ERROR("Could not addAxis(x) in align_hatch_panels");
		return -1;
	}
	// Y-distance is hooked up to camera data coming from
	// the align_with_camera node
	if (!align_hatch_action.addAxis(AlignActionAxisConfig("y",
				"align_with_camera/enable_pub",
				"align_with_camera/aligned",
				"/align_hatch/align_hatch_params/y_timeout",
				"/align_hatch/align_hatch_params/y_error_threshold"
				)))
	{
		ROS_ERROR("Could not addAxis(y) in align_hatch_panels");
		return -1;
	}
	// orient is using the snap to angle code
	if (!align_hatch_action.addAxis(AlignActionAxisConfig("orient",
				"orient_pid/pid_enable",
				"orient_pid/pid_debug",
				"/align_hatch/align_hatch_params/orient_timeout",
				"/align_hatch/align_hatch_params/orient_error_threshold"
				)))
	{
		ROS_ERROR("Could not addAxis(orient) in align_hatch_panels");
		return -1;
	}
	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
