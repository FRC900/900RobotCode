#include "behaviors/base_align_with_vision_server.h"

bool debug = false;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_cargo_cargoship_server");

	ros::NodeHandle n_private_params("~");

	if(!n_private_params.getParam("debug", debug))
		ROS_ERROR_STREAM("Could not read debug in align_server");

	BaseAlignVisionAction align_cargo_cargoship_action("align_cargo_cargoship_server",
			"align_pid/pid_enable",

            "/align_cargo/align_cargo_params/align_timeout",

			"align_cargo_pid/ratio_xy",
			"cargo_distance_pid/x_command",

			"align_cargo_cargoship/orient_pid/set_parameters");

	align_cargo_cargoship_action.addAxis(AlignActionAxisConfig("x",
			"cargo_distance_pid/pid_enable",
			"cargo_distance_pid/state",
            "/align_cargo/align_cargo_params/x_timeout",
            "/align_cargo/align_cargo_params/x_error_threshold"));
	align_cargo_cargoship_action.addAxis(AlignActionAxisConfig("y",
			"align_with_camera/enable_y_pub",
			"align_with_camera/y_aligned",
            "/align_cargo/align_cargo_params/y_timeout",
            "/align_cargo/align_cargo_params/y_error_threshold"));
	align_cargo_cargoship_action.addAxis(AlignActionAxisConfig("orient",
			"orient_pid/pid_enable",
			"orient_pid/pid_debug",
			"/align_hatch/align_hatch_params/orient_timeout",
			"/align_hatch/align_hatch_params/orient_error_threshold"
			));


	ros::spin();
	return 0;
}
