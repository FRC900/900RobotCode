#include "behaviors/base_align_server.h"

class AlignHatchPanelAction : public BaseAlignAction {
	public:
		void y_error_cb(const std_msgs::Bool &msg) {
			y_aligned_ = msg.data;
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_hatch_server");

	ros::NodeHandle n;

	std::shared_ptr<ros::Publisher> enable_align_hatch_pub_ = std::make_shared<ros::Publisher>(); //for the publish_pid_cmd_vel node
	std::shared_ptr<ros::Publisher> enable_navx_pub_ = std::make_shared<ros::Publisher>(); //for PID navx
	std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_ = std::make_shared<ros::Publisher>(); //for PID distance from target
	std::shared_ptr<ros::Publisher> enable_y_pub_ = std::make_shared<ros::Publisher>(); //for the align_with_zed node (not ros_pid node)

	*enable_align_hatch_pub_ = n.advertise<std_msgs::Bool>("align_hatch_pid/pid_enable", 1,  true);
	*enable_navx_pub_ = n.advertise<std_msgs::Bool>("navX_pid/pid_enable", 1,  true);
	*hatch_panel_enable_distance_pub_ = n.advertise<std_msgs::Bool>("hatch_panel_distance_pid/pid_enable", 1,  true);
	*enable_y_pub_ = n.advertise<std_msgs::Bool>("align_with_terabee/enable_y_pub", 1,  true);

	AlignHatchPanelAction align_hatch_action("align_hatch_server", enable_align_hatch_pub_, enable_navx_pub_, hatch_panel_enable_distance_pub_, enable_y_pub_, "navX_pid/pid_debug", "hatch_panel_distance_pid/pid_debug", "align_with_zed/y_aligned");

	ros::spin();
	return 0;
}
