#include "behaviors/base_align_server.h"

class AlignHatchPanelAction : public BaseAlignAction {
	public:
		AlignHatchPanelAction(const std::string &name,
			const std::shared_ptr<ros::Publisher>& enable_align_pub_,
			const std::shared_ptr<ros::Publisher>& enable_orient_pub_,
			const std::shared_ptr<ros::Publisher>& enable_x_pub_,
			const std::shared_ptr<ros::Publisher>& enable_y_pub_,
			const std::string &orient_error_topic_,
			const std::string &x_error_topic_,
			const std::string &y_error_topic_) :
			BaseAlignAction(name,
				enable_align_pub_,
				enable_orient_pub_,
				enable_x_pub_,
				enable_y_pub_,
				orient_error_topic_,
				x_error_topic_,
				y_error_topic_)
		{
			y_error_ = nh_.subscribe(y_error_topic_, 1, &AlignHatchPanelAction::y_error_cb, this);
		}

		void y_error_cb(const std_msgs::Bool &msg) {
			y_aligned_ = msg.data;
		}
	
		/*HACK TO GET CODE TO COMPILE -- @kevinj please help*/
		void orient_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			orient_aligned_ = (fabs(msg.data[0]) < orient_error_threshold);
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "orient error: " << fabs(msg.data[0]));
		}
		void x_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			x_aligned_ = (fabs(msg.data[0]) < x_error_threshold);
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "x error: " << fabs(msg.data[0]));
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "align_server");

	ros::NodeHandle n;
	ros::NodeHandle n_private_params("~");
	ros::NodeHandle n_params(n, "align_server_params");
    ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_intake_params");

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


	std::shared_ptr<ros::Publisher> enable_align_hatch_pub_ = std::make_shared<ros::Publisher>(); //for the publish_pid_cmd_vel node
	std::shared_ptr<ros::Publisher> enable_navx_pub_ = std::make_shared<ros::Publisher>(); //for PID navx
	std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_ = std::make_shared<ros::Publisher>(); //for PID distance from target
	std::shared_ptr<ros::Publisher> enable_y_pub_ = std::make_shared<ros::Publisher>(); //for the align_with_zed node (not ros_pid node)

	*enable_align_hatch_pub_ = n.advertise<std_msgs::Bool>("align_hatch_pid/pid_enable", 1,  true);
	*enable_navx_pub_ = n.advertise<std_msgs::Bool>("navX_pid/pid_enable", 1,  true);
	*hatch_panel_enable_distance_pub_ = n.advertise<std_msgs::Bool>("hatch_panel_distance_pid/pid_enable", 1,  true);
	*enable_y_pub_ = n.advertise<std_msgs::Bool>("align_with_camera/enable_y_pub", 1,  true);

	AlignHatchPanelAction align_hatch_action("align_server", enable_align_hatch_pub_, enable_navx_pub_, hatch_panel_enable_distance_pub_, enable_y_pub_, "navX_pid/pid_debug", "hatch_panel_distance_pid/pid_debug", "align_with_zed/y_aligned");

	ros::spin();
	return 0;
}
