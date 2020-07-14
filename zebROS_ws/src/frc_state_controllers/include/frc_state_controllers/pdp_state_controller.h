#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/PDPData.h>
#include <frc_interfaces/pdp_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>

namespace pdp_state_controller
{
class PDPStateController: public controller_interface::Controller<hardware_interface::PDPStateInterface>
{
	public:
		PDPStateController() : publish_rate_(15.0) {
                }

		virtual bool init(hardware_interface::PDPStateInterface *hw,
							ros::NodeHandle						&root_nh,
							ros::NodeHandle						&controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration & ) override;
		virtual void stopping(const ros::Time &time) override;

	private:
		hardware_interface::PDPStateHandle pdp_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::PDPData> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;

}; //class

} //namespace
