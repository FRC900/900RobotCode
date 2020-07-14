#pragma once

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_mode_interface.h>
#include "frc_msgs/JointMode.h"

namespace joint_mode_state_controller
{
class JointModeStateController: public controller_interface::Controller<hardware_interface::JointModeInterface>
{
	public:
		JointModeStateController() : publish_rate_(100.0) {}

		virtual bool init(hardware_interface::JointModeInterface *hw,
						ros::NodeHandle							 &root_nh,
						ros::NodeHandle							 &controller_nh) override;

		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration & ) override;
		virtual void stopping(const ros::Time &time) override;

	private:
		std::vector<hardware_interface::JointModeHandle> joint_modes_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::JointMode>> realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;

}; //class

} //namespace
