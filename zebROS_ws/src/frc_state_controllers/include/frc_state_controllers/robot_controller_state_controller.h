#pragma once
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/robot_controller_interface.h>
#include <frc_msgs/RobotControllerData.h>

namespace robot_controller_state_controller
{
    class RobotControllerStateController: public controller_interface::Controller<hardware_interface::RobotControllerStateInterface>
	{
	public:
		RobotControllerStateController() : publish_rate_(0.0) {}

		virtual bool init(hardware_interface::RobotControllerStateInterface *hw,
						  ros::NodeHandle                                   &root_nh,
						  ros::NodeHandle                                   &controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &/*time*/, const ros::Duration & /*period*/) override;
		virtual void stopping(const ros::Time & /*time*/) override;

	private:
		hardware_interface::RobotControllerStateHandle rc_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::RobotControllerData>> realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;
	};
}

