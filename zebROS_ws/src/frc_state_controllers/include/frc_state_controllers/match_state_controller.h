#pragma once

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/match_data_interface.h>
#include "frc_msgs/MatchSpecificData.h"

namespace match_state_controller
{
class MatchStateController: public controller_interface::Controller<hardware_interface::MatchStateInterface>
{
	public:
		MatchStateController() : publish_rate_(15.0) {}

		virtual bool init(hardware_interface::MatchStateInterface *hw,
						ros::NodeHandle					&root_nh,
						ros::NodeHandle					&controller_nh) override;

		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration & ) override;
		virtual void stopping(const ros::Time &time) override;

	private:
		hardware_interface::MatchStateHandle match_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::MatchSpecificData> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;

}; //class

} //namespace
