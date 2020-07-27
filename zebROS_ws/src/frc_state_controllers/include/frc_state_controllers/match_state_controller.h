#pragma once

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/match_data_interface.h>
#include "frc_msgs/MatchSpecificData.h"

namespace match_state_controller
{
class MatchStateController: public controller_interface::Controller<hardware_interface::MatchStateInterface>
{
	public:
		MatchStateController() : publish_rate_(15.0) {}

		bool init(hardware_interface::MatchStateInterface *hw,
				  ros::NodeHandle						  &root_nh,
				  ros::NodeHandle						  &controller_nh) override;

		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & ) override;
		void stopping(const ros::Time &time) override;

	private:
		hardware_interface::MatchStateHandle match_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::MatchSpecificData> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;

}; //class

} //namespace match_state_controller

namespace state_listener_controller
{
class MatchStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteMatchStateInterface>
{
	public:
		MatchStateListenerController();
		~MatchStateListenerController();

		bool init(hardware_interface::RemoteMatchStateInterface *hw, ros::NodeHandle &n) override;
		void starting(const ros::Time & /*time*/) override;
		void stopping(const ros::Time & /*time*/) override;
		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override;

	private:
		ros::Subscriber sub_command_;
		hardware_interface::MatchStateWritableHandle handle_;

		// Real-time buffer holds the last command value read from the "command" topic.
		realtime_tools::RealtimeBuffer<hardware_interface::MatchHWState> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		void commandCB(const frc_msgs::MatchSpecificDataConstPtr &msg);
};

} // namespace state_listener_controller
