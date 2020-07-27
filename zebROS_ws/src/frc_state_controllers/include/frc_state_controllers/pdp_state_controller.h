#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_buffer.h>
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
		PDPStateController() : publish_rate_(15.0)
		{
        }

		bool init(hardware_interface::PDPStateInterface *hw,
				  ros::NodeHandle						&root_nh,
				  ros::NodeHandle						&controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & ) override;
		void stopping(const ros::Time &time) override;

	private:
		hardware_interface::PDPStateHandle pdp_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::PDPData> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;

}; //class
}

namespace state_listener_controller
{
class PDPStateListenerController :
	public controller_interface::Controller<hardware_interface::RemotePDPStateInterface>
{
	public:
		PDPStateListenerController();
		~PDPStateListenerController();

		bool init(hardware_interface::RemotePDPStateInterface *hw, ros::NodeHandle &n) override;
		void starting(const ros::Time & /*time*/) override;
		void stopping(const ros::Time & /*time*/) override;
		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override;

	private:
		ros::Subscriber sub_command_;
		hardware_interface::PDPWritableStateHandle handle_;

		// Real-time buffer holds the last command value read from the "command" topic.
		realtime_tools::RealtimeBuffer<hardware_interface::PDPHWState> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		virtual void commandCB(const frc_msgs::PDPDataConstPtr &msg);
};
} //namespace
