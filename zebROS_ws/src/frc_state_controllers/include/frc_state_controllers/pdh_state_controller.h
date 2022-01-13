#ifndef PDH_STATE_CONTROLLER_INC_
#define PDH_STATE_CONTROLLER_INC_

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/PDHData.h>
#include <frc_interfaces/pdh_state_interface.h>

namespace pdh_state_controller
{
class PDHStateController: public controller_interface::Controller<hardware_interface::PDHStateInterface>
{
	public:
		bool init(hardware_interface::PDHStateInterface *hw,
				  ros::NodeHandle						&root_nh,
				  ros::NodeHandle						&controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration &) override;
		void stopping(const ros::Time &time) override;

	private:
		hardware_interface::PDHStateHandle pdh_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::PDHData> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_{20};

}; //class
}

namespace state_listener_controller
{
class PDHStateListenerController :
	public controller_interface::Controller<hardware_interface::RemotePDHStateInterface>
{
	public:
		PDHStateListenerController();
		~PDHStateListenerController();

		bool init(hardware_interface::RemotePDHStateInterface *hw, ros::NodeHandle &n) override;
		void starting(const ros::Time & /*time*/) override;
		void stopping(const ros::Time & /*time*/) override;
		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override;

	private:
		ros::Subscriber sub_command_;
		hardware_interface::PDHWritableStateHandle handle_;

		// Real-time buffer holds the last command value read from the "command" topic.
		realtime_tools::RealtimeBuffer<hardware_interface::PDHHWState> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		virtual void commandCB(const frc_msgs::PDHDataConstPtr &msg);
};
} //namespace
#endif
