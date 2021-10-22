#ifndef BUTTON_BOX_STATE_CONTROLLER_INC_
#define BUTTON_BOX_STATE_CONTROLLER_INC_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/ButtonBoxState.h>
#include <frc_interfaces/joystick_interface.h>
#include <pluginlib/class_list_macros.h>

namespace button_box_state_controller
{
class ButtonBoxStateController: public controller_interface::Controller<hardware_interface::JoystickStateInterface>
{
	public:
		ButtonBoxStateController()
		{
        }

		bool init(hardware_interface::JoystickStateInterface *hw,
				  ros::NodeHandle							 &root_nh,
				  ros::NodeHandle							 &controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & ) override;
		void stopping(const ros::Time &time) override;

	private:
		hardware_interface::JoystickStateHandle button_box_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::ButtonBoxState>> realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_{50};
		frc_msgs::ButtonBoxState prev_button_box_msg_;
}; //class
}

#endif
