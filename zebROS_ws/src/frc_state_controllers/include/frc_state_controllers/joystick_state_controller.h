#ifndef INC_JOYSTICK_STATE_CONTROLLER_H_
#define INC_JOYSTICK_STATE_CONTROLLER_H_

//#include <std_srvs/SetBool.h>
//#include <atomic>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/JoystickState.h>
#include <frc_interfaces/joystick_interface.h>

namespace joystick_state_controller
{
class JoystickStateController: public controller_interface::Controller<hardware_interface::JoystickStateInterface>
{
	public:
		bool init(hardware_interface::JoystickStateInterface *hw,
				  ros::NodeHandle							&root_nh,
				  ros::NodeHandle							&controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & ) override;
		void stopping(const ros::Time &time) override;

	private:
		hardware_interface::JoystickStateHandle joystick_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::JoystickState>> realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_{50};
		frc_msgs::JoystickState prev_joystick_msg_;

		//bool use16bitsCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
		//std::atomic<bool> use16bits_{true};
		//ros::ServiceServer service_;
}; //class
}

#endif
