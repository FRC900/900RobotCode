#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/JoystickState.h>
#include <frc_interfaces/joystick_interface.h>
#include <pluginlib/class_list_macros.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace spacemouse_state_controller
{
class SpacemouseStateController: public controller_interface::Controller<hardware_interface::JoystickStateInterface>
{
bool init(hardware_interface::JoystickStateInterface *hw,
		  ros::NodeHandle							 &root_nh,
		  ros::NodeHandle							 &controller_nh) override
{
	ROS_INFO_STREAM_NAMED("spacemouse_state_controller", "init is running");
	std::string name;
	if (!controller_nh.getParam("name", name))
	{
		ROS_ERROR("Could not read joystick name parameter in Spacemouse State Controller");
		return false;
	}
	if (!controller_nh.getParam("publish_rate", publish_rate_))
	{
		ROS_WARN_STREAM("Could not read publish_rate in Spacemouse State Controller, using default " << publish_rate_);
	}
	else if (publish_rate_ <= 0.0)
	{
		ROS_ERROR_STREAM("Invalid publish rate in Spacemouse State Controller (" << publish_rate_ << ")");
		return false;
	}
	interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

	std::vector<std::string> joystick_names = hw->getNames();
	const auto it = std::find(joystick_names.begin(), joystick_names.end(), name);
	if (it == joystick_names.cend())
	{
		ROS_ERROR_STREAM("Could not find requested name " << name << " in joystick interface list");
		return false;
	}
	joystick_state_ = hw->getHandle(*it);
	const auto id = joystick_state_->getId();
	std::stringstream pub_name;
	// TODO : maybe use pub_names instead, or joy id unconditionally?
	pub_name << "js" << id;

	realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::JoystickState>>(root_nh, pub_name.str(), 1);

	return true;
}

void starting(const ros::Time &time) override
{
	interval_counter_->reset();
}

void update(const ros::Time &time, const ros::Duration &period) override
{
	if (interval_counter_->update(period))
	{
		if (realtime_pub_->trylock())
		{
			const auto &js = joystick_state_;
			auto &m = realtime_pub_->msg_;

			m.header.stamp = time;

			m.leftStickX         = js->getAxis(0);
			m.leftStickY         = js->getAxis(1);
			m.leftTrigger        = 0;
			m.rightTrigger       = 0;
			m.rightStickX        = js->getAxis(5);
			m.rightStickY        = 0;

			m.buttonAButton      = false;
			m.buttonBButton      = false;
			m.buttonXButton      = false;
			m.buttonYButton      = false;
			m.bumperLeftButton   = js->getAxis(2) < 0.75; // hack - default to slow mode on z-axis push?
			m.bumperRightButton  = false;
			m.buttonBackButton   = false;
			m.buttonStartButton  = false;
			m.stickLeftButton    = false;
			m.stickRightButton   = false;

			m.buttonAPress       = false;
			m.buttonBPress       = false;
			m.buttonXPress       = false;
			m.buttonYPress       = false;
			m.bumperLeftPress    = !prev_left_bumper_ && m.bumperLeftButton;
			m.bumperRightPress   = false;
			m.buttonBackPress    = false;
			m.buttonStartPress   = false;
			m.stickLeftPress     = false;
			m.stickRightPress    = false;

			m.buttonARelease     = false;
			m.buttonBRelease     = false;
			m.buttonXRelease     = false;
			m.buttonYRelease     = false;
			m.bumperLeftRelease  = prev_left_bumper_ && !m.bumperLeftButton;
			m.bumperRightRelease = false;
			m.buttonBackRelease  = false;
			m.buttonStartRelease = false;
			m.stickLeftRelease   = false;
			m.stickRightRelease  = false;

			m.directionUpButton  = false;
			m.directionUpPress   = false;
			m.directionUpRelease = false;

			m.directionDownButton  = false;
			m.directionDownPress   = false;
			m.directionDownRelease = false;

			m.directionLeftButton  = false;
			m.directionLeftPress   = false;
			m.directionLeftRelease = false;

			m.directionRightButton  = false;
			m.directionRightPress   = false;
			m.directionRightRelease = false;

			realtime_pub_->unlockAndPublish();

			prev_left_bumper_ = m.bumperLeftButton;
		}
		else
		{
			interval_counter_->force_publish();
		}
	}
}

void stopping(const ros::Time & ) override
{}

private:
		hardware_interface::JoystickStateHandle joystick_state_;
		std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::JoystickState>> realtime_pub_;
		std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
		double publish_rate_{50};
		bool prev_left_bumper_{false};
}; // class

} // namespace

PLUGINLIB_EXPORT_CLASS(spacemouse_state_controller::SpacemouseStateController, controller_interface::ControllerBase)
