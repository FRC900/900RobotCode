#include "frc_state_controllers/joystick_state_controller.h"

namespace joystick_state_controller
{

bool JoystickStateController::init(hardware_interface::JoystickStateInterface *hw,
								ros::NodeHandle						&root_nh,
								ros::NodeHandle						&controller_nh)
{
	ROS_INFO_STREAM_NAMED("joystick_state_controller", "init is running");
	std::string name;
	if (!controller_nh.getParam("name", name))
	{
		ROS_ERROR("Could not read joystick name parameter in Joystick State Controller");
		return false;
	}

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

	realtime_pub_.reset(new realtime_tools::RealtimePublisher<frc_msgs::JoystickState>(root_nh, pub_name.str(), 1));

	if (!controller_nh.getParam("publish_rate", publish_rate_))
		ROS_WARN_STREAM("Could not read publish_rate in Joystick state controller, using default " << publish_rate_);

	return true;
}

void JoystickStateController::starting(const ros::Time &time)
{
	last_publish_time_ = time;
}

void JoystickStateController::update(const ros::Time &time, const ros::Duration & )
{
	if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
	{
		if (realtime_pub_->trylock())
		{
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

			const auto &js = joystick_state_;
			auto &m = realtime_pub_->msg_;

			m.header.stamp = time;

			m.leftStickX         = js->getAxis(0);
			m.leftStickY         = js->getAxis(1);
			m.leftTrigger        = js->getAxis(2);
			m.rightTrigger       = js->getAxis(3);
			m.rightStickX        = js->getAxis(4);
			m.rightStickY        = js->getAxis(5);

			m.buttonAButton      = js->getButton(0);
			m.buttonBButton      = js->getButton(1);
			m.buttonXButton      = js->getButton(2);
			m.buttonYButton      = js->getButton(3);
			m.bumperLeftButton   = js->getButton(4);
			m.bumperRightButton  = js->getButton(5);
			m.buttonBackButton   = js->getButton(6);
			m.buttonStartButton  = js->getButton(7);
			m.stickLeftButton    = js->getButton(8);
			m.stickRightButton   = js->getButton(9);

			m.buttonAPress       = !prev_joystick_msg_.buttonAButton     && m.buttonAButton;
			m.buttonBPress       = !prev_joystick_msg_.buttonBButton     && m.buttonBButton;
			m.buttonXPress       = !prev_joystick_msg_.buttonXButton     && m.buttonXButton;
			m.buttonYPress       = !prev_joystick_msg_.buttonYButton     && m.buttonYButton;
			m.bumperLeftPress    = !prev_joystick_msg_.bumperLeftButton  && m.bumperLeftButton;
			m.bumperRightPress   = !prev_joystick_msg_.bumperRightButton && m.bumperRightButton;
			m.buttonBackPress    = !prev_joystick_msg_.buttonBackButton  && m.buttonBackButton;
			m.buttonStartPress   = !prev_joystick_msg_.buttonStartButton && m.buttonStartButton;
			m.stickLeftPress     = !prev_joystick_msg_.stickLeftButton   && m.stickLeftButton;
			m.stickRightPress    = !prev_joystick_msg_.stickRightButton  && m.stickRightButton;

			m.buttonARelease     = prev_joystick_msg_.buttonAButton     && !m.buttonAButton;
			m.buttonBRelease     = prev_joystick_msg_.buttonBButton     && !m.buttonBButton;
			m.buttonXRelease     = prev_joystick_msg_.buttonXButton     && !m.buttonXButton;
			m.buttonYRelease     = prev_joystick_msg_.buttonYButton     && !m.buttonYButton;
			m.bumperLeftRelease  = prev_joystick_msg_.bumperLeftButton  && !m.bumperLeftButton;
			m.bumperRightRelease = prev_joystick_msg_.bumperRightButton && !m.bumperRightButton;
			m.buttonBackRelease  = prev_joystick_msg_.buttonBackButton  && !m.buttonBackButton;
			m.buttonStartRelease = prev_joystick_msg_.buttonStartButton && !m.buttonStartButton;
			m.stickLeftRelease   = prev_joystick_msg_.stickLeftButton   && !m.stickLeftButton;
			m.stickRightRelease  = prev_joystick_msg_.stickRightButton  && !m.stickRightButton;

			bool joystick_up = false;
			bool joystick_down = false;
			bool joystick_left = false;
			bool joystick_right = false;
			switch (js->getPOV(0))
			{
				case 0 :
					joystick_up = true;
					break;
				case 45:
					joystick_up = true;
					joystick_right = true;
					break;
				case 90:
					joystick_right = true;
					break;
				case 135:
					joystick_down = true;
					joystick_right = true;
					break;
				case 180:
					joystick_down = true;
					break;
				case 225:
					joystick_down = true;
					joystick_left = true;
					break;
				case 270:
					joystick_left = true;
					break;
				case 315:
					joystick_up = true;
					joystick_left = true;
					break;
				default:
					break;
			}

			m.directionUpButton  = joystick_up;
			m.directionUpPress   = joystick_up && !prev_joystick_msg_.directionUpButton;
			m.directionUpRelease = !joystick_up && prev_joystick_msg_.directionUpButton;

			m.directionDownButton  = joystick_down;
			m.directionDownPress   = joystick_down && !prev_joystick_msg_.directionDownButton;
			m.directionDownRelease = !joystick_down && prev_joystick_msg_.directionDownButton;

			m.directionLeftButton  = joystick_left;
			m.directionLeftPress   = joystick_left && !prev_joystick_msg_.directionLeftButton;
			m.directionLeftRelease = !joystick_left && prev_joystick_msg_.directionLeftButton;

			m.directionRightButton  = joystick_right;
			m.directionRightPress   = joystick_right && !prev_joystick_msg_.directionRightButton;
			m.directionRightRelease = !joystick_right && prev_joystick_msg_.directionRightButton;

			realtime_pub_->unlockAndPublish();
			prev_joystick_msg_ = m;
		}
	}
}

void JoystickStateController::stopping(const ros::Time & )
{}
} // namespace

PLUGINLIB_EXPORT_CLASS(joystick_state_controller::JoystickStateController, controller_interface::ControllerBase)
