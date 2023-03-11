//#include <atomic>
//#include <std_srvs/SetBool.h>

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/JoystickState.h>
#include <frc_interfaces/joystick_interface.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace joystick_state_controller
{
class JoystickStateController: public controller_interface::Controller<hardware_interface::JoystickStateInterface>
{
private:
	hardware_interface::JoystickStateHandle joystick_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::JoystickState>> realtime_pub_;
	double publish_rate_{50};
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	frc_msgs::JoystickState prev_joystick_msg_;

	//std::atomic<bool> use16bits_{true};
	//ros::ServiceServer service_;
#if 0
	bool use16bitsCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
	{
		if (req.data)
		{
			use16bits_ = true;
		}
		else
		{
			use16bits_ = false;
		}

		res.success = true;
		res.message = "";
		return true;
	}
#endif

	bool init(hardware_interface::JoystickStateInterface *hw,
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
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN_STREAM("Could not read publish_rate in Joystick state controller, using default " << publish_rate_);
		}
		else if (publish_rate_ < 0.0)
		{
			ROS_ERROR_STREAM("Invalid publist_rate in Joystick state controller (" << publish_rate_ << ")");
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

		//service_ = root_nh.advertiseService("use_16_bits_server", &JoystickStateController::use16bitsCB, this);

		return true;
	}

	void starting(const ros::Time &time)
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration &period)
	{
		if (interval_counter_->update(period))
		{
			if (realtime_pub_->trylock())
			{
				const auto &js = joystick_state_;
				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;
	#if 0
				//ROS_INFO_STREAM("USE 16 bits=" << use16bits_);
				// Still don't want out of bounds reads so keep the getRawAxisCount
				if (js->getRawAxisCount() >= 6 && use16bits_) {
					// axis 3 and 0 lftx
					// 4 and 1 lft y
					// 5 and 2 rht x
					// concatinates the two 8 bit values into 16 bits
	#if 1
					ROS_INFO_STREAM("js->getRawAxis(0) = " << std::hex << (((unsigned int)js->getRawAxis(0)) & 0xff) <<
							" (1) = " << (((unsigned int)js->getRawAxis(1)) & 0xff) <<
							" (2) = " << (((unsigned int)js->getRawAxis(2)) & 0xff) <<
							" (3) = " << (((unsigned int)js->getRawAxis(3)) & 0xff) <<
							" (4) = " << (((unsigned int)js->getRawAxis(4)) & 0xff) <<
							" (5) = " << (((unsigned int)js->getRawAxis(5)) & 0xff));
					ROS_INFO_STREAM("js->getAxis(0) = " << js->getAxis(0) <<
							" (1) = " << js->getAxis(1) <<
							" (2) = " << js->getAxis(2) <<
							" (3) = " << js->getAxis(3) <<
							" (4) = " << js->getAxis(4) <<
							" (5) = " << js->getAxis(5) );
	#endif
					int16_t lftx = (((uint16_t) js->getRawAxis(0)) << 8) | ((uint16_t) js->getRawAxis(3));
					int16_t lfty = (((uint16_t) js->getRawAxis(1)) << 8) | ((uint16_t) js->getRawAxis(4));
					int16_t rhtx = (((uint16_t) js->getRawAxis(2)) << 8) | ((uint16_t) js->getRawAxis(5));
					// hopefully don't get messed up by int division
					m.leftStickX = (lftx > 0) ? lftx / 32767.0 : lftx / 32768.0;
					m.leftStickY = (lfty > 0) ? lfty / 32767.0 : lfty / 32768.0;
					m.rightStickX = (rhtx > 0) ? rhtx / 32767.0 : rhtx / 32768.0;
					// who needs Y!
					m.rightStickY = 0;
				}
				// need to fix joystick on other side, will need to update this
				else
	#endif
				{
					m.leftStickX = js->getAxis(0);
					m.leftStickY = js->getAxis(1);
					// TODO triggers encode as buttons
					m.leftTrigger        = js->getAxis(2);
					m.rightTrigger       = js->getAxis(3);
					m.rightStickX        = js->getAxis(4);
					m.rightStickY        = js->getAxis(5);
				}

				m.buttonAButton      = js->getButton(1);
				m.buttonBButton      = js->getButton(0);
				m.buttonXButton      = js->getButton(3);
				m.buttonYButton      = js->getButton(2);
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
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & )
	{}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(joystick_state_controller::JoystickStateController, controller_interface::ControllerBase)
