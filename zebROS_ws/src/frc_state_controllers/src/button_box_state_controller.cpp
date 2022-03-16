#include "frc_state_controllers/button_box_state_controller.h"

namespace button_box_state_controller
{

	bool ButtonBoxStateController::init(hardware_interface::JoystickStateInterface *hw,
			ros::NodeHandle						           &root_nh,
			ros::NodeHandle						           &controller_nh)
	{
		ROS_INFO_STREAM_NAMED("button_box_state_controller", "init is running");
		std::string name;
		if (!controller_nh.getParam("name", name))
		{
			ROS_ERROR("Could not read button box name parameter in ButtonBox State Controller");
			return false;
		}

		std::vector<std::string> button_box_names = hw->getNames();
		const auto it = std::find(button_box_names.begin(), button_box_names.end(), name);
		if (it == button_box_names.cend())
		{
			ROS_ERROR_STREAM("Could not find requested name " << name << " in button box interface list");
			return false;
		}
		button_box_state_ = hw->getHandle(*it);
		const auto id = button_box_state_->getId();
		std::stringstream pub_name;
		// TODO : maybe use pub_names instead, or joy id unconditionally?
		pub_name << "js" << id;

		realtime_pub_.reset(new realtime_tools::RealtimePublisher<frc_msgs::ButtonBoxState>(root_nh, pub_name.str(), 1));

		if (!controller_nh.getParam("publish_rate", publish_rate_))
			ROS_WARN_STREAM("Could not read publish_rate in ButtonBox state controller, using default " << publish_rate_);

		return true;
	}

	void ButtonBoxStateController::starting(const ros::Time &time)
	{
		last_publish_time_ = time;
	}

	void ButtonBoxStateController::update(const ros::Time &time, const ros::Duration & )
	{
		if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
		{
			if (realtime_pub_->trylock())
			{
				last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

				const auto &bbs = button_box_state_;
				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				m.lockingSwitchButton    = bbs->getButton(0); // ?
				m.topRedButton           = bbs->getButton(1);
				m.leftRedButton          = bbs->getButton(2);
				m.rightRedButton         = bbs->getButton(3);
				m.leftSwitchUpButton     = bbs->getButton(4);
				m.leftSwitchDownButton   = bbs->getButton(5);
				m.rightSwitchUpButton    = bbs->getButton(6);
				m.rightSwitchDownButton  = bbs->getButton(7);
				m.leftBlueButton         = bbs->getButton(8);
				m.rightBlueButton        = bbs->getButton(9);
				m.yellowButton           = bbs->getButton(10);
				m.leftGreenButton        = bbs->getButton(11);
				m.rightGreenButton       = bbs->getButton(12);
				m.topGreenButton         = bbs->getButton(13);
				m.bottomGreenButton      = bbs->getButton(14);
				m.bottomSwitchUpButton   = bbs->getButton(15);
				m.bottomSwitchDownButton = bbs->getButton(16);

				// Creating press booleans by comparing the last publish to the current one
				m.lockingSwitchPress        = !prev_button_box_msg_.lockingSwitchButton    && m.lockingSwitchButton;
				m.topRedPress               = !prev_button_box_msg_.topRedButton           && m.topRedButton;
				m.leftRedPress              = !prev_button_box_msg_.leftRedButton          && m.leftRedButton;
				m.rightRedPress             = !prev_button_box_msg_.rightRedButton         && m.rightRedButton;
				m.leftSwitchUpPress         = !prev_button_box_msg_.leftSwitchUpButton     && m.leftSwitchUpButton;
				m.leftSwitchDownPress       = !prev_button_box_msg_.leftSwitchDownButton   && m.leftSwitchDownButton;
				m.rightSwitchUpPress        = !prev_button_box_msg_.rightSwitchUpButton    && m.rightSwitchUpButton;
				m.rightSwitchDownPress      = !prev_button_box_msg_.rightSwitchDownButton  && m.rightSwitchDownButton;
				m.leftBluePress             = !prev_button_box_msg_.leftBlueButton         && m.leftBlueButton;
				m.rightBluePress            = !prev_button_box_msg_.rightBlueButton        && m.rightBlueButton;
				m.yellowPress               = !prev_button_box_msg_.yellowButton           && m.yellowButton;
				m.leftGreenPress            = !prev_button_box_msg_.leftGreenButton        && m.leftGreenButton;
				m.rightGreenPress           = !prev_button_box_msg_.rightGreenButton       && m.rightGreenButton;
				m.topGreenPress             = !prev_button_box_msg_.topGreenButton         && m.topGreenButton;
				m.bottomGreenPress          = !prev_button_box_msg_.bottomGreenButton      && m.bottomGreenButton;
				m.bottomSwitchUpPress       = !prev_button_box_msg_.bottomSwitchUpButton   && m.bottomSwitchUpButton;
				m.bottomSwitchDownPress     = !prev_button_box_msg_.bottomSwitchDownButton && m.bottomSwitchDownButton;

				// Creating release booleans by comparing the last publish to the current one
                m.lockingSwitchRelease      = prev_button_box_msg_.lockingSwitchButton    && !m.lockingSwitchButton;
                m.topRedRelease             = prev_button_box_msg_.topRedButton           && !m.topRedButton;
                m.leftRedRelease            = prev_button_box_msg_.leftRedButton          && !m.leftRedButton;
                m.rightRedRelease           = prev_button_box_msg_.rightRedButton         && !m.rightRedButton;
                m.leftSwitchUpRelease       = prev_button_box_msg_.leftSwitchUpButton     && !m.leftSwitchUpButton;
                m.leftSwitchDownRelease     = prev_button_box_msg_.leftSwitchDownButton   && !m.leftSwitchDownButton;
                m.rightSwitchUpRelease      = prev_button_box_msg_.rightSwitchUpButton    && !m.rightSwitchUpButton;
                m.rightSwitchDownRelease    = prev_button_box_msg_.rightSwitchDownButton  && !m.rightSwitchDownButton;
                m.leftBlueRelease           = prev_button_box_msg_.leftBlueButton         && !m.leftBlueButton;
                m.rightBlueRelease          = prev_button_box_msg_.rightBlueButton        && !m.rightBlueButton;
                m.yellowRelease             = prev_button_box_msg_.yellowButton           && !m.yellowButton;
                m.leftGreenRelease          = prev_button_box_msg_.leftGreenButton        && !m.leftGreenButton;
                m.rightGreenRelease         = prev_button_box_msg_.rightGreenButton       && !m.rightGreenButton;
                m.topGreenRelease           = prev_button_box_msg_.topGreenButton         && !m.topGreenButton;
                m.bottomGreenRelease        = prev_button_box_msg_.bottomGreenButton      && !m.bottomGreenButton;
                m.bottomSwitchUpRelease     = prev_button_box_msg_.bottomSwitchUpButton   && !m.bottomSwitchUpButton;
                m.bottomSwitchDownRelease   = prev_button_box_msg_.bottomSwitchDownButton && !m.bottomSwitchDownButton;

				realtime_pub_->unlockAndPublish();
				prev_button_box_msg_ = m;
			}
		}
}

void ButtonBoxStateController::stopping(const ros::Time & )
{}
} // namespace

PLUGINLIB_EXPORT_CLASS(button_box_state_controller::ButtonBoxStateController, controller_interface::ControllerBase)
