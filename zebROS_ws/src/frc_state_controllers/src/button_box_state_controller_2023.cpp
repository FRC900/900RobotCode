#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/ButtonBoxState2023.h>
#include <frc_interfaces/joystick_interface.h>
#include <pluginlib/class_list_macros.h>
#include "periodic_interval_counter/periodic_interval_counter.h"

namespace button_box_state_controller
{
class ButtonBoxStateController_2023 : public controller_interface::Controller<hardware_interface::JoystickStateInterface>
{
private:
	hardware_interface::JoystickStateHandle button_box_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::ButtonBoxState2023>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{50};
	frc_msgs::ButtonBoxState2023 prev_button_box_msg_;

public:
	ButtonBoxStateController_2023() = default;
	bool init(hardware_interface::JoystickStateInterface *hw,
			 ros::NodeHandle						     &root_nh,
			 ros::NodeHandle						     &controller_nh) override
	{
		ROS_INFO_STREAM_NAMED("button_box_state_controller_2023", "init is running");
		std::string name;
		if (!controller_nh.getParam("name", name))
		{
			ROS_ERROR("Could not read button box name parameter in ButtonBox State Controller");
			return false;
		}

		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN_STREAM("Could not read publish_rate in ButtonBox state controller, using default " << publish_rate_);
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invliad publish_rate in ButtonBox state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

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

		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::ButtonBoxState2023>>(root_nh, pub_name.str(), 1);

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
				const auto &bbs = button_box_state_;
				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				// If the buttons are pressed
				m.lockingSwitchButton = bbs->getButton(0);
				m.rightGreenButton = bbs->getButton(1);
				m.leftGreenButton = bbs->getButton(2);
				m.topGreenButton = bbs->getButton(3);
				m.bottomGreenButton = bbs->getButton(4);
				m.bottomRightWhiteButton = bbs->getButton(5);
				m.bottomLeftYellowButton = bbs->getButton(6);
				m.redButton = bbs->getButton(7);
				m.centralYellowButton = bbs->getButton(8);
				m.heightSelectSwitchDownButton = bbs->getButton(9);
				m.heightSelectSwitchUpButton = bbs->getButton(10);
				m.heightSelectSwitchLeftButton = bbs->getButton(11);
				m.heightSelectSwitchRightButton = bbs->getButton(13);
				// Skip 13 (that's the Arduino LED)
				m.gridSelectConeRightButton = bbs->getButton(14);
				m.gridSelectCubeButton = bbs->getButton(15);
				m.gridSelectConeLeftButton = bbs->getButton(16);
				m.topRightCubeButton = bbs->getButton(17);
				m.topMiddleConeButton = bbs->getButton(18);
				m.topLeftConeButton = bbs->getButton(19);

				// Creating press booleans by comparing the last publish to the current one
				m.lockingSwitchPress				= !prev_button_box_msg_.lockingSwitchButton				&& m.lockingSwitchButton;
				m.rightGreenPress					= !prev_button_box_msg_.rightGreenButton				&& m.rightGreenButton;
				m.leftGreenPress					= !prev_button_box_msg_.leftGreenButton					&& m.leftGreenButton;
				m.topGreenPress						= !prev_button_box_msg_.topGreenButton					&& m.topGreenButton;
				m.bottomGreenPress					= !prev_button_box_msg_.bottomGreenButton				&& m.bottomGreenButton;
				m.bottomRightWhitePress				= !prev_button_box_msg_.bottomRightWhiteButton			&& m.bottomRightWhiteButton;
				m.bottomLeftYellowPress				= !prev_button_box_msg_.bottomLeftYellowButton			&& m.bottomLeftYellowButton;
				m.redPress							= !prev_button_box_msg_.redButton						&& m.redButton;
				m.centralYellowPress				= !prev_button_box_msg_.centralYellowButton				&& m.centralYellowButton;
				m.heightSelectSwitchDownPress		= !prev_button_box_msg_.heightSelectSwitchDownButton	&& m.heightSelectSwitchDownButton;
				m.heightSelectSwitchUpPress			= !prev_button_box_msg_.heightSelectSwitchUpButton		&& m.heightSelectSwitchUpButton;
				m.heightSelectSwitchLeftPress		= !prev_button_box_msg_.heightSelectSwitchLeftButton	&& m.heightSelectSwitchLeftButton;
				m.heightSelectSwitchRightPress		= !prev_button_box_msg_.heightSelectSwitchRightButton	&& m.heightSelectSwitchRightButton;
				m.gridSelectConeRightPress			= !prev_button_box_msg_.gridSelectConeRightButton		&& m.gridSelectConeRightButton;
				m.gridSelectCubePress				= !prev_button_box_msg_.gridSelectCubeButton			&& m.gridSelectCubeButton;
				m.gridSelectConeLeftPress			= !prev_button_box_msg_.gridSelectConeLeftButton		&& m.gridSelectConeLeftButton;
				m.topRightCubePress					= !prev_button_box_msg_.topRightCubeButton				&& m.topRightCubeButton;
				m.topMiddleConePress				= !prev_button_box_msg_.topMiddleConeButton				&& m.topMiddleConeButton;
				m.topLeftConePress					= !prev_button_box_msg_.topLeftConeButton				&& m.topLeftConeButton;

				// Creating release booleans by comparing the last publish to the current one
				m.lockingSwitchRelease				= prev_button_box_msg_.lockingSwitchButton				&& !m.lockingSwitchButton;
				m.rightGreenRelease					= prev_button_box_msg_.rightGreenButton					&& !m.rightGreenButton;
				m.leftGreenRelease					= prev_button_box_msg_.leftGreenButton					&& !m.leftGreenButton;
				m.topGreenRelease					= prev_button_box_msg_.topGreenButton					&& !m.topGreenButton;
				m.bottomGreenRelease				= prev_button_box_msg_.bottomGreenButton				&& !m.bottomGreenButton;
				m.bottomRightWhiteRelease			= prev_button_box_msg_.bottomRightWhiteButton			&& !m.bottomRightWhiteButton;
				m.bottomLeftYellowRelease			= prev_button_box_msg_.bottomLeftYellowButton			&& !m.bottomLeftYellowButton;
				m.redRelease						= prev_button_box_msg_.redButton						&& !m.redButton;
				m.centralYellowRelease				= prev_button_box_msg_.centralYellowButton				&& !m.centralYellowButton;
				m.heightSelectSwitchDownRelease		= prev_button_box_msg_.heightSelectSwitchDownButton		&& !m.heightSelectSwitchDownButton;
				m.heightSelectSwitchUpRelease		= prev_button_box_msg_.heightSelectSwitchUpButton		&& !m.heightSelectSwitchUpButton;
				m.heightSelectSwitchLeftRelease		= prev_button_box_msg_.heightSelectSwitchLeftButton		&& !m.heightSelectSwitchLeftButton;
				m.heightSelectSwitchRightRelease	= prev_button_box_msg_.heightSelectSwitchRightButton	&& !m.heightSelectSwitchRightButton;
				m.gridSelectConeRightRelease		= prev_button_box_msg_.gridSelectConeRightButton		&& !m.gridSelectConeRightButton;
				m.gridSelectCubeRelease				= prev_button_box_msg_.gridSelectCubeButton				&& !m.gridSelectCubeButton;
				m.gridSelectConeLeftRelease			= prev_button_box_msg_.gridSelectConeLeftButton			&& !m.gridSelectConeLeftButton;
				m.topRightCubeRelease				= prev_button_box_msg_.topRightCubeButton				&& !m.topRightCubeButton;
				m.topMiddleConeRelease				= prev_button_box_msg_.topMiddleConeButton				&& !m.topMiddleConeButton;
				m.topLeftConeRelease				= prev_button_box_msg_.topLeftConeButton				&& !m.topLeftConeButton;

				realtime_pub_->unlockAndPublish();
				prev_button_box_msg_ = m;
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & ) override
	{
	}

}; //class

} // namespace

PLUGINLIB_EXPORT_CLASS(button_box_state_controller::ButtonBoxStateController_2023, controller_interface::ControllerBase)
