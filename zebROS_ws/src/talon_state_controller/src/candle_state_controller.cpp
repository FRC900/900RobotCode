#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ctre_interfaces/candle_state_interface.h>
#include <talon_state_msgs/CANdleStateArray.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace candle_state_controller
{
class CANdleStateController: public controller_interface::Controller<hardware_interface::candle::CANdleStateInterface>
{
private:
	std::vector<hardware_interface::candle::CANdleStateHandle> candle_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::CANdleStateArray>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the CANdleStateInterface

public:
	bool init(hardware_interface::candle::CANdleStateInterface *hw,
			  ros::NodeHandle &root_nh,
			  ros::NodeHandle &controller_nh) override
	{
		// get all joint names from the hardware interface
		const std::vector<std::string> &joint_names = hw->getNames();
		num_hw_joints_ = joint_names.size();
		for (size_t i = 0; i < num_hw_joints_; i++)
			ROS_DEBUG("Got joint %s", joint_names[i].c_str());

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set in candle state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish rate in candle state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<talon_state_msgs::CANdleStateArray>>(root_nh, "candle_states", 2);

		auto &m = realtime_pub_->msg_;

		// get joints and allocate message
		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			candle_state_.push_back(hw->getHandle(joint_names[i]));
			m.candles.emplace_back();
			m.candles[i].name = joint_names[i];
			m.candles[i].can_id = candle_state_.back()->getDeviceID();
		}

		return true;
	}

	void starting(const ros::Time &time) override
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration & period) override
	{
		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				auto &msg = realtime_pub_->msg_;
				msg.header.stamp = time;
				for (size_t i = 0; i < num_hw_joints_; i++)
				{
					const auto &cs = candle_state_[i];
					auto &m = msg.candles[i];

					m.brightness = cs->getBrightness();
					m.status_led_when_active = cs->getStatusLEDWhenActive();
					m.enabled_5v = cs->getEnabled();

					m.leds.clear();

					m.leds.resize(cs->getLEDCount());
					for (size_t led_idx = 0; led_idx < cs->getLEDCount(); led_idx++)
					{
						auto &led_msg = m.leds[led_idx];
						auto led = cs->getLED(led_idx);
						if (led.has_value())
						{
							const auto led_value = led->getColour();
							if (led_value.has_value())
							{
								led_msg.red = led_value->red_;
								led_msg.green = led_value->green_;
								led_msg.blue = led_value->blue_;
								led_msg.white = led_value->white_;
								led_msg.valid = true;
							}
							else
							{
								led_msg.valid = false;
							}
						}
					}
					m.animations.clear();
					m.animations.resize(cs->getAnimationCount());
					for (size_t anim_idx = 0; anim_idx < cs->getAnimationCount(); anim_idx++)
					{
						auto &anim_msg = m.animations[anim_idx];
						auto anim = cs->getAnimation(anim_idx);
						if (anim.has_value())
						{
							anim_msg.speed = anim->speed_;
							anim_msg.start = anim->start_;
							anim_msg.count = anim->count_;
							switch(anim->type_)
							{
								case hardware_interface::candle::AnimationType::ColourFlow:
									anim_msg.type = "ColourFlow";
									break;
								case hardware_interface::candle::AnimationType::Fire:
									anim_msg.type = "Fire";
									break;
								case hardware_interface::candle::AnimationType::Larson:
									anim_msg.type = "Larson";
									break;
								case hardware_interface::candle::AnimationType::Rainbow:
									anim_msg.type = "Rainbow";
									break;
								case hardware_interface::candle::AnimationType::RGBFade:
									anim_msg.type = "RGBFade";
									break;
								case hardware_interface::candle::AnimationType::SingleFade:
									anim_msg.type = "SingleFade";
									break;
								case hardware_interface::candle::AnimationType::Strobe:
									anim_msg.type = "Strobe";
									break;
								case hardware_interface::candle::AnimationType::Twinkle:
									anim_msg.type = "Twinkle";
									break;
								case hardware_interface::candle::AnimationType::TwinkleOff:
									anim_msg.type = "TwinkleOff";
									break;
								default:
									anim_msg.type = "Unknown";
									break;
							}
							anim_msg.color.red = anim->colour_.red_;
							anim_msg.color.green = anim->colour_.green_;
							anim_msg.color.blue = anim->colour_.blue_;
							anim_msg.color.white = anim->colour_.white_;
							anim_msg.color.valid = true; // not sure this matters
							if ((anim->type_ == hardware_interface::candle::AnimationType::Twinkle) ||
								(anim->type_ == hardware_interface::candle::AnimationType::TwinkleOff))
							{
								// Twinkle/TwinkleOff Percent values are encoded in this field
								switch (anim->direction_)
								{
									case 0:
										anim_msg.direction = "Percen100";
										break;
									case 1:
										anim_msg.direction = "Percent88";
										break;
									case 2:
										anim_msg.direction = "Percent76";
										break;
									case 3:
										anim_msg.direction = "Percent64";
										break;
									case 4:
										anim_msg.direction = "Percent42";
										break;
									case 5:
										anim_msg.direction = "Percent30";
										break;
									case 6:
										anim_msg.direction = "Percent18";
										break;
									case 7:
										anim_msg.direction = "Percent6";
										break;
									default:
										anim_msg.direction = "Unknown";
										break;
								}
							}
							else
							{
								switch (anim->direction_)
								{
									case 0:
										anim_msg.direction = "Forward";
										break;
									case 1:
										anim_msg.direction = "Reverse";
										break;
									default:
										anim_msg.direction = "Unknown";
										break;
								}
							}
							anim_msg.brightness = anim->brightness_;
							anim_msg.reversed = anim->reversed_;
							anim_msg.param4 = anim->param4_;
							anim_msg.param5 = anim->param5_;
							anim_msg.valid = true;
						}
						else
						{
							anim_msg.valid = false;
						}

					}
				}
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & /*time*/) override
	{
	}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(candle_state_controller::CANdleStateController, controller_interface::ControllerBase)
