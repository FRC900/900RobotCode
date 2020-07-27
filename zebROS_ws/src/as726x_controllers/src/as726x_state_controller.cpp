// Controller used to publish state info from the AS726x color sensor

#include <pluginlib/class_list_macros.h>
#include "as726x_controllers/as726x_state_controller.h"

namespace as726x_state_controller
{

bool AS726xStateController::init(hardware_interface::as726x::AS726xStateInterface *hw,
								 ros::NodeHandle                                  &root_nh,
								 ros::NodeHandle                                  &controller_nh)
{
	// get all joint names from the hardware interface
	const std::vector<std::string> &joint_names = hw->getNames();
	num_hw_joints_ = joint_names.size();
	for (size_t i = 0; i < num_hw_joints_; i++)
		ROS_DEBUG("Got joint %s", joint_names[i].c_str());

	// get publishing period
	if (!controller_nh.getParam("publish_rate", publish_rate_))
	{
		ROS_ERROR("Parameter 'publish_rate' not set");
		return false;
	}

	// realtime publisher
	realtime_pub_.reset(new
						realtime_tools::RealtimePublisher<as726x_msgs::AS726xState>(root_nh, "as726x_states", 2));

	auto &m = realtime_pub_->msg_;
	// get joints and allocate message

	for (size_t i = 0; i < num_hw_joints_; i++)
	{
		m.name.push_back(joint_names[i]);
		m.port.push_back("");
		m.address.push_back(0);
		m.ind_led_current_limit.push_back("");
		m.ind_led_enable.push_back(false);
		m.drv_led_current_limit.push_back("");
		m.drv_led_enable.push_back(false);
		m.conversion_type.push_back("");
		m.gain.push_back("");
		m.integration_time.push_back(0);
		m.temperature.push_back(0);
		m.raw_channel_data.push_back(as726x_msgs::AS726xRawChannelData());
		m.calibrated_channel_data.push_back(as726x_msgs::AS726xCalibratedChannelData());
		for (size_t j = 0; j < 6; j++)
		{
			m.raw_channel_data[i].raw_channel_data.push_back(0);
			m.calibrated_channel_data[i].calibrated_channel_data.push_back(0);
		}

		as726x_state_.push_back(hw->getHandle(joint_names[i]));
	}

	return true;
}

void AS726xStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

void AS726xStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
{
	// limit rate of publishing
	if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
	{
		// try to publish
		if (realtime_pub_->trylock())
		{
			// we're actually publishing, so increment time
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

			// populate joint state message:
			// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
			// - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
			auto &m = realtime_pub_->msg_;
			m.header.stamp = time;
			for (size_t i = 0; i < num_hw_joints_; i++)
			{
				auto &as = as726x_state_[i];
				m.port[i] = as->getPort();
				m.address[i] = as->getAddress();
				const auto ind_led_current_limit = as->getIndLedCurrentLimit();
				switch (ind_led_current_limit)
				{
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_1MA:
						m.ind_led_current_limit[i] = "1mA";
						break;
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_2MA:
						m.ind_led_current_limit[i] = "2mA";
						break;
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_4MA:
						m.ind_led_current_limit[i] = "4mA";
						break;
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_8MA:
						m.ind_led_current_limit[i] = "8mA";
						break;
					default:
						m.ind_led_current_limit[i] = "Unknown";
						break;
				}
				m.ind_led_enable[i] = as->getIndLedEnable();
				const auto drv_led_current_limit = as->getDrvLedCurrentLimit();
				switch (drv_led_current_limit)
				{
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_12MA5:
						m.drv_led_current_limit[i] = "12mA5";
						break;
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_25MA:
						m.drv_led_current_limit[i] = "25mA";
						break;
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_50MA:
						m.drv_led_current_limit[i] = "50mA";
						break;
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_100MA:
						m.drv_led_current_limit[i] = "100mA";
						break;
					default:
						m.drv_led_current_limit[i] = "Unknown";
						break;
				}
				m.drv_led_enable[i] = as->getDrvLedEnable();
				const auto conversion_type = as->getConversionType();
				switch (conversion_type)
				{
					case hardware_interface::as726x::ConversionTypes::MODE_0:
						m.conversion_type[i] = "MODE_0";
						break;
					case hardware_interface::as726x::ConversionTypes::MODE_1:
						m.conversion_type[i] = "MODE_1";
						break;
					case hardware_interface::as726x::ConversionTypes::MODE_2:
						m.conversion_type[i] = "MODE_2";
						break;
					case hardware_interface::as726x::ConversionTypes::ONE_SHOT:
						m.conversion_type[i] = "ONE_SHOT";
						break;
					default:
						m.conversion_type[i] = "Unknown";
						break;
				}
				const auto gain = as->getGain();
				switch (gain)
				{
					case hardware_interface::as726x::ChannelGain::GAIN_1X:
						m.gain[i] = "GAIN_1X";
						break;
					case hardware_interface::as726x::ChannelGain::GAIN_3X7:
						m.gain[i] = "GAIN_3X7";
						break;
					case hardware_interface::as726x::ChannelGain::GAIN_16X:
						m.gain[i] = "GAIN_16X";
						break;
					case hardware_interface::as726x::ChannelGain::GAIN_64X:
						m.gain[i] = "GAIN_64X";
						break;
					default:
						m.gain[i] = "Unknown";
						break;
				}
				m.integration_time[i] = static_cast<int>(as->getIntegrationTime());
				m.temperature[i] = as->getTemperature();
				const auto raw_channel_data = as->getRawChannelData();
				const auto calibrated_channel_data = as->getCalibratedChannelData();
				for (size_t j = 0; j < 6; j++)
				{
					m.raw_channel_data[i].raw_channel_data[j] = raw_channel_data[j];
					m.calibrated_channel_data[i].calibrated_channel_data[j] = calibrated_channel_data[j];
				}
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void AS726xStateController::stopping(const ros::Time & /*time*/)
{}

}

namespace state_listener_controller
{
AS726xStateListenerController::AS726xStateListenerController() {}
AS726xStateListenerController::~AS726xStateListenerController()
{
	sub_command_.shutdown();
}

bool AS726xStateListenerController::init(hardware_interface::as726x::RemoteAS726xStateInterface *hw, ros::NodeHandle &n)
{
	// Read list of hw, make a list, grab handles for them, plus allocate storage space
	joint_names_ = hw->getNames();
	if (joint_names_.size() == 0)
	{
		ROS_ERROR("AS726x State Listener Controller : no remote as726x joints defined");
	}
	for (auto j : joint_names_)
	{
		ROS_INFO_STREAM("AS726x State Listener Controller got joint " << j);
		handles_.push_back(hw->getHandle(j));
	}

	std::string topic;

	// get topic to subscribe to
	if (!n.getParam("topic", topic))
	{
		ROS_ERROR("Parameter 'topic' not set");
		return false;
	}

	sub_command_ = n.subscribe<as726x_msgs::AS726xState>(topic, 1, &AS726xStateListenerController::commandCB, this);
	return true;
}

void AS726xStateListenerController::starting(const ros::Time & /*time*/)
{
}
void AS726xStateListenerController::stopping(const ros::Time & /*time*/)
{
	//handles_.release();
}

void AS726xStateListenerController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
	// Take the most recent set of values read from the joint_states
	// topic and write them to the local joints
	auto vals = *command_buffer_.readFromRT();
	for (size_t i = 0; i < vals.size(); i++)
		if (vals[i].valid_)
			// Quick way to do a shallow copy of the entire HW state
			*(handles_[i].operator->()) = vals[i].value_;
}


// Iterate through each desired joint state.  If it is found in
// the message, save the value here in the realtime buffer.
void AS726xStateListenerController::commandCB(const as726x_msgs::AS726xStateConstPtr &msg)
{
	std::vector<ValueValid<hardware_interface::as726x::AS726xState>> data;
	for (size_t i = 0; i < joint_names_.size(); i++)
	{
		auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
		if (it != msg->name.cend())
		{
			const size_t loc = it - msg->name.cbegin();
			data.push_back(hardware_interface::as726x::AS726xState(msg->port[loc], msg->address[loc]));

			hardware_interface::as726x::IndLedCurrentLimits ind_led_current_limit = static_cast< hardware_interface::as726x::IndLedCurrentLimits>(0);
			if (msg->ind_led_current_limit[loc] == "1mA")
				ind_led_current_limit = hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_1MA;
			else if (msg->ind_led_current_limit[loc] == "2mA")
				ind_led_current_limit = hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_2MA;
			else if (msg->ind_led_current_limit[loc] == "4mA")
				ind_led_current_limit = hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_4MA;
			else if (msg->ind_led_current_limit[loc] == "8mA")
				ind_led_current_limit = hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_8MA;
			data[i].value_.setIndLedCurrentLimit(ind_led_current_limit);
			data[i].value_.setIndLedEnable(msg->ind_led_enable[loc]);

			hardware_interface::as726x::DrvLedCurrentLimits drv_led_current_limit = static_cast<hardware_interface::as726x::DrvLedCurrentLimits>(0);
			if (msg->drv_led_current_limit[loc] == "12mA5")
				drv_led_current_limit = hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_12MA5;
			else if (msg->drv_led_current_limit[loc] == "25mA")
				drv_led_current_limit = hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_25MA;
			else if (msg->drv_led_current_limit[loc] == "50mA")
				drv_led_current_limit = hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_50MA;
			else if (msg->drv_led_current_limit[loc] == "100mA")
				drv_led_current_limit = hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_100MA;
			data[i].value_.setDrvLedCurrentLimit(drv_led_current_limit);

			hardware_interface::as726x::ConversionTypes conversion_type = static_cast<hardware_interface::as726x::ConversionTypes>(0);
			if (msg->conversion_type[loc] == "MODE_0")
				conversion_type = hardware_interface::as726x::ConversionTypes::MODE_0;
			else if (msg->conversion_type[loc] == "MODE_1")
				conversion_type = hardware_interface::as726x::ConversionTypes::MODE_1;
			else if (msg->conversion_type[loc] == "MODE_2")
				conversion_type = hardware_interface::as726x::ConversionTypes::MODE_2;
			else if (msg->conversion_type[loc] == "ONE_SHOT")
				conversion_type = hardware_interface::as726x::ConversionTypes::ONE_SHOT;
			data[i].value_.setConversionType(conversion_type);

			hardware_interface::as726x::ChannelGain gain = static_cast<hardware_interface::as726x::ChannelGain>(0);
			if (msg->gain[loc] == "GAIN_1X")
				gain = hardware_interface::as726x::ChannelGain::GAIN_1X;
			else if (msg->gain[loc] == "GAIN_3X7")
				gain = hardware_interface::as726x::ChannelGain::GAIN_3X7;
			else if (msg->gain[loc] == "GAIN_16X")
				gain = hardware_interface::as726x::ChannelGain::GAIN_16X;
			else if (msg->gain[loc] == "GAIN_64X")
				gain = hardware_interface::as726x::ChannelGain::GAIN_64X;
			data[i].value_.setGain(gain);
			data[i].value_.setIntegrationTime(msg->integration_time[loc]);
			data[i].value_.setTemperature(msg->temperature[loc]);
			std::array<uint16_t, 6> raw_channel_data;
			std::array<float, 6> calibrated_channel_data;
			for (size_t j = 0; j < 6; j++)
			{
				raw_channel_data[j] = msg->raw_channel_data[loc].raw_channel_data[j];
				calibrated_channel_data[j] = msg->calibrated_channel_data[loc].calibrated_channel_data[j];
			}
			data[i].value_.setRawChannelData(raw_channel_data);
			data[i].value_.setCalibratedChannelData(calibrated_channel_data);
			data[i].valid_ = true;
		}
		else
		{
			data.push_back(hardware_interface::as726x::AS726xState("", 0));
		}
	}

	command_buffer_.writeFromNonRT(data);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(as726x_state_controller::AS726xStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::AS726xStateListenerController, controller_interface::ControllerBase)
