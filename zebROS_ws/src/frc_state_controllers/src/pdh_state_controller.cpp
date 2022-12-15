#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_msgs/PDHData.h>
#include <frc_interfaces/pdh_state_interface.h>
#include "frc_msgs/PDHData.h"
#include "periodic_interval_counter/periodic_interval_counter.h"

namespace pdh_state_controller
{
class PDHStateController: public controller_interface::Controller<hardware_interface::PDHStateInterface>
{
private:
	hardware_interface::PDHStateHandle pdh_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::PDHData>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{20};

public:	
	bool init(hardware_interface::PDHStateInterface *hw,
			  ros::NodeHandle                       &root_nh,
			  ros::NodeHandle                       &controller_nh)
	{
		ROS_INFO_STREAM_NAMED("pdh_state_controller", "init is running");

		std::vector<std::string> pdh_names = hw->getNames();
		if (pdh_names.size() > 1)
		{
			ROS_ERROR_STREAM("Cannot initialize multiple PDHs.");
			return false;
		}
		if (pdh_names.size() < 1)
		{
			ROS_ERROR_STREAM("Cannot initialize zero PDHs.");
			return false;
		}

		const std::string pdh_name = pdh_names[0];

		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN_STREAM("Could not read publish_rate in PDH state controller, using default " << publish_rate_);
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in PDH state controller (" << publish_rate_ << ")");
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::PDHData>>(root_nh, "pdh_states", 2);
		pdh_state_ = hw->getHandle(pdh_name);

		auto &m = realtime_pub_->msg_;
		if (m.current.size() != m.thingsPluggedIn.size() ||
			m.current.size() != m.channelBreakerFault.size() ||
			m.current.size() != m.stickyChannelBreakerFault.size())
		{
			ROS_ERROR_STREAM("PDH State Controller size mismatch in channel message arrays:"
							 << " m.current.size = " << m.current.size() << " m.thingsPluggedIn.size = " << m.thingsPluggedIn.size() << " m.channelBreakerFault.size = " << m.channelBreakerFault.size() << " m.stickyChannelBreakerFault.size = " << m.stickyChannelBreakerFault.size());
			return false;
		}

		m.enabled = false;
		m.voltage = 0;
		m.canWarning = false;
		m.stickyCanWarning = false;
		m.stickyCanBusOff = false;
		m.hardwareFault = false;
		m.stickyHasReset = false;
		m.moduleNumber = 0;
		m.firmwareMajor = 0;
		m.firmwareMinor = 0;
		m.firmwareFix = 0;
		m.hardwareMajor = 0;
		m.hardwareMinor = 0;
		m.switchableChannelState = false;
		m.totalCurrent = 0;

		for (size_t i = 0; i < m.current.size(); i++)
		{
			m.current[i] = 0;
			m.channelBreakerFault[i] = false;
			m.stickyChannelBreakerFault[i] = false;
		}

		XmlRpc::XmlRpcValue thingsPluggedIn;
		if (!controller_nh.getParam("things_plugged_in_pdh_channel", thingsPluggedIn))
		{
			ROS_ERROR("No things plugged in specified");
		}
		else
		{
			for (size_t channel = 0; channel < m.current.size(); channel++)
			{
				XmlRpc::XmlRpcValue thing_plugged_in = thingsPluggedIn[channel];
				if (!thing_plugged_in.valid() || thing_plugged_in.getType() != XmlRpc::XmlRpcValue::TypeString)
				{
					ROS_ERROR_STREAM("An invalid thing_plugged_in name was specified for channel " << channel << " (expecting a string)");
				}
				else
				{
					std::string thing_string = thing_plugged_in;
					m.thingsPluggedIn[channel] = thing_string;
				}
			}
		}

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
				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				auto &ps = pdh_state_;

				m.enabled = ps->getEnabled();
				m.voltage = ps->getVoltage();
				m.canWarning = ps->getCANWarning();
				m.stickyCanWarning = ps->getStickyCANWarning();
				m.stickyCanBusOff = ps->getStickyCANBusOff();
				m.hardwareFault = ps->getHardwareFault();
				m.stickyHasReset = ps->getStickyHasReset();
				m.moduleNumber = ps->getModuleNumber();
				m.firmwareMajor = ps->getFirmwareMajor();
				m.firmwareMinor = ps->getFirmwareMinor();
				m.firmwareFix = ps->getFirmwareiFix();
				m.hardwareMajor = ps->getHardwareMajor();
				m.hardwareMinor = ps->getHardwareMinor();
				m.switchableChannelState = ps->getSwitchableChannelState();
				m.totalCurrent = ps->getTotalCurrent();

				for (size_t i = 0; i < m.current.size(); i++)
				{
					m.current[i] = ps->getChannelCurrent(i);
					m.channelBreakerFault[i] = ps->getChannelBreakerFault(i);
					m.stickyChannelBreakerFault[i] = ps->getStickyChannelBreakerFault(i);
				}

				realtime_pub_->unlockAndPublish();
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

namespace state_listener_controller
{
class PDHStateListenerController :
	public controller_interface::Controller<hardware_interface::RemotePDHStateInterface>
{
private:
	ros::Subscriber sub_command_;
	hardware_interface::PDHWritableStateHandle handle_;

	// Real-time buffer holds the last command value read from the "command" topic.
	realtime_tools::RealtimeBuffer<hardware_interface::PDHHWState> command_buffer_;

	// Iterate through each desired joint state.  If it is found in
	// the message, save the value here in the realtime buffer.
	virtual void commandCB(const frc_msgs::PDHDataConstPtr &msg)
	{
		hardware_interface::PDHHWState data(msg->moduleNumber);

		data.setEnabled(msg->enabled);
		data.setVoltage(msg->voltage);
		data.setCANWarning(msg->canWarning);
		data.setStickyCANWarning(msg->stickyCanWarning);
		data.setStickyCANBusOff(msg->stickyCanBusOff);
		data.setHardwareFault(msg->hardwareFault);
		data.setStickyHasReset(msg->stickyHasReset);
		data.setFirmwareMajor(msg->firmwareMajor);
		data.setFirmwareMinor(msg->firmwareMinor);
		data.setFirmwareFix(msg->firmwareFix);
		data.setHardwareMajor(msg->hardwareMajor);
		data.setHardwareMinor(msg->hardwareMinor);
		data.setSwitchableChannelState(msg->switchableChannelState);
		data.setTotalCurrent(msg->totalCurrent);

		for (size_t i = 0; i < msg->current.size(); i++)
		{
			data.setChannelCurrent(msg->current[i], i);
			data.setChannelBreakerFault(msg->channelBreakerFault[i], i);
			data.setStickyChannelBreakerFault(msg->stickyChannelBreakerFault[i], i);
		}
		command_buffer_.writeFromNonRT(data);
	}


public:
	PDHStateListenerController()
		: command_buffer_(hardware_interface::PDHHWState(-1))
	{
	}

	~PDHStateListenerController()
	{
		sub_command_.shutdown();
	}

	bool init(hardware_interface::RemotePDHStateInterface *hw, ros::NodeHandle &n)
	{
		// Read list of hw, make a list, grab handles for them, plus allocate storage space
		auto joint_names = hw->getNames();
		if (joint_names.size() == 0)
		{
			ROS_ERROR("PDH State Listener Controller : no remote pdh joints defined");
		}
		ROS_INFO_STREAM("PDH State Listener Controller got joint " << joint_names[0]);
		handle_ = hw->getHandle(joint_names[0]);

		std::string topic;

		// get topic to subscribe to
		if (!n.getParam("topic", topic))
		{
			ROS_ERROR("PDH State Listener Controller parameter 'topic' not set");
			return false;
		}

		sub_command_ = n.subscribe<frc_msgs::PDHData>(topic, 1, &PDHStateListenerController::commandCB, this);
		return true;
	}

	void starting(const ros::Time & /*time*/)
	{
	}
	void stopping(const ros::Time & /*time*/)
	{
	}

	void update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
	{
		// Quick way to do a shallow copy of the entire HW state
		*(handle_.operator->()) = *command_buffer_.readFromRT();
	}
}; // class

} // namespace state_listener_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pdh_state_controller::PDHStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::PDHStateListenerController, controller_interface::ControllerBase)
