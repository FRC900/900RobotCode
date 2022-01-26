#include "frc_state_controllers/pdh_state_controller.h"
#include "frc_msgs/PDHData.h"

namespace pdh_state_controller
{

bool PDHStateController::init(hardware_interface::PDHStateInterface *hw,
							  ros::NodeHandle						&root_nh,
							  ros::NodeHandle						&controller_nh)
{
	ROS_INFO_STREAM_NAMED("pdh_state_controller", "init is running");

	std::vector<std::string> pdh_names = hw->getNames();
	if (pdh_names.size() > 1) {
		ROS_ERROR_STREAM("Cannot initialize multiple PDHs.");
		return false;
	}
	if (pdh_names.size() < 1) {
		ROS_ERROR_STREAM("Cannot initialize zero PDHs.");
		return false;
	}

	const std::string pdh_name = pdh_names[0];

	if (!controller_nh.getParam("publish_rate", publish_rate_)) {
		ROS_ERROR("Could not read publish_rate in PDH state controller");
	}

	realtime_pub_.reset(new realtime_tools::RealtimePublisher<frc_msgs::PDHData>(root_nh, "pdh_states", 2));
	pdh_state_ = hw->getHandle(pdh_name);

	auto &m = realtime_pub_->msg_;
	if (m.current.size() != m.thingsPluggedIn.size() ||
	    m.current.size() != m.channelBreakerFault.size() ||
		m.current.size() != m.stickyChannelBreakerFault.size())
	{
		ROS_ERROR_STREAM("PDH State Controller size mismatch in channel message arrays:" <<
				" m.current.size = " << m.current.size() <<
				" m.thingsPluggedIn.size = " << m.thingsPluggedIn.size() <<
				" m.channelBreakerFault.size = " << m.channelBreakerFault.size() <<
				" m.stickyChannelBreakerFault.size = " << m.stickyChannelBreakerFault.size());
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
    if(!controller_nh.getParam("things_plugged_in_pdh_channel", thingsPluggedIn)){
        ROS_ERROR("No things plugged in specified");
	}
	else
	{
		for(size_t channel = 0; channel < m.current.size(); channel++)
		{
			XmlRpc::XmlRpcValue thing_plugged_in = thingsPluggedIn[channel];
			if(!thing_plugged_in.valid() || thing_plugged_in.getType() != XmlRpc::XmlRpcValue::TypeString){
				ROS_ERROR_STREAM("An invalid thing_plugged_in name was specified for channel " << channel << " (expecting a string)");
			} else {
				std::string thing_string = thing_plugged_in;
				m.thingsPluggedIn[channel] = thing_string;
			}
		}
	}

	return true;
}

void PDHStateController::starting(const ros::Time &time)
{
	last_publish_time_ = time;
}

void PDHStateController::update(const ros::Time &time, const ros::Duration & )
{
	//ROS_INFO_STREAM("pdh pub: " << publish_rate_);
	if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
	{
		if (realtime_pub_->trylock())
		{
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

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
	}
}

void PDHStateController::stopping(const ros::Time & )
{}
} // namespace

namespace state_listener_controller
{
PDHStateListenerController::PDHStateListenerController()
	: command_buffer_(hardware_interface::PDHHWState(-1))
{
}

PDHStateListenerController::~PDHStateListenerController()
{
	sub_command_.shutdown();
}

bool PDHStateListenerController::init(hardware_interface::RemotePDHStateInterface *hw, ros::NodeHandle &n)
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

void PDHStateListenerController::starting(const ros::Time & /*time*/)
{
}
void PDHStateListenerController::stopping(const ros::Time & /*time*/)
{
}

void PDHStateListenerController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
	// Quick way to do a shallow copy of the entire HW state
	*(handle_.operator->()) = *command_buffer_.readFromRT();
}

// Iterate through each desired joint state.  If it is found in
// the message, save the value here in the realtime buffer.
void PDHStateListenerController::commandCB(const frc_msgs::PDHDataConstPtr &msg)
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

} // namespace state_listener_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pdh_state_controller::PDHStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::PDHStateListenerController, controller_interface::ControllerBase)
