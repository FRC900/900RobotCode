#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/pdp_state_interface.h>
#include "frc_msgs/PDPData.h"
#include "periodic_interval_counter/periodic_interval_counter.h"

namespace pdp_state_controller
{
class PDPStateController: public controller_interface::Controller<hardware_interface::PDPStateInterface>
{

private:
	hardware_interface::PDPStateHandle pdp_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::PDPData>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{20};

public:
	bool init(hardware_interface::PDPStateInterface *hw,
			  ros::NodeHandle						&root_nh,
			  ros::NodeHandle						&controller_nh) override
	{
		ROS_INFO_STREAM_NAMED("pdp_state_controller", "init is running");

		std::vector<std::string> pdp_names = hw->getNames();
		if (pdp_names.size() > 1) {
			ROS_ERROR_STREAM("Cannot initialize multiple PDPs.");
			return false; }
		else if (pdp_names.size() < 1) {
			ROS_ERROR_STREAM("Cannot initialize zero PDPs.");
			return false; }

		const std::string pdp_name = pdp_names[0];

		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN("Could not read publish_rate in PDP state controller");
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in PDP state controller (" << publish_rate_ << ")");
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::PDPData>>(root_nh, "pdp_states", 4);

		auto &m = realtime_pub_->msg_;

		m.voltage = 0.0;
		m.temperature = 0.0;
		m.totalCurrent = 0.0;
		m.totalPower = 0.0;
		m.totalEnergy = 0.0;

		XmlRpc::XmlRpcValue thingsPluggedIn;
		if(!controller_nh.getParam("things_plugged_in_pdp_channel", thingsPluggedIn)){
			ROS_ERROR("No things plugged in specified");
		}

		for(size_t channel = 0; channel < m.current.size(); channel++)
		{
			m.current[channel] = 0;
			XmlRpc::XmlRpcValue thing_plugged_in = thingsPluggedIn[channel];
			if(!thing_plugged_in.valid() || thing_plugged_in.getType() != XmlRpc::XmlRpcValue::TypeString)
				ROS_ERROR("An invalid thing_plugged_in name was specified (expecting a string)");
			else {
				std::string thing_string = thing_plugged_in;
				m.thingsPluggedIn[channel] = thing_string;
			}
		}

		pdp_state_ = hw->getHandle(pdp_name);

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
				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				const auto &ps = pdp_state_;

				//read from the object and stuff it in a msg
				m.voltage = ps->getVoltage();
				m.temperature = ps->getTemperature();
				m.totalCurrent = ps->getTotalCurrent();
				m.totalPower = ps->getTotalPower();
				m.totalEnergy = ps->getTotalEnergy();

				for(size_t channel = 0; channel < m.current.size(); channel++)
				{
					m.current[channel] = ps->getCurrent(channel);
				}

				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & ) override
	{}
}; // class
} // namespace

namespace state_listener_controller
{
class PDPStateListenerController :
	public controller_interface::Controller<hardware_interface::RemotePDPStateInterface>
{
private:
	ros::Subscriber sub_command_;
	hardware_interface::PDPWritableStateHandle handle_;

	// Real-time buffer holds the last command value read from the "command" topic.
	realtime_tools::RealtimeBuffer<hardware_interface::PDPHWState> command_buffer_;

	// Iterate through each desired joint state.  If it is found in
	// the message, save the value here in the realtime buffer.
	virtual void commandCB(const frc_msgs::PDPDataConstPtr &msg)
	{
		hardware_interface::PDPHWState data;

		data.setVoltage(msg->voltage);
		data.setTemperature(msg->temperature);
		data.setTotalCurrent(msg->totalCurrent);
		data.setTotalPower(msg->totalPower);
		data.setTotalEnergy(msg->totalEnergy);
		for (size_t channel = 0; channel < msg->current.size() ; channel++)
		{
			data.setCurrent(msg->current[channel], channel);
		}
		command_buffer_.writeFromNonRT(data);
	}
public:
	bool init(hardware_interface::RemotePDPStateInterface *hw, ros::NodeHandle &n) override
	{

		// Read list of hw, make a list, grab handles for them, plus allocate storage space
		auto joint_names = hw->getNames();
		if (joint_names.empty())
		{
			ROS_ERROR("PDP State Listener Controller : no remote pdp joints defined");
		}
		ROS_INFO_STREAM("PDP State Listener Controller got joint " << joint_names[0]);
		handle_ = hw->getHandle(joint_names[0]);

		std::string topic;

		// get topic to subscribe to
		if (!n.getParam("topic", topic))
		{
			ROS_ERROR("PDP State Listener Controller parameter 'topic' not set");
			return false;
		}

		sub_command_ = n.subscribe<frc_msgs::PDPData>(topic, 1, &PDPStateListenerController::commandCB, this, ros::TransportHints().tcpNoDelay());
		return true;
	}

	void starting(const ros::Time & /*time*/) override
	{
	}
	void stopping(const ros::Time & /*time*/) override
	{
	}

	void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
	{
		// Quick way to do a shallow copy of the entire HW state
		*(handle_.operator->()) = *command_buffer_.readFromRT();
	}
}; // class

} // namespace state_listener_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pdp_state_controller::PDPStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::PDPStateListenerController, controller_interface::ControllerBase)
