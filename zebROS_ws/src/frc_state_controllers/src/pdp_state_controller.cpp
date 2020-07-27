#include "frc_state_controllers/pdp_state_controller.h"
#include "frc_msgs/PDPData.h"

namespace pdp_state_controller
{

bool PDPStateController::init(hardware_interface::PDPStateInterface *hw,
								ros::NodeHandle						&root_nh,
								ros::NodeHandle						&controller_nh)
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
                ROS_ERROR("Could not read publish_rate in PDP state controller");

	realtime_pub_.reset(new realtime_tools::RealtimePublisher<frc_msgs::PDPData>(root_nh, "pdp_states", 4));

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

	for(int channel = 0; channel <= 15; channel++)
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

void PDPStateController::starting(const ros::Time &time)
{
	last_publish_time_ = time;
}

void PDPStateController::update(const ros::Time &time, const ros::Duration & )
{
	//ROS_INFO_STREAM("pdp pub: " << publish_rate_);
	if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
	{
		if (realtime_pub_->trylock())
		{
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

			auto &m = realtime_pub_->msg_;

			m.header.stamp = time;

			auto &ps = pdp_state_;

			//read from the object and stuff it in a msg
			m.voltage = ps->getVoltage();
			m.temperature = ps->getTemperature();
			m.totalCurrent = ps->getTotalCurrent();
			m.totalPower = ps->getTotalPower();
			m.totalEnergy = ps->getTotalEnergy();

			for(int channel = 0; channel <= 15; channel++)
			{
				m.current[channel] = ps->getCurrent(channel);
			}

			realtime_pub_->unlockAndPublish();
		}
	}
}

void PDPStateController::stopping(const ros::Time & )
{}
} // namespace

namespace state_listener_controller
{
PDPStateListenerController::PDPStateListenerController()
{
}

PDPStateListenerController::~PDPStateListenerController()
{
	sub_command_.shutdown();
}

bool PDPStateListenerController::init(hardware_interface::RemotePDPStateInterface *hw, ros::NodeHandle &n)
{
	// Read list of hw, make a list, grab handles for them, plus allocate storage space
	auto joint_names = hw->getNames();
	if (joint_names.size() == 0)
	{
		ROS_ERROR("PDP State Listener Controller : no remote pdp joints defined");
	}
	ROS_INFO_STREAM("PDP State Listener Controller got joint " << joint_names[0]);
	handle_ = hw->getHandle(joint_names[0]);

	std::string topic;

	// get topic to subscribe to
	if (!n.getParam("topic", topic))
	{
		ROS_ERROR("Parameter 'topic' not set");
		return false;
	}

	sub_command_ = n.subscribe<frc_msgs::PDPData>(topic, 1, &PDPStateListenerController::commandCB, this);
	return true;
}

void PDPStateListenerController::starting(const ros::Time & /*time*/)
{
}
void PDPStateListenerController::stopping(const ros::Time & /*time*/)
{
}

void PDPStateListenerController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
	// Quick way to do a shallow copy of the entire HW state
	*(handle_.operator->()) = *command_buffer_.readFromRT();
}

// Iterate through each desired joint state.  If it is found in
// the message, save the value here in the realtime buffer.
void PDPStateListenerController::commandCB(const frc_msgs::PDPDataConstPtr &msg)
{
	hardware_interface::PDPHWState data;

	data.setVoltage(msg->voltage);
	data.setTemperature(msg->temperature);
	data.setTotalCurrent(msg->totalCurrent);
	data.setTotalPower(msg->totalPower);
	data.setTotalEnergy(msg->totalEnergy);
	for (size_t channel = 0; channel <= 15; channel++)
		data.setCurrent(msg->current[channel], channel);
	command_buffer_.writeFromNonRT(data);
}



} // namespace state_listener_controller

PLUGINLIB_EXPORT_CLASS(pdp_state_controller::PDPStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::PDPStateListenerController, controller_interface::ControllerBase)
