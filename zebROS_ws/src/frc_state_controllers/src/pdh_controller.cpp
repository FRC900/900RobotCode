#include <atomic>
#include <controller_interface/controller.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <frc_interfaces/pdh_command_interface.h>

namespace pdh_controller
{
class PDHController: public controller_interface::Controller<hardware_interface::PDHCommandInterface>
{
private:
	hardware_interface::PDHCommandHandle pdh_command_;
	std::atomic<bool> command_switchable_channel_enable_{false};
	std::atomic<bool> trigger_clear_faults_{false};

	ros::ServiceServer switchable_channel_enable_service_;
	ros::ServiceServer clear_faults_service_;

	bool switchableChannelEnableService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
	{
		command_switchable_channel_enable_ = req.data;
		res.success = true;
		return true;
	}

	bool clearFaultsService(std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/)
	{
		trigger_clear_faults_ = true;
		return true;
	}

public:
	bool init(hardware_interface::PDHCommandInterface *hw,
			  ros::NodeHandle & /*root_nh*/,
			  ros::NodeHandle &controller_nh) override
	{
		ROS_INFO_STREAM_NAMED("pdh_controller", "init is running");

		std::vector<std::string> pdh_names = hw->getNames();
		if (pdh_names.size() > 1)
		{
			ROS_ERROR_STREAM("Cannot initialize multiple PDHs in PDHController.");
			return false;
		}
		if (pdh_names.size() < 1)
		{
			ROS_ERROR_STREAM("Cannot initialize zero PDHs in PDHController");
			return false;
		}

		const std::string pdh_name = pdh_names[0];
		pdh_command_ = hw->getHandle(pdh_name);

		switchable_channel_enable_service_ = controller_nh.advertiseService("switchable_channel_enable", &PDHController::switchableChannelEnableService, this);
		clear_faults_service_ = controller_nh.advertiseService("clear_faults", &PDHController::clearFaultsService, this);

		return true;
	}

	void starting(const ros::Time & /*time*/) override
	{
	}

	void update(const ros::Time &, const ros::Duration &) override
	{
		pdh_command_->setSwitchableChannelEnable(command_switchable_channel_enable_);
		// Do an atomic test and clear of the one-shot commands
		// That is, in 1 atomic step, read the value and set it
		// to false. If the read value was true, trigger the
		// one shot action in the command interface
		if (trigger_clear_faults_.exchange(false))
		{
			pdh_command_->setClearStickyFaults();
		}
	}

	void stopping(const ros::Time &) override
	{
	}

}; // class PDHController

} // namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pdh_controller::PDHController, controller_interface::ControllerBase)
