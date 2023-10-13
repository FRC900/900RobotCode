#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>

#include <frc_interfaces/rumble_command_interface.h>
#include <frc_msgs/RumbleCommand.h>

namespace rumble_controller
{
class RumbleController: public controller_interface::Controller<hardware_interface::RumbleCommandInterface>
{
private: 
	hardware_interface::RumbleCommandHandle rumble_command_;

	class RumbleCommand
	{
		public:
			RumbleCommand() {}
			RumbleCommand(unsigned int left, unsigned int right)
			: left_ {left}
			, right_{right}
			{
			}

			unsigned int left_{0};
			unsigned int right_{0};
	};
	realtime_tools::RealtimeBuffer<RumbleCommand> command_buffer_;
	ros::ServiceServer service_;
	bool command(frc_msgs::RumbleCommand::Request &req, frc_msgs::RumbleCommand::Response &/*res*/)
	{
		command_buffer_.writeFromNonRT(RumbleCommand(req.left, req.right));
		return true;
	}

public:
	bool init(hardware_interface::RumbleCommandInterface *hw,
			  ros::NodeHandle & /*root_nh*/,
			  ros::NodeHandle &controller_nh) override
	{
		ROS_INFO_STREAM_NAMED("rumble_controller", "init is running");

		std::string joint_name;
		if (!controller_nh.getParam("joint", joint_name))
		{
			ROS_ERROR("Rumble controller : no joint name given");
			return false;
		}
		rumble_command_ = hw->getHandle(joint_name);

		service_ = controller_nh.advertiseService("command", &RumbleController::command, this);

		return true;
	}

	void starting(const ros::Time & /*time*/) override
	{
	}

	void update(const ros::Time &, const ros::Duration &) override
	{
		RumbleCommand command_data{*command_buffer_.readFromRT()};

		rumble_command_->setLeft(command_data.left_);
		rumble_command_->setRight(command_data.right_);
	}

	void stopping(const ros::Time &) override
	{
	}
};
} // namespace

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rumble_controller::RumbleController, controller_interface::ControllerBase)
