#include "controllers_2020/indexer_controller.h"

namespace indexer_controller
{
bool IndexerController::init(hardware_interface::RobotHW *hw,
							 ros::NodeHandle             &/*root_nh*/,
							 ros::NodeHandle             &controller_nh)
{
	//get interface
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

	//Initialize motor joints
	//get params from config file
	XmlRpc::XmlRpcValue indexer_params;
	if ( !controller_nh.getParam("indexer_joint", indexer_params)) //grabbing the config value under the controller's section in the main config file
	{
		ROS_ERROR_STREAM("Could not read indexer params");
		return false;
	}

	//initialize motor joint using those config values
	if ( !indexer_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, indexer_params))
	{
		ROS_ERROR("Cannot initialize indexer joint!");
		return false;
	}

	//Initialize your ROS server
	indexer_service_ = controller_nh.advertiseService("indexer_command", &IndexerController::cmdService, this);

	return true;
}

void IndexerController::starting(const ros::Time &/*time*/)
{
	//give command buffer(s) an initial value
	indexer_cmd_.writeFromNonRT(IndexerCommand(0));
}

void IndexerController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
	//grab value from command buffer(s)
	const IndexerCommand indexer_cmd = *(indexer_cmd_.readFromRT());
	const double velocity_cmd = indexer_cmd.indexer_velocity_;
	indexer_joint_.setCommand(velocity_cmd);
}

void IndexerController::stopping(const ros::Time &/*time*/)
{
}
bool IndexerController::cmdService(controllers_2020_msgs::IndexerSrv::Request &req, controllers_2020_msgs::IndexerSrv::Response &/*response*/)
{
	if (isRunning())
	{
		//assign request value to command buffer(s)
		indexer_cmd_.writeFromNonRT(IndexerCommand(req.indexer_velocity));
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. IndexerController is not running.");
		return false;
	}
	return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(indexer_controller::IndexerController, controller_interface::ControllerBase)

