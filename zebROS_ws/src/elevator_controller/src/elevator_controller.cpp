#include "elevator_controller/elevator_controller.h"

namespace elevator_controller
{
bool ElevatorController::init(hardware_interface::RobotHW *hw,
		ros::NodeHandle                 &root_nh,
		ros::NodeHandle                 &controller_nh)
{
	//create the interface used to initialize the talon joint
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

	//hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

	//get config values for the elevator talon
	XmlRpc::XmlRpcValue elevator_params;
	if (!controller_nh.getParam("elevator_joint", elevator_params))
	{ ROS_ERROR("Could not find elevator_joint");
		return false;
	}

	//initialize the elevator joint
	if(!elevator_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, elevator_params))
	{
		ROS_ERROR("Cannot initialize elevator joint!");
	}

	//read locations for elevator placement for HATCH PANEL
	hatch_locations_.resize(4); //TODO: not hard-coded
	cargo_locations_.resize(4); //TODO: not hard-coded
	double hatch_cargo_ship_position;
	if (!controller_nh.getParam("hatch/cargo_ship_position", hatch_cargo_ship_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_cargo_ship_position");
		return false;
	}
	hatch_locations_[0] = hatch_cargo_ship_position;

	double hatch_rocket1_position;
	if (!controller_nh.getParam("hatch/rocket1_position", hatch_rocket1_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_rocket1_position");
		return false;
	}
	hatch_locations_[1] = hatch_rocket1_position;

	double hatch_rocket2_position;
	if (!controller_nh.getParam("hatch/rocket3_position", hatch_rocket2_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_rocket2_position");
		return false;
	}
	hatch_locations_[2] = hatch_rocket2_position;

	double hatch_rocket3_position;
	if (!controller_nh.getParam("hatch/rocket3_position", hatch_rocket3_position))
	{
		ROS_ERROR_STREAM("Could not read hatch_rocket3_position");
		return false;
	}
	hatch_locations_[3] = hatch_rocket3_position;

	//read locations for elevator placement for HATCH PANEL
	double cargo_cargo_ship_position;
	if (!controller_nh.getParam("cargo/cargo_ship_position", cargo_cargo_ship_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_cargo_ship_position");
		return false;
	}
	cargo_locations_[0] = cargo_cargo_ship_position;

	double cargo_rocket1_position;
	if (!controller_nh.getParam("cargo/rocket1_position", cargo_rocket1_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_rocket1_position");
		return false;
	}
	cargo_locations_[1] = cargo_rocket1_position;

	double cargo_rocket2_position;
	if (!controller_nh.getParam("cargo/rocket3_position", cargo_rocket2_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_rocket2_position");
		return false;
	}
	cargo_locations_[2] = cargo_rocket2_position;

	double cargo_rocket3_position;
	if (!controller_nh.getParam("cargo/rocket3_position", cargo_rocket3_position))
	{
		ROS_ERROR_STREAM("Could not read cargo_rocket3_position");
		return false;
	}
	cargo_locations_[3] = cargo_rocket3_position;

	elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController::cmdService, this);

	place_hatch_.writeFromNonRT(false);
	place_cargo_.writeFromNonRT(false);

	return true;
}

void ElevatorController::starting(const ros::Time &/*time*/) {
}

// set the command to the spinny part of the intake
void ElevatorController::update(const ros::Time &time,const  ros::Duration &duration)
{
	int index = *(position_index_command_.readFromRT());
	if(place_hatch_.readFromRT())
	{
		if(index < hatch_locations_.size())
		{
			elevator_joint_.setCommand(hatch_locations_[index]);
		}
		else
			ROS_WARN_STREAM("invalid index in elevator_controller");
	}
	else if(place_cargo_.readFromRT())
	{
		if(index < cargo_locations_.size())
		{
			elevator_joint_.setCommand(cargo_locations_[index]);
		}
		else
			ROS_WARN_STREAM("invalid index in elevator_controller");
	}
	else
		ROS_WARN_STREAM("if statements are broken");

}

void ElevatorController::stopping(const ros::Time &time) {
}


//Command Service Function
bool ElevatorController::cmdService(elevator_controller::ElevatorSrv::Request &req, elevator_controller::ElevatorSrv::Response &response) {
	if(isRunning())
	{
		if(req.place_hatch == req.place_cargo)
		{
			ROS_WARN_STREAM("skipping command from elevator server because we can only place one game piece at a time :(");
			return false;
		}
		position_index_command_.writeFromNonRT(req.position_index); //take the service request for a certain amount of power (-1 to 1) and write it to the command variable
		place_hatch_.writeFromNonRT(req.place_hatch);
		place_cargo_.writeFromNonRT(req.place_cargo);
		ROS_INFO_STREAM("req.position = " << req.position_index);
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. ElevatorController is not running.");
		return false;
	}
	return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase)
