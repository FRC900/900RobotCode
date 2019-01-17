#include "cargo_intake_controller/cargo_intake_controller.h"

namespace cargo_intake_controller
{

bool CargoIntakeController::init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                 &root_nh,
                                                        ros::NodeHandle                 &controller_nh)
{
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
    hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();
	
    //read cargo intake actuator name from config file
    XmlRpc::XmlRpcValue cargo_intake_actuator_params;
    if (!controller_nh.getParam("cargo_intake_actuator_joint", cargo_intake_actuator_params))
    {
        ROS_ERROR_STREAM("Can not read cargo intake actuator name");
        return false;
    }

    //read cargo intake name from config file
    XmlRpc::XmlRpcValue cargo_intake_params;
    if (!controller_nh.getParam("cargo_intake_joint", cargo_intake_params))
    {
        ROS_ERROR_STREAM("Can not read cargo intake name");
        return false;
    }

    //initialize cargo actuator
    if (!cargo_intake_actuator_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, cargo_intake_actuator_params))
    {
        ROS_ERROR("Cannot initialize cargo intake actuator joint!");
        return false;
    }
    else
    {
        ROS_INFO("Initialized cargo intake actuator joint!");
    }

    //initialize cargo intake joint
    if (!cargo_intake_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, cargo_intake_params))
    {
        ROS_ERROR("Cannot initialize cargo intake joint!");
        return false;
    }
    else
    {
        ROS_INFO("Initialized cargo intake joint!");
    }


    cargo_intake_service_ = controller_nh.advertiseService("cargo_intake_command", &CargoIntakeController::cmdService, this);
	/*
    intake_in_ = pos_joint_iface->getHandle("clamp");

    //read intake name from config file
    XmlRpc::XmlRpcValue intake_params;
    if (!controller_nh.getParam("intake_joint", intake_params))
    {
        ROS_ERROR_STREAM("Can not read intake name");
        return false;
    }
    //initialize joint with that name
    if (!intake_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, intake_params))
    {
        ROS_ERROR("Cannot initialize intake joint!");
        return false;
    }
    else
    {
        ROS_INFO("Initialized intake joint!");
    }

    service_command_ = controller_nh.advertiseService("intake_command", &IntakeController::cmdService, this);
	*/
    return true;
}

void CargoIntakeController::starting(const ros::Time &/*time*/) {
    cargo_intake_joint_.setCommand(0.0); // set the command to the spinny part of the intake
}

void CargoIntakeController::update(const ros::Time &time, const ros::Duration &period) {
    cargo_intake_joint_.setCommand( * spin_command_.readFromRT() ); // set the command to the spinny intake bar
}

void CargoIntakeController::stopping(const ros::Time &time) {
}

bool CargoIntakeController::cmdService(cargo_intake_controller::CargoIntakeSrv::Request &req, cargo_intake_controller::CargoIntakeSrv::Response &res) {
    if(isRunning())
    {
        spin_command_.writeFromNonRT(req.power); //take the service request for a certain amount of power (-1 to 1) and write it to the command variable
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. CargoIntakeController is not running.");
        return false;
    }
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(cargo_intake_controller::CargoIntakeController, controller_interface::ControllerBase)
