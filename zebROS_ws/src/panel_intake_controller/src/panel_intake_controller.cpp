#include "panel_intake_controller/panel_intake_controller.h"

namespace panel_intake_controller
{

bool PanelIntakeController::init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                 &root_nh,
                                                        ros::NodeHandle                 &controller_nh)
{
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
    hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();


	
   // intake_in_ = pos_joint_iface->getHandle("clamp");
    //read intake name from config file
    XmlRpc::XmlRpcValue panel_params;
    if (!controller_nh.getParam("panel_joint", panel_params))
    {
        ROS_ERROR_STREAM("Can not read panel name");
        return false;
    }
	/*
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
	*/

    panel_service_ = controller_nh.advertiseService("intake_command", &PanelIntakeController::cmdService, this);

    return true;
}

void PanelIntakeController::starting(const ros::Time &/*time*/) {
}

void PanelIntakeController::update(const ros::Time &time, const ros::Duration &period) {
    /*double spin_command = *(spin_command_.readFromRT());
    bool intake_in_cmd = *(intake_in_cmd_.readFromRT());
    double intake_in_cmd_double;
    if(intake_in_cmd == true) {
        //ROS_WARN("intake in");
        intake_in_cmd_double = -1;
    }
    else if (intake_in_cmd == false) {
        intake_in_cmd_double = 1;
        //ROS_WARN("intake out");
    }
    intake_joint_.setCommand(spin_command); // set the command to the spinny part of the intake
    intake_in_.setCommand(intake_in_cmd_double); // set the in/out command to the clampy part of the intake
	*/
}

void PanelIntakeController::stopping(const ros::Time &time) {
}
/*
bool PanelIntakeController::cmdService(panel_intake_controller::PanelIntakeSrv::Request &req, panel_intake_controller::PanelIntakeSrv::Response &response) {
    if(isRunning())
    {
        spin_command_.writeFromNonRT(req.power); //take the service request for a certain amount of power (-1 to 1) and write it to the command variable
        intake_in_cmd_.writeFromNonRT(req.intake_in); //take the service request for in/out (true/false???) and write to a command variable
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. PanelIntakeController is not running.");
        return false;
    }
    return true;
}
*/
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(panel_intake_controller::PanelIntakeController, controller_interface::ControllerBase)
