#include "panel_intake_controller/panel_intake_controller.h"

namespace panel_intake_controller
{

bool PanelIntakeController::init(hardware_interface::PositionJointInterface *hw,
                                                        ros::NodeHandle                 &root_nh,
                                                        ros::NodeHandle                 &controller_nh)
{

	claw_in_ = hw->getHandle("panel_claw_in");
	push_in_ = hw->getHandle("panel_push_in"); 
	wedge_in_ = hw->getHandle("panel_wedge_in");

   panel_service_ = controller_nh.advertiseService("panel_command", &PanelIntakeController::cmdService, this);

    return true;
}

void PanelIntakeController::starting(const ros::Time &/*time*/) {
}

void PanelIntakeController::update(const ros::Time &time, const ros::Duration &period) {
    /*double spin_command = *(spin_command_.readFromRT()); */
    bool claw_cmd = *(claw_cmd_.readFromRT());
    if(claw_cmd == true) {
        //ROS_WARN("intake in");
        claw_joint_.setCommand(-1.0);
    }
    else if (claw_cmd == false) {

        claw_joint_.setCommand(1.0);
    }

 bool push_cmd = *(push_cmd_.readFromRT());
    if(push_cmd == true) {
        //ROS_WARN("intake in");
        push_joint_.setCommand(-1.0);
    }
    else if (push_cmd == false) {

        push_joint_.setCommand(1.0);
	}

 bool wedge_cmd = *(wedge_cmd_.readFromRT());
    if(wedge_cmd == true) {
        //ROS_WARN("intake in");
        wedge_joint_.setCommand(-1.0);
    }
    else if (wedge_cmd == false) {

        wedge_joint_.setCommand(1.0);
	}

}

void PanelIntakeController::stopping(const ros::Time &time) {
}

bool PanelIntakeController::cmdService(panel_intake_controller::PanelIntakeSrv::Request &req, panel_intake_controller::PanelIntakeSrv::Response &response) {
    if(isRunning())
    {
        claw_cmd_.writeFromNonRT(req.claw_in); //take the service request for in/out (true/false???) and write to a command variable
		push_cmd_.writeFromNonRT(req.push_in);
		wedge_cmd_.writeFromNonRT(req.wedge_in);
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
