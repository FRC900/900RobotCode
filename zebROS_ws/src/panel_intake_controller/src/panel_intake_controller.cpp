#include "panel_intake_controller/panel_intake_controller.h"

namespace panel_intake_controller
{
	bool PanelIntakeController::init(hardware_interface::PositionJointInterface *hw,
									 ros::NodeHandle                 &/*root_nh*/,
									 ros::NodeHandle                 &controller_nh)
	{
		claw_joint_ = hw->getHandle("panel_claw");
		push_joint_ = hw->getHandle("panel_push_extend");

		panel_intake_service_ = controller_nh.advertiseService("panel_command", &PanelIntakeController::cmdService, this);

		return true;
	}

	void PanelIntakeController::starting(const ros::Time &/*time*/) {
		panel_cmd_.writeFromNonRT(PanelCommand(true, false));
	}

	void PanelIntakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		const PanelCommand panel_cmd = *(panel_cmd_.readFromRT());

        if(panel_cmd.claw_cmd_) {
            claw_joint_.setCommand(-1.0);
        }
        else {
            claw_joint_.setCommand(1.0);
        }

		if(panel_cmd.push_cmd_) {
			//ROS_WARN("intake in");
			push_joint_.setCommand(1.0);
		}
		else {
			push_joint_.setCommand(0.0);
		}
	}

	void PanelIntakeController::stopping(const ros::Time &/*time*/) {
	}

	bool PanelIntakeController::cmdService(panel_intake_controller::PanelIntakeSrv::Request &req, panel_intake_controller::PanelIntakeSrv::Response &/*response*/) {
		if(isRunning())
		{
			panel_cmd_.writeFromNonRT(PanelCommand(!req.claw_release, req.push_extend));
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. PanelIntakeController is not running.");
			return false;
		}
		return true;
	}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(panel_intake_controller::PanelIntakeController, controller_interface::ControllerBase)
