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
		joint_states_sub_ = controller_nh.subscribe("/frcrobot_jetson/joint_states", 1, &PanelIntakeController::jointStateCallback, this);

		return true;
	}

	void PanelIntakeController::starting(const ros::Time &/*time*/) {
		last_claw_cmd_ = true; //claw not released, mech not extended

		panel_cmd_.writeFromNonRT(PanelCommand(false, false));
	}

	void PanelIntakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		const PanelCommand panel_cmd = *(panel_cmd_.readFromRT());

		//if(claw_cmd == true) {
		//	//ROS_WARN("intake in");
		//	claw_joint_.setCommand(1.0);
		//}
		//else if (claw_cmd == false) {

		//	claw_joint_.setCommand(0.0);
		//}
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
		//last_claw_cmd_ = panel_cmd.claw_cmd;
	}

	void PanelIntakeController::stopping(const ros::Time &/*time*/) {
	}

	bool PanelIntakeController::cmdService(panel_intake_controller::PanelIntakeSrv::Request &req, panel_intake_controller::PanelIntakeSrv::Response &/*response*/) {
		if(isRunning())
		{
			panel_cmd_.writeFromNonRT(PanelCommand(req.claw_release, req.push_extend));
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. PanelIntakeController is not running.");
			return false;
		}
		return true;
	}

	void PanelIntakeController::jointStateCallback(const sensor_msgs::JointState &joint_state)
	{
		//get index of linebreak sensor for this actionlib server
		static size_t linebreak_idx_1 = std::numeric_limits<size_t>::max();
		static size_t linebreak_idx_2 = std::numeric_limits<size_t>::max();
		static size_t linebreak_idx_3 = std::numeric_limits<size_t>::max();
		static size_t linebreak_idx_4 = std::numeric_limits<size_t>::max();

		if (linebreak_idx_1 >= joint_state.name.size()
			|| linebreak_idx_2 >= joint_state.name.size()
			|| linebreak_idx_3 >= joint_state.name.size()
			|| linebreak_idx_4 >= joint_state.name.size() )
		{
			for (size_t i = 0; i < joint_state.name.size(); i++)
			{
				if (joint_state.name[i] == "panel_intake_linebreak_1")
					linebreak_idx_1 = i;
				if (joint_state.name[i] == "panel_intake_linebreak_2")
					linebreak_idx_2 = i;
				if (joint_state.name[i] == "panel_intake_linebreak_3")
					linebreak_idx_3 = i;
				if (joint_state.name[i] == "panel_intake_linebreak_4")
					linebreak_idx_4 = i;
			}
		}

		//update linebreak counts based on the value of the linebreak sensor
		if (linebreak_idx_1 < joint_state.position.size()
			&& linebreak_idx_2 < joint_state.position.size()
			&& linebreak_idx_3 < joint_state.position.size()
			&& linebreak_idx_4 < joint_state.position.size() )
		{
			bool linebreak_true = ( joint_state.position[linebreak_idx_1] != 0
									|| joint_state.position[linebreak_idx_2] != 0
									|| joint_state.position[linebreak_idx_3] != 0
									|| joint_state.position[linebreak_idx_4] != 0
								);
			if(linebreak_true)
			{
				linebreak_true_count += 1;
				linebreak_false_count = 0;
			}
			else
			{
				linebreak_true_count = 0;
				linebreak_false_count += 1;
			}
		}
		else
		{
			ROS_WARN_THROTTLE(1.0, "intake line break sensor not found in joint_states");
			linebreak_true_count = 0;
			linebreak_false_count += 1;
		}
	}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(panel_intake_controller::PanelIntakeController, controller_interface::ControllerBase)
