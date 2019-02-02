#include "panel_intake_controller/panel_intake_controller.h"

namespace panel_intake_controller
{

	bool PanelIntakeController::init(hardware_interface::PositionJointInterface *hw,
			ros::NodeHandle                 &root_nh,
			ros::NodeHandle                 &controller_nh)
	{
		claw_joint_ = hw->getHandle("panel_claw_release");
		push_joint_ = hw->getHandle("panel_push_extend");

		panel_intake_service_ = controller_nh.advertiseService("panel_command", &PanelIntakeController::cmdService, this);
		joint_states_sub_ = controller_nh.subscribe("/frcrobot_jetson/joint_states", 1, &PanelIntakeController::jointStateCallback, this);

		return true;
	}

	void PanelIntakeController::starting(const ros::Time &/*time*/) {
		// TODO : defaults?
	}

	void PanelIntakeController::update(const ros::Time &time, const ros::Duration &period) {
		const bool claw_cmd = *(claw_cmd_.readFromRT());
		if(claw_cmd == true) {
			//ROS_WARN("intake in");
			claw_joint_.setCommand(1.0);
		}
		else if (claw_cmd == false) {

			claw_joint_.setCommand(0.0);
		}

		const bool push_cmd = *(push_cmd_.readFromRT());
		if(push_cmd == true) {
			//ROS_WARN("intake in");
			push_joint_.setCommand(1.0);
		}
		else if (push_cmd == false) {
			push_joint_.setCommand(0.0);
		}
	}

	void PanelIntakeController::stopping(const ros::Time &time) {
	}

	bool PanelIntakeController::cmdService(panel_intake_controller::PanelIntakeSrv::Request &req, panel_intake_controller::PanelIntakeSrv::Response &response) {
		if(isRunning())
		{
			claw_cmd_.writeFromNonRT(req.claw_release); //take the service request for in/out (true/false???) and write to a command variable
			push_cmd_.writeFromNonRT(req.push_extend);
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
		static size_t linebreak_idx = std::numeric_limits<size_t>::max();

		if ((linebreak_idx >= joint_state.name.size()))
		{
			for (size_t i = 0; i < joint_state.name.size(); i++)
			{
				if (joint_state.name[i] == "cargo_intake_linebreak_1")
					linebreak_idx = i;
			}
		}

		//update linebreak counts based on the value of the linebreak sensor
		if (linebreak_idx < joint_state.position.size())
		{
			bool linebreak_true = (joint_state.position[linebreak_idx] != 0);
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
			static int count = 0;
			if(count % 100 == 0)
			{
				ROS_WARN("intake line break sensor not found in joint_states");
			}
			count++;
			linebreak_true_count = 0;
			linebreak_false_count += 1;
		}
	}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(panel_intake_controller::PanelIntakeController, controller_interface::ControllerBase)
