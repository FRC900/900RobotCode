#include "controllers_2020/indexer_test_controller.h"

namespace indexer_test_controller
{

bool IndexerTestController::init(hardware_interface::RobotHW *hw,
                                 ros::NodeHandle             &/*root_nh*/,
                                 ros::NodeHandle             &controller_nh)
{
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
    hardware_interface::JointStateInterface   *const joint_state_iface   = hw->get<hardware_interface::JointStateInterface>();

	indexer_linebreak_1_joint_ = joint_state_iface->getHandle("indexer_line_break_1");
	indexer_linebreak_2_joint_ = joint_state_iface->getHandle("indexer_line_break_2");
	indexer_linebreak_3_joint_ = joint_state_iface->getHandle("indexer_line_break_3");
	indexer_linebreak_4_joint_ = joint_state_iface->getHandle("indexer_line_break_4");

	//initialize cargo_intake_joint (spinny thing)
	//read cargo intake name from config file
    XmlRpc::XmlRpcValue indexer_params;
    if (!controller_nh.getParam("indexer_joint", indexer_params))
    {
        ROS_ERROR_STREAM("Can not read indexer joint name");
        return false;
    }

	//initialize cargo intake joint
    if (!indexer_motor_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, indexer_params))
    {
        ROS_ERROR("Cannot initialize indexer motor joint!");
        return false;
    }
	else
    {
        ROS_INFO("Initialized indexer motor joint!");
    }

	shooter_ready_callback_ = controller_nh.subscribe("shooter_ready", 1, &IndexerTestController::shooterReadyCallback, this);

	controller_nh.param("indexer_motor_speed", indexer_motor_speed_, .2);
    ddr_.registerVariable<double>("indexer_motor_speed", &indexer_motor_speed_, "percent out setting used to run indexer motor", 0, 1);
	controller_nh.param("delay_before_stop_motor", delay_before_stop_motor_, .2);
    ddr_.registerVariable<double>("delay_before_stop_motor", &delay_before_stop_motor_, "time to wait after 2nd linebreak sensor passed before stopping motor", 0, 5);
    ddr_.publishServicesTopics();

	return true;
}

void IndexerTestController::starting(const ros::Time &/*time*/) {
	// Default to not ready to shoot
	shooter_ready_cmd_.writeFromNonRT(false);
	indexer_state_ = IndexerState_NotShooting_Idle;
	indexer_motor_joint_.setCommand(0);
}

// TODO - service call to reset state machine?
//

void IndexerTestController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
	const bool shooter_ready_cmd = *(shooter_ready_cmd_.readFromRT());

	switch (indexer_state_)
	{
		case IndexerState_NotShooting_Idle:
			indexer_motor_joint_.setCommand(0);
			//ROS_INFO_STREAM("IndexerState_NotShooting_Idle : lb1 = " << indexer_linebreak_1_joint_.getPosition());
			if (indexer_linebreak_1_joint_.getPosition() == 1)
			{
				indexer_motor_joint_.setCommand(indexer_motor_speed_);
				indexer_state_ = IndexerState_NotShooting_RunMotor1;
			}
			break;

		// Wait for power cells to hit the next linebreak sensor
		// then transition to the next state
		case IndexerState_NotShooting_RunMotor1:
			indexer_motor_joint_.setCommand(indexer_motor_speed_);
			ROS_INFO_STREAM("IndexerState_NotShooting_RunMotor1 : lb2 = " << indexer_linebreak_2_joint_.getPosition());
			if (indexer_linebreak_2_joint_.getPosition() == 1)
			{
				indexer_state_ = IndexerState_NotShooting_RunMotor2;
			}
			break;

		// A ball is in progress going past the second linebreak
		// Wait for the sensor to go back low and then return to idle
		case IndexerState_NotShooting_RunMotor2:
			indexer_motor_joint_.setCommand(indexer_motor_speed_);
			ROS_INFO_STREAM("IndexerState_NotShooting_RunMotor2 : lb2 = " << indexer_linebreak_2_joint_.getPosition());
			if ((indexer_linebreak_2_joint_.getPosition() == 0) )
			{
				delay_before_stop_motor_start_time_ = ros::Time::now();
				indexer_state_ = IndexerState_NotShooting_DelayBeforeStopMotor;
			}
			break;

		// Wait a small bit after the 2nd linebreak sensor stops
		// before stopping motors
		case IndexerState_NotShooting_DelayBeforeStopMotor:
			ROS_INFO_STREAM("IndexerState_NotShooting_DelayBeforeStopMotor : delta" << (ros::Time::now() - delay_before_stop_motor_start_time_).toSec());
			if ((ros::Time::now() - delay_before_stop_motor_start_time_).toSec() >= delay_before_stop_motor_)
			{
				indexer_motor_joint_.setCommand(0.);
				indexer_state_ = IndexerState_NotShooting_Idle;
			}

			break;
		default:
			ROS_ERROR_STREAM("Invalid case in IndexerTestController state machine");
			break;
	}
#if 0
	//process input for the up/down part of the intake (pneumatic piston)
	const CargoIntakeCommand cargo_intake_cmd = *(cargo_intake_cmd_.readFromRT());
	double intake_arm_cmd_double; //to store processed input
	if(cargo_intake_cmd.intake_arm_cmd_ == true) {
		intake_arm_cmd_double = 1;
	}
	else {
		intake_arm_cmd_double = 0;
	}
	//ROS_WARN_STREAM("cargo intake arm command: " << intake_arm_cmd_double);

	//read spin command
	cargo_intake_joint_.setCommand(cargo_intake_cmd.spin_cmd_); // set the command to the spinny part of the intake
	cargo_intake_arm_joint_.setCommand(intake_arm_cmd_double); // set the in/out command to the up/down part of the intake
#endif
}

void IndexerTestController::stopping(const ros::Time &/*time*/) {
}

void IndexerTestController::shooterReadyCallback(const std_msgs::Bool &msg)
{
    if(isRunning())
    {
		// True / false if shooter is ready to recieve a new
		// power cell from the indexer.
		shooter_ready_cmd_.writeFromNonRT(msg.data);
	}
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. IndexerTestController is not running.");
    }
}

#if 0
bool IndexerTestController::cmdService(controllers_2019_msgs::CargoIntakeSrv::Request &req, controllers_2019_msgs::CargoIntakeSrv::Response &/*res*/) {
    if(isRunning())
    {
		//take the service request for a certain amount of power (-1 to 1) and write it to the command variable
		//take the service request for in/out (true/false???) and write to a command variable
		cargo_intake_cmd_.writeFromNonRT(CargoIntakeCommand(req.power, req.intake_arm));
	}
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. IndexerTestController is not running.");
        return false;
    }
    return true;
}
#endif

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
#include <pluginlib/class_list_macros.h> //to compile as a controller
PLUGINLIB_EXPORT_CLASS(indexer_test_controller::IndexerTestController, controller_interface::ControllerBase)
