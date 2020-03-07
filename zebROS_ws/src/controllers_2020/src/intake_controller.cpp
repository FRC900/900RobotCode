#include "controllers_2020/intake_controller.h"
#include <pluginlib/class_list_macros.h> //to compile as a controller

namespace intake_controller
{
    bool IntakeController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
        hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

        //Initialize intake piston joint
        intake_arm_joint_ = pos_joint_iface->getHandle("intake_arm_joint"); //read from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml

        //Initialize motor joints
        //get params from config file
        XmlRpc::XmlRpcValue intake_params;
        if ( !controller_nh.getParam("intake_joint", intake_params)) //grabbing the config value under the controller's section in the main config file
        {
            ROS_ERROR_STREAM("Could not read intake_params");
            return false;
        }

        //initialize motor joint using those config values
        if (!intake_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, intake_params)) {
            ROS_ERROR("Cannot initialize intake_joint!");
            return false;
        }
	else
	{
		ROS_INFO("Initialized intake joint");
	}

        //Initialize your ROS server
        intake_arm_service_ = controller_nh.advertiseService("intake_arm_command", &IntakeController::cmdServiceArm, this);
        intake_roller_service_ = controller_nh.advertiseService("intake_roller_command", &IntakeController::cmdServiceRoller, this);
		intake_disable_service_ = controller_nh.advertiseService("intake_disable", &IntakeController::disableIntakeCallback, this);

		return true;
    }

    void IntakeController::starting(const ros::Time &/*time*/) {
        //give command buffer(s) an initial value
		arm_extend_cmd_buffer_.writeFromNonRT(false);
		percent_out_cmd_buffer_.writeFromNonRT(0.0);

		forward_disabled_.writeFromNonRT(false);
    }

    void IntakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
        //grab value from command buffer(s)
        const bool arm_extend_cmd = *(arm_extend_cmd_buffer_.readFromRT());
		double arm_extend_double;
		if(arm_extend_cmd == true){
			arm_extend_double = 0.0;
		}
		else {
			arm_extend_double = 1.0;
		}

		//if moving forwards was disabled by the indexer server, don't allow forward movement
		double percent_out_cmd = *percent_out_cmd_buffer_.readFromRT();
		if(*forward_disabled_.readFromRT() && percent_out_cmd > 0)
		{
			percent_out_cmd = 0.0;
		}

		intake_joint_.setCommand(percent_out_cmd);
		intake_arm_joint_.setCommand(arm_extend_double);
    }

    void IntakeController::stopping(const ros::Time &/*time*/) {
    }

	bool IntakeController::cmdServiceArm(controllers_2020_msgs::IntakeArmSrv::Request &req, controllers_2020_msgs::IntakeArmSrv::Response &/*response*/) {
        if(isRunning())
        {
            //assign request value to command buffer(s)
            arm_extend_cmd_buffer_.writeFromNonRT(req.intake_arm_extend);
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. IntakeController is not running.");
            return false;
        }
        return true;
    }

	bool IntakeController::cmdServiceRoller(controllers_2020_msgs::IntakeRollerSrv::Request &req, controllers_2020_msgs::IntakeRollerSrv::Response &/*response*/) {
        if(isRunning())
        {
            //assign request value to command buffer(s)
            percent_out_cmd_buffer_.writeFromNonRT(req.percent_out);
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. IntakeController is not running.");
            return false;
        }
        return true;
    }

	bool IntakeController::disableIntakeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &) {
        if(isRunning())
        {
            forward_disabled_.writeFromNonRT(req.data);
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. IntakeController is not running.");
            return false;
        }
        return true;
    }


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(intake_controller::IntakeController, controller_interface::ControllerBase)

