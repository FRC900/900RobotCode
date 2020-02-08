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
        intake_service_ = controller_nh.advertiseService("intake_command", &IntakeController::cmdService, this);

		return true;
    }

    void IntakeController::starting(const ros::Time &/*time*/) {
        //give command buffer(s) an initial value
		intake_cmd_.writeFromNonRT(IntakeCommand(0,false));
    }

    void IntakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
        //grab value from command buffer(s)
        const IntakeCommand intake_cmd = *(intake_cmd_.readFromRT());
		double intake_arm_double;
		if(intake_cmd.intake_arm_extend_ == true){
			intake_arm_double = 1.0;
		}
		else {
			intake_arm_double = 0.0;
		}
		intake_joint_.setCommand(intake_cmd.set_percent_out_);
		intake_arm_joint_.setCommand(intake_arm_double);
    }

    void IntakeController::stopping(const ros::Time &/*time*/) {
    }
    bool IntakeController::cmdService(controllers_2020_msgs::IntakeSrv::Request &req, controllers_2020_msgs::IntakeSrv::Response &/*response*/) {
        if(isRunning())
        {
            //assign request value to command buffer(s)
            //Ex:
            intake_cmd_.writeFromNonRT(IntakeCommand(req.percent_out, req.intake_arm_extend));
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

