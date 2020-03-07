#include "controllers_2020/climber_controller.h"

namespace climber_controller_2020
{
    bool ClimberController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {
		//get interface
        hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();
		hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

		//Initialize piston joints
		deploy_joint_ = pos_joint_iface->getHandle("climber_deploy_joint");
        brake_joint_ = pos_joint_iface->getHandle("climber_elevator_brake_joint");

        //Initialize motor joints
        //get params from config file
        XmlRpc::XmlRpcValue winch_motor_params;
        if ( !controller_nh.getParam("winch_joint", winch_motor_params)) //grabbing the config value under the controller's section in the main config file
        {
            ROS_ERROR_STREAM("Could not read climber winch_joint_params");
            return false;
        }

		if( !controller_nh.getParam("softlimit_offset_from_start", softlimit_offset_))
		{
			ROS_ERROR("Could not read softlimit_offset_from_start in climber_controller");
			softlimit_offset_ = 0.1;
		}

        //initialize motor joint using those config values
        if ( !winch_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, winch_motor_params) ) {
            ROS_ERROR("Cannot initialize winch joint!");
            return false;
        }

        //Initialize your ROS server
        climber_service_ = controller_nh.advertiseService("climber_command", &ClimberController::cmdService, this);

        return true;
    }

    void ClimberController::starting(const ros::Time &/*time*/) {
        // Set the initial position of the winch to 0
        winch_joint_.setSelectedSensorPosition(0.0);

		// set reverse softlimit to just a bit above initial position
		winch_joint_.setReverseSoftLimitThreshold(softlimit_offset_);
		winch_joint_.setReverseSoftLimitEnable(true);

        //give command buffer(s) an initial value
        cmd_buffer_.writeFromNonRT(ClimberCommand(0.0, false, true));
    }

    void ClimberController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
        //grab value from command buffer(s)
        const ClimberCommand cmd = *(cmd_buffer_.readFromRT());
		const bool brake_cmd = cmd.brake_;
		const bool deploy_cmd = cmd.deploy_;
		const double winch_set_point_cmd = cmd.winch_set_point_;

        //Set values of the pistons based on the command. Can be 1.0, 0.0, or -1.0. -1.0 is only used with double solenoids
        if(brake_cmd == true){
			brake_joint_.setCommand(0.0);
		}
		else {
			brake_joint_.setCommand(1.0);
		}

		if(deploy_cmd == true){
			deploy_joint_.setCommand(-1.0);
		}
		else {
			deploy_joint_.setCommand(1.0); 
		}

		//set value of motors
		winch_joint_.setCommand(winch_set_point_cmd);
    }

    void ClimberController::stopping(const ros::Time &/*time*/) {
    }

	bool ClimberController::cmdService(controllers_2020_msgs::ClimberSrv::Request &req,
									   controllers_2020_msgs::ClimberSrv::Response &res) {
        if(isRunning())
        {
            //assign request value to command buffer(s)
            cmd_buffer_.writeFromNonRT(ClimberCommand(req.winch_set_point, req.climber_deploy, req.climber_elevator_brake));
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. ClimberController is not running.");
            return false;
        }
        return true;
    }

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(climber_controller_2020::ClimberController, controller_interface::ControllerBase)

