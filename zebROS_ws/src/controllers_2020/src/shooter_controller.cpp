#include "controllers_2020/shooter_controller.h"

namespace shooter_controller
{
    bool ShooterController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {

        //get interface
        //hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

        //Initialize piston joints
        /* Ex:
        push_joint_ = pos_joint_iface->getHandle("joint_name"); //joint_name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml
        */

        //Initialize motor joints
        /* Ex:
        //get params from config file
        XmlRpc::XmlRpcValue intake_motor_params;
        if ( !controller_nh.getParam("config_value_name", intake_motor_params)) //grabbing the config value under the controller's section in the main config file
        {
            ROS_ERROR_STREAM("Could not read _______ params");
            return false;
        }
        //initialize motor joint using those config values
        if ( !motor_name_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, intake_motor_params) {
            ROS_ERROR("Cannot initialize ______ joint!");
            return false;
        }
        */


        //Initialize your ROS server
        /* Ex:
        shooter_service_ = controller_nh.advertiseService("shooter_command", &ShooterController::cmdService, this);
        */
        return true;
    }

    void ShooterController::starting(const ros::Time &/*time*/) {
        //give command buffer(s) an initial value
        /* Ex:
        cmd_buffer_.writeFromNonRT(true);
        */

    }

    void ShooterController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
        //grab value from command buffer(s)
        /* Ex:
        const bool extend_cmd = *(cmd_buffer_.readFromRT());
        */


        //Set values of the pistons based on the command. Can be 1.0, 0.0, or -1.0. -1.0 is only used with double solenoids
        /* Syntax: push_joint_.setCommand(1.0); */

        //for motors, it's the same syntax, but the meaning of the argument passed to setCommand() differs based on what motor mode you're using

    }

    void ShooterController::stopping(const ros::Time &/*time*/) {
    }
    /*
    bool ShooterController::cmdService(package::ShooterSrv::Request &req, package::ShooterSrv::Response &//response//) {
        if(isRunning())
        {
            //assign request value to command buffer(s)
            //Ex:
            cmd_buffer_.writeFromNonRT(req.claw_release);
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. ShooterController is not running.");
            return false;
        }
        return true;
    }
    */

}//namespace

PLUGINLIB_EXPORT_CLASS(shooter_controller::ShooterController, controller_interface::ControllerBase)

