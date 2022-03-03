#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include <controllers_2022_msgs/Intake.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller

namespace intake_controller
{
class IntakeController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
{

public:
    IntakeController()
    {
    }

    bool init(hardware_interface::RobotHW *hw,
                                ros::NodeHandle                 &/*root_nh*/,
                                ros::NodeHandle                 &controller_nh)
    {
        hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
        hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

		//Initialize intake piston joint
        intake_arm_joint_ = pos_joint_iface->getHandle("intake_solenoid"); //read from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml

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
        intake_service_ = controller_nh.advertiseService("command", &IntakeController::cmdIntake, this);
        return true;
    }

    void starting(const ros::Time &/*time*/) {
        //give command buffer(s) an initial value
        intake_cmd_buffer_.writeFromNonRT({});
    }

    void update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
        //grab value from command buffer(s)
		// naming vars is hard
        const intake_cmd_ intake_data = *(intake_cmd_buffer_.readFromRT());
        double arm_extend_double;
        if(intake_data.arm_extend == true) {
            arm_extend_double = 1.0;
        }
        else {
            arm_extend_double = 0.0;
        }

        intake_joint_.setCommand(intake_data.percent_out);
        intake_arm_joint_.setCommand(arm_extend_double);
    }

    void stopping(const ros::Time &/*time*/) {
    }

private:
    talon_controllers::TalonPercentOutputControllerInterface intake_joint_;//intake for intake motor
    hardware_interface::JointHandle intake_arm_joint_;//interface for intake arm solenoid
    realtime_tools::RealtimeBuffer<bool> arm_extend_cmd_buffer_;
    realtime_tools::RealtimeBuffer<double> percent_out_cmd_buffer_;

	// Things to remember: the realtime buffer can .writeFromNonRT using the struct it was created with, an intake_cmd_ struct still needs to be made to store the command and then write it
	// https://blog.katastros.com/a?ID=00600-d24bebd3-999f-4265-a9a7-fdab37b9f0cc was useful for how to work with structs and realtime
	// You do not need to make the memebers of the class realtime buffers

	struct intake_cmd_ {
         bool arm_extend;
         double percent_out;
		 intake_cmd_()
			 : arm_extend{false}
			 , percent_out(0.0)
		 {
		 }
      };
	intake_cmd_ intake_cmd_struct_;
	realtime_tools::RealtimeBuffer<intake_cmd_> intake_cmd_buffer_;

	ros::ServiceServer intake_service_;

	bool cmdIntake(controllers_2022_msgs::Intake::Request &req, controllers_2022_msgs::Intake::Response &/*response*/) {
		if(isRunning())
          {
              //assign request value to command buffer(s)
			  intake_cmd_struct_.arm_extend = req.intake_arm_extend;
			  intake_cmd_struct_.percent_out = req.percent_out;
			  intake_cmd_buffer_.writeFromNonRT(intake_cmd_struct_);
          }
          else
          {
              ROS_ERROR_STREAM("Can't accept new commands. IntakeController is not running.");
              return false;
          }
          return true;
      }

}; //class
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(intake_controller::IntakeController, controller_interface::ControllerBase)
