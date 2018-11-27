//TO DO:V
//make siure the controller_nh.param is reading talons correctly
//configure talons in YAML file correctly
//figure out service response
//process state integer to actual talon input

#include "arm_controller/arm_controller.h"

namespace arm_controller
{

bool ArmController::init(hardware_interface::RobotHW *hw,
							ros::NodeHandle			&root_nh,
							ros::NodeHandle			&controller_nh)
{
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
        hardware_interface::JointStateInterface *const joint_state_iface = hw->get<hardware_interface::JointStateInterface>();

        //limit_switch_intake_ = joint_state_iface->getHandle("limit_switch_intake");
        //limit_switch_exchange_ = joint_state_iface->getHandle("limit_switch_exchange");
        //read parameters and names
        if (!controller_nh.getParam("position_array", arm_positions_))
        {
            ROS_ERROR_STREAM("Could not read arm_positions");
            return false;
        }
	

	if (!controller_nh.getParam("position_array_with_cube", arm_positions_with_cube_))
	{
		ROS_ERROR_STREAM("Could not read arm_positions_with_cube");

	}

        double forward_soft_limit;
        if (!controller_nh.getParam("forward_soft_limit", forward_soft_limit))
        {
            ROS_ERROR_STREAM("Could not read forward_soft_limit");
            return false;
        }
        double reverse_soft_limit;
        if (!controller_nh.getParam("reverse_soft_limit", reverse_soft_limit))
        {
            ROS_ERROR_STREAM("Could not read reverse_soft_limit");
            return false;
        }
        XmlRpc::XmlRpcValue arm_params;
        if(!controller_nh.getParam("arm_joint", arm_params))
        {
            ROS_ERROR_STREAM("Could not read arm_joint name");
        }

        //initialize the joint
        if (!arm_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, arm_params))
        {
            ROS_ERROR("Cannot initialize arm_joint");
            return false;
        }
        else
        {
            ROS_INFO("Initialized arm_joint");
        }

        /*arm_joint_.setForwardSoftLimitThreshold(forward_soft_limit);
        arm_joint_.setReverseSoftLimitThreshold(reverse_soft_limit);
        arm_joint_.setForwardSoftLimitEnable(true);
        arm_joint_.setReverseSoftLimitEnable(true);*/
        arm_joint_.setPeakOutputForward(1);
        arm_joint_.setPeakOutputReverse(-1);
        arm_joint_.setPIDFSlot(0);

        ROS_INFO_STREAM("arm_joint_.getMotionCruiseVelocity = " << arm_joint_.getMotionCruiseVelocity());

        arm_state_service_ = controller_nh.advertiseService("arm_state_service", &ArmController::cmdService, this);
        stop_arm_srv_ = controller_nh.advertiseService("stop_arm_srv", &ArmController::stop_arm_service, this);
        arm_cur_command_srv_ = controller_nh.advertiseService("arm_cur_command_srv", &ArmController::arm_cur_command_service, this);
        command_pub_ = controller_nh.advertise<std_msgs::Float64>("arm_command", 1);
	joint_states_sub = controller_nh.subscribe("joint_states", &ArmController::joint_states_callback,this);

	return true;
}

void joint_states_callback(const sensor_msgs::JointState &joint_state)
{
	static size t cube idx = std::numeric_limits<size t>::max();
	if ((cube_idx >= joint_state.name.size()))
	
	{
		for (size t i=0; i < joint_state.name.size(); i++)
		{
			if (joint_state.name[i] == "intake_line_break")
				cube_idx =i;
		}
	} 
	bool cube_state = (joint_state.position[cube_idx] !=0);
}


void ArmController::starting(const ros::Time &time) {
	ROS_ERROR_STREAM("ArmController was started");
        service_command_.writeFromNonRT(1);
}

void ArmController::update(const ros::Time &time, const ros::Duration &period) {
	// TODO : translate from a number to an arm posttion
	// An idea - create a param which is an array.  Use the value
	// read here to index into the array. That is, the command here is the
	// index of an array, and the value at that index is the position to move
	// the arm to. Be sure to do bounds checking - make sure you don't index
	// past the end of the array.  But this will make it very easy
	// to configure different positions for the arm simply by changing a config file
        size_t command = *(service_command_.readFromRT());
        ROS_INFO_STREAM("arm_joint command = " << command );
        bool stop_arm = *(stop_arm_.readFromRT());
        ROS_INFO_STREAM("stop_arm = " << stop_arm);

        //stop arm
        if(stop_arm)
        {
            arm_joint_.setPeakOutputForward(0);
            arm_joint_.setPeakOutputReverse(0);
        }
        else
        {
            arm_joint_.setPeakOutputForward(1);
            arm_joint_.setPeakOutputReverse(-1);
        }

        //set to motor
        if (command < arm_positions_.size())
        {
         if(!cube_state) 
	 {double position = arm_positions_[command];}
	 else 
		{ double position = arm_position_with_cube_[command];}
            arm_joint_.setCommand(position);
            //pub most recent command
            std_msgs::Float64 msg;
            msg.data = command;
            command_pub_.publish(msg);
        
	}
        else
            ROS_ERROR_STREAM("the command to arm_controller needs to be 0, 1, or 2");
	
}

void ArmController::stopping(const ros::Time &time) {
}

bool ArmController::cmdService(arm_controller::SetArmState::Request &req, arm_controller::SetArmState::Response &res) {
	if(isRunning())
	{
		service_command_.writeFromNonRT(req.position); //write service request
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. ArmController is not running.");
		return false;
	}
	return true;
}

bool ArmController::arm_cur_command_service(arm_controller::CurArmCommand::Request &req, arm_controller::CurArmCommand::Response &res) {
    size_t cur_command = *(service_command_.readFromRT());
    if (cur_command < arm_positions_.size())
    {
        double position = arm_positions_[cur_command];
        res.cur_command = position;
    }
    else
        ROS_ERROR_STREAM("the command to arm_controller needs to be 0, 1, or 2");
    return true;
}

bool ArmController::stop_arm_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	if(isRunning())
	{
            stop_arm_.writeFromNonRT(req.data);
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. ArmController is not running.");
		return false;
	}
	return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS( arm_controller::ArmController, controller_interface::ControllerBase)
