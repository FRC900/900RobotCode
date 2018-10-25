#include "intake_controller/intake_controller.h"

namespace intake_controller
{

bool IntakeController::init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                 &root_nh,
                                                        ros::NodeHandle                 &controller_nh)
{
        hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
        //if I had to read values from fake joints (like line break sensors) I would initialize a JointStateInterface, then getHandle
        //if I had to change non-Talon joint values (like pneumatics) I would initialize a PositionJointInterface, then getHandle

        if (!intake_joint.initWithNode(talon_command_iface, nullptr, controller_nh))
        {
            ROS_ERROR("Cannot initialize joint %d!", i);
            return false;
        }
        else
        {
            ROS_INFO("Initialized joint %d!!", i);
        }
        //set soft limits, deadband, neutral mode, PIDF slots, acceleration and cruise velocity, all the things HERE

        /*joint_1.setPIDFSlot(0);
        joint_1.setMotionAcceleration(1); //TODO
        joint_1.setMotionCruiseVelocity(1); //TODO*/

        service_command_ = controller_nh.advertiseService("intake_posS", &IntakeController::cmdService, this);

        return true;
}

void IntakeController::starting(const ros::Time &time) {
        ROS_ERROR_STREAM("IntakeController was started");
}

void IntakeController::update(const ros::Time &time, const ros::Duration &period) {
        //float curr_cmd = *(command_.readFromRT()); //why do we put it into a new variable
        //ROS_ERROR_STREAM("curr_cmd : " << curr_cmd);
        int final_cmd = *(command_.readFromRT());
	intake_joint.setCommand(final_cmd/2);
}
bool IntakeController::cmdService(intake_controller::IntakeSrv::Request &req, intake_controller::IntakeSrv::Response &/*response*/) {
        if(isRunning())
        {
		int temp_cmd = req.power;
                command_.writeFromNonRT(temp_cmd);
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
PLUGINLIB_EXPORT_CLASS( intake_controller::IntakeController, controller_interface::ControllerBase)

