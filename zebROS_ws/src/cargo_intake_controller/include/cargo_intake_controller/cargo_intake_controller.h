#ifndef CARGO_INTAKE_CONTROLLER
#define CARGO_INTAKE_CONTROLLER

#include <ros/ros.h>
#include <vector>
#include <hardware_interface/joint_state_interface.h> //other than talon data
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <boost/shared_ptr.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller.h> //for writing controllers
#include <talon_interface/talon_state_interface.h> // "
#include <talon_controllers/talon_controller.h> // "
#include <talon_controllers/talon_controller_interface.h> // "
#include <atomic>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <cargo_intake_controller/CargoIntakeSrv.h>

namespace cargo_intake_controller
{
//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class CargoIntakeController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface, hardware_interface::PositionJointInterface>
{
        public:
            CargoIntakeController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh);
            virtual void starting(const ros::Time &time);
            virtual void update(const ros::Time & time, const ros::Duration& period);
            virtual void stopping(const ros::Time &time);

            virtual bool cmdService(cargo_intake_controller::CargoIntakeSrv::Request &req,
					                cargo_intake_controller::CargoIntakeSrv::Response &res);

        private:
            std::vector<std::string> joint_names_; //still not used, but we might have to for config file things?
            talon_controllers::TalonPercentOutputControllerInterface cargo_intake_joint_; //interface for the spinny part of the intake
			hardware_interface::JointHandle cargo_intake_arm_joint_; //interface for the up/down arm of the intake

            realtime_tools::RealtimeBuffer<double> spin_command_; //this is the buffer for percent output commands to be published
            realtime_tools::RealtimeBuffer<double> intake_arm_command_; //buffer for commands for up/down of the arm
            realtime_tools::RealtimeBuffer<double> timeout_; //buffer for timeout commands

            ros::ServiceServer cargo_intake_service_; //service for receiving commands
}; //class

} //namespace
#endif
