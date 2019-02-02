#ifndef PANEL_INTAKE_CONTROLLER
#define PANEL_INTAKE_CONTROLLER

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/controller.h> //for writing controllers
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <std_msgs/Bool.h>
#include "panel_intake_controller/PanelIntakeSrv.h"
#include "sensor_msgs/JointState.h"

namespace panel_intake_controller
{
//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class PanelIntakeController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            PanelIntakeController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::PositionJointInterface *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

            virtual bool cmdService(panel_intake_controller::PanelIntakeSrv::Request &req,
					                panel_intake_controller::PanelIntakeSrv::Response &res);

        private:
            std::vector<std::string> joint_names_; //still not used, but we might have to for config file things?
            hardware_interface::JointHandle claw_joint_; //interface for the in/out solenoid joint
 			hardware_interface::JointHandle push_joint_;

            realtime_tools::RealtimeBuffer<bool> claw_cmd_; //this is the buffer for percent output commands to be published
            realtime_tools::RealtimeBuffer<bool> push_cmd_; //buffer for in/out commands 

            ros::ServiceServer panel_intake_service_; //service for receiving commands
			ros::Subscriber joint_states_sub_; //to subscribe to joint states - for sensors
}; //class

} //namespace
#endif
