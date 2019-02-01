#ifndef ELEVATOR_CONTROLLER
#define ELEVATOR_CONTROLLER

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h> // "
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <std_msgs/Bool.h>
#include "elevator_controller/ElevatorSrv.h"

namespace elevator_controller
{
//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class ElevatorController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface, hardware_interface::PositionJointInterface>
{
        public:
            ElevatorController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

            bool cmdService(elevator_controller::ElevatorSrv::Request &req,
			                elevator_controller::ElevatorSrv::Response &res);

        private:
            std::vector<std::string> joint_names_; //still not used, but we might have to for config file things?
            talon_controllers::TalonControllerInterface elevator_joint_; //interface for the talon joint

            realtime_tools::RealtimeBuffer<int> position_command_; //this is the buffer for percent output commands to be published
            realtime_tools::RealtimeBuffer<double> timeout_; //buffer for timeout commands

            ros::ServiceServer elevator_service_; //service for receiving commands

			bool zeroed_;
			double arb_feed_forward_up_;
			double initial_position_;
			double elevator_zeroing_percent_output_;

			size_t moving_down_count_;
			ros::Time last_time_down_;
			double elevator_zeroing_timeout_;
}; //class

} //namespace
#endif
