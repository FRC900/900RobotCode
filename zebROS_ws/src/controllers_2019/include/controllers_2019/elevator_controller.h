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
#include "controllers_2019/ElevatorSrv.h"
#include <dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h>
#include <controllers_2019/ElevatorConfig.h>

namespace elevator_controller
{

	class ElevatorCommand
{
	public:
		ElevatorCommand()
		{
		  position_ = 0;
		  go_slow_ = false;
		}
		ElevatorCommand(double position, bool go_slow)
		{
		  position_ = position;
		  go_slow_ = go_slow;
		}
		double GetPosition() const
		{
		return position_;
		}
		bool GetGoSlow() const
		{
		return go_slow_;
		}


	private:
		double position_;
		bool go_slow_;
}
;
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

            bool cmdService(controllers_2019::ElevatorSrv::Request &req,
			                controllers_2019::ElevatorSrv::Response &res);

			void callback(elevator_controller::ElevatorConfig &config, uint32_t level);

        private:
            talon_controllers::TalonControllerInterface elevator_joint_; //interface for the talon joint

            realtime_tools::RealtimeBuffer<ElevatorCommand> position_command_; //this is the buffer for percent output commands to be published
            ros::ServiceServer elevator_service_; //service for receiving commands

			bool zeroed_;
			bool last_zeroed_;
			double last_position_;
			//double last_setpoint_;
			hardware_interface::TalonMode last_mode_;

			ros::Time last_time_down_;

			DynamicReconfigureWrapper<ElevatorConfig> dynamic_reconfigure_server_;
			ElevatorConfig config_;

}; //class


} //namespace
#endif
