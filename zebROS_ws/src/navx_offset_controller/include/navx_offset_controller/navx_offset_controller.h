#ifndef NAVX_OFFSET_CONTROLLER
#define NAVX_OFFSET_CONTROLLER


#include <ros/ros.h>
#include <vector>
#include <hardware_interface/joint_state_interface.h> //manages joints that aren't talons
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for realtime buffer (stops multiple things from writing/reading same variable at the same time)
#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h> //manages controllers
#include <atomic>
#include <pluginlib/class_list_macros.h> //needed to compile as a controller

#include <navx_offset_controller/NavXSrv.h>


namespace navx_offset_controller
{
	//define the controller - contains the init, starting, update, and stopping functions needed to run the controller
	//this class is derived (an extension of) the controller_interface::Controller class with the inferred type provided in angle brackets
	class NavXOffsetController : public controller_interface::Controller<hardware_interface::PositionJointInterface>	{
		public:
			//constructor for the controller; doesn't need to do anything
			NavXOffsetController()
			{
			}

			//define the 4 functions that get called to run the controller; the bodies of these functions are defined in the corresponding src file
			virtual bool init(hardware_interface::PositionJointInterface *hw,
								ros::NodeHandle &root_nh,
								ros::NodeHandle &controller_nh) override; //called to when controller initializing - initialize stuff here
			virtual void starting(const ros::Time &time) override; //called when controller starting, we don't really use it for much
			virtual void update(const ros::Time &time, const ros::Duration &period) override; //called in a loop when controller running
			virtual void stopping(const ros::Time &time) override; //called when controller stopping

			//define the function that executes the service each time a request is received
			bool cmdService(navx_offset_controller::NavXSrv::Request &req, navx_offset_controller::NavXSrv::Response &res);

		private:
			//define the service server that will accept requests and do something each time it gets a request
			ros::ServiceServer navX_offset_service_;

			//define a variable to hold the most recent command - buffer is used so that the update() loop (or any other code) doesn't try to read this variable at the same time something is writing to it
			realtime_tools::RealtimeBuffer<double> service_command_;

			//define the interface/handle for the joint
			hardware_interface::JointHandle navX_joint_;

	}; //class end

} //namespace end


#endif
