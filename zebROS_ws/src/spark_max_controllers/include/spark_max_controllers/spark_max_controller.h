#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <spark_max_controllers/spark_max_controller_interface.h>
//#include <spark_max_controllers/PidfSlot.h>

namespace spark_max_controllers
{

/**
 * \brief Simple SparkMax Controllers`
 *
 * These classes implement simple controllers for SparkMaxSRX
 * hardware running in various modes.
 *
 * \section ROS interface
 *
 * \param type Must be "SparkMax<type>Controller".
 * \param joint Name of the spark_max-controlled joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint setpoint to apply
 */

// Since most controllers are going to share a lot of common code,
// create a base class template. The big difference between controllers
// will be the mode the SparkMax is run in. This is specificed by the type
// of spark_max interface, so make this the templated parameter.
template <class SPARK_MAX_IF>
class SparkMaxController:
	public controller_interface::Controller<hardware_interface::SparkMaxCommandInterface>
{
	public:
		SparkMaxController() {}
		~SparkMaxController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::SparkMaxCommandInterface *hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!spark_max_if_.initWithNode(hw, nullptr, n))
				return false;

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &SparkMaxController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/)
		{
			// Start controller with motor stopped
			// for great safety
			command_buffer_.writeFromNonRT(0.0);
		}
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
		{
			// Take the most recent value stored in the command
			// buffer (the most recent value read from the "command"
			// topic) and set the SparkMax to that commanded value
			spark_max_if_.setCommand(*command_buffer_.readFromRT());
		}

	protected:
		SPARK_MAX_IF spark_max_if_;

	private:
		ros::Subscriber sub_command_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.  This buffer is read in each call to update()
		// to get the command to send to the SparkMax
		realtime_tools::RealtimeBuffer<double> command_buffer_;

		// Take each message read from the "command" topic and push
		// it into the command buffer. This buffer will later be
		// read by update() and sent to the SparkMax.  The buffer
		// is used because incoming messages aren't necessarily
		// synchronized to update() calls - the buffer holds the
		// most recent command value that update() can use
		// when the update() code is run.
		void commandCB(const std_msgs::Float64ConstPtr &msg)
		{
			command_buffer_.writeFromNonRT(msg->data);
		}
};

class SparkMaxDutyCycleController: public SparkMaxController<SparkMaxDutyCycleControllerInterface>
{
	// Override or add methods different from the base class here
};

// Add a service to set PIDF config slot for all close-loop controllers
template <class SPARK_MAX_IF>
class SparkMaxCloseLoopController :
	public SparkMaxController<SPARK_MAX_IF>
{
	public:
		SparkMaxCloseLoopController() { }
		~SparkMaxCloseLoopController() { }

		virtual bool init(hardware_interface::SparkMaxCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!SparkMaxController<SPARK_MAX_IF>::init(hw,n))
				return false;

			//pidf_service_ = n.advertiseService("pidf_slot", &SparkMaxCloseLoopController::pidf_slot_service, this);
			return true;
		}

	private:
#if 0
		ros::ServiceServer pidf_service_;
		bool pidf_slot_service(spark_max_controllers::PidfSlot::Request  &req,
		                       spark_max_controllers::PidfSlot::Response &/*res*/)
		{
			return this->spark_max_if_.setPIDFSlot(req.pidf_slot);
		}
#endif

};

class SparkMaxPositionCloseLoopController: public SparkMaxCloseLoopController<SparkMaxPositionCloseLoopControllerInterface>
{
		// Override or add methods here
};

class SparkMaxVelocityCloseLoopController: public SparkMaxCloseLoopController<SparkMaxVelocityCloseLoopControllerInterface>
{
		// Override or add methods here
};
class SparkMaxVoltageCloseLoopController: public SparkMaxCloseLoopController<SparkMaxVelocityCloseLoopControllerInterface>
{
		// Override or add methods here
};

// Follower controller sets up a SparkMax to mirror the actions
// of another spark_max. This spark_max is defined by joint name in
// params/yaml config.
class SparkMaxFollowerController:
	public controller_interface::MultiInterfaceController<hardware_interface::SparkMaxCommandInterface,
														  hardware_interface::SparkMaxStateInterface>
{
	public:
		SparkMaxFollowerController() {}
		~SparkMaxFollowerController() {}

		bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!spark_max_if_.initWithNode(hw->get<hardware_interface::SparkMaxCommandInterface>(),
										hw->get<hardware_interface::SparkMaxStateInterface>(), n))
				return false;

			return true;
		}

		void starting(const ros::Time & /*time*/) override
		{
		}
		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
		}

	private:
		// Keep ownership of the SparkMax being run in follower mode.
		// Even though there's currently no commands that can be sent
		// to the SparkMax keeping this will prevent other controllers
		// from grabbing that SparkMax until this controller is
		// explicitly unloaded.
		SparkMaxFollowerControllerInterface spark_max_if_;
};

}
