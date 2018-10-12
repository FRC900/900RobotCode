#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <talon_controllers/talon_controller_interface.h>
#include <talon_controllers/PidfSlot.h>

namespace talon_controllers
{

/**
 * \brief Simple Talon Controllers`
 *
 * These classes implement simple controllers for TalonSRX
 * hardware running in various modes.
 *
 * \section ROS interface
 *
 * \param type Must be "Talon<type>Controller".
 * \param joint Name of the talon-controlled joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint setpoint to apply
 */


// Since most controllers are going to share a lot of common code,
// create a base class template. The big difference between controllers
// will be the mode the Talon is run in. This is specificed by the type
// of talon interface, so make this the templated parameter.
template <class TALON_IF>
class TalonController:
	public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:
		TalonController() {}
		~TalonController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n)
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw, nullptr, n))
				return false;

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &TalonController::commandCB, this);
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
			// topic) and set the Talon to that commanded value
			talon_if_.setCommand(*command_buffer_.readFromRT());
		}

	protected:
		TALON_IF talon_if_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.  This buffer is read in each call to update()
		// to get the command to send to the Talon
		realtime_tools::RealtimeBuffer<double> command_buffer_;

	private:
		ros::Subscriber sub_command_;
		// Take each message read from the "command" topic and push
		// it into the command buffer. This buffer will later be
		// read by update() and sent to the Talon.  The buffer
		// is used because incoming messages aren't necessarily
		// synchronized to update() calls - the buffer holds the
		// most recent command value that update() can use
		// when the update() code is run.
		void commandCB(const std_msgs::Float64ConstPtr &msg)
		{
			command_buffer_.writeFromNonRT(msg->data);
		}
};

class TalonPercentOutputController: public TalonController<TalonPercentOutputControllerInterface>
{
	// Override or add methods different from the base class here
};

// Add a service to set PIDF config slot for all close-loop controllers
template <class TALON_IF>
class TalonCloseLoopController :
	public TalonController<TALON_IF>
{
	public:
		TalonCloseLoopController() { }
		~TalonCloseLoopController() { }

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!TalonController<TALON_IF>::init(hw,n))
				return false;

			pidf_service_ = n.advertiseService("pidf_slot", &TalonCloseLoopController::pidf_slot_service, this);
		}

	private:
		ros::ServiceServer pidf_service_;
		bool pidf_slot_service(talon_controllers::PidfSlot::Request  &req,
		                       talon_controllers::PidfSlot::Response &res)
		{
			return this->talon_if_.setPIDFSlot(req.pidf_slot);
		}

};

class TalonPositionCloseLoopController: public TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>
{
		// Override or add methods here
};
class TalonMotionMagicCloseLoopController: public TalonCloseLoopController<TalonMotionMagicCloseLoopControllerInterface>
{
		// Override or add methods here
};
class TalonVelocityCloseLoopController: public TalonCloseLoopController<TalonVelocityCloseLoopControllerInterface>
{
		// Override or add methods here
};

// Follower controller sets up a Talon to mirror the actions
// of another talon. This talon is defined by joint name in
// params/yaml config.
class TalonFollowerController:
	public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface,
														  hardware_interface::TalonStateInterface>
{
	public:
		TalonFollowerController() {}
		~TalonFollowerController() {}

		bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw->get<hardware_interface::TalonCommandInterface>(),
										hw->get<hardware_interface::TalonStateInterface>(), n))
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
		// Keep ownership of the Talon being run in follower mode.
		// Even though there's currently no commands that can be sent
		// to the Talon keeping this will prevent other controllers
		// from grabbing that Talon until this controller is
		// explicitly unloaded.
		TalonFollowerControllerInterface talon_if_;

};

// Convert Linear Position and Displacement to radians
// and input on the Talon
// Use of Positional PID and the radius of the gear next
// to the Talon
class TalonLinearPositionCloseLoopController :
	public TalonPositionCloseLoopController
{
	//Used radius
	private:
		double radius_;
	public:
		TalonLinearPositionCloseLoopController(void) {}

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>::init(hw,n))
				return false;

			//radius for length
			n.getParam("radius", radius_);
		}

		// Same as TalonClosedLoopController but setCommand
		// has converted radians as input
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			talon_if_.setCommand(*command_buffer_.readFromRT() / radius_);
		}
};

class TalonLinearMotionMagicCloseLoopController :
	public TalonCloseLoopController<TalonMotionMagicCloseLoopControllerInterface>
{
	//Used radius
	private:
		double radius_;
		double gear_ratio_from_encoder_;
	public:
		TalonLinearMotionMagicCloseLoopController(void) {}

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!TalonCloseLoopController<TalonMotionMagicCloseLoopControllerInterface>::init(hw,n))
				return false;

			//radius for length
			n.getParam("radius", radius_);
			//Ratio to convert to correct angle
			n.getParam("gear_ratio_from_encoder", gear_ratio_from_encoder_);
			return true;
		}

		// Same as TalonClosedLoopController but setCommand
		// has converted radians as input
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			talon_if_.setCommand(*command_buffer_.readFromRT() / radius_ / gear_ratio_from_encoder_);
			//ROS_INFO_STREAM(cmd.command_ / radius_ << "works?");
		}
};
class TalonAnglePositionCloseLoopController :
	public TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>
{
	//Ratio
	private:
		double gear_ratio_from_encoder_;
	public:
		TalonAnglePositionCloseLoopController(void) {}

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!TalonCloseLoopController<TalonPositionCloseLoopControllerInterface>::init(hw,n))
				return false;

			//Ratio to convert to correct angle
			n.getParam("gear_ratio_from_encoder", gear_ratio_from_encoder_);
			return true;
		}

		// Same as TalonClosedLoopController but setCommand
		// has converted radians as input
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			talon_if_.setCommand(*command_buffer_.readFromRT() / gear_ratio_from_encoder_);
		}
};

class TalonAngleMotionMagicCloseLoopController :
	public TalonCloseLoopController<TalonMotionMagicCloseLoopControllerInterface>
{
	//Used ratio
	private:
		double gear_ratio_from_encoder_;
	public:
		TalonAngleMotionMagicCloseLoopController(void) {}

		virtual bool init(hardware_interface::TalonCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!TalonCloseLoopController<TalonMotionMagicCloseLoopControllerInterface>::init(hw,n))
				return false;

			//ratio to convert to correct angle
			n.getParam("gear_ratio_from_encoder", gear_ratio_from_encoder_);
			return true;
		}

		// Same as TalonClosedLoopController but setCommand
		// has converted radians as input
		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			talon_if_.setCommand(*command_buffer_.readFromRT() / gear_ratio_from_encoder_);
		}
};

}
