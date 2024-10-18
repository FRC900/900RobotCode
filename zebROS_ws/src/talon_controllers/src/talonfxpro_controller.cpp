#include <ros/ros.h>

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include "ctre_interfaces/talonfxpro_command_interface.h"
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64.h>
#include <talon_controllers/talonfxpro_controller_interface.h>
#include <talon_controller_msgs/Command.h>
#include <talon_controller_msgs/PidfSlot.h>

namespace talonfxpro_controllers
{

/**
 * \brief Simple TalonFXPro/V6 Controllers`
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
 * Service servers:
 * - \b command (std_msgs::Float64) : The joint setpoint to apply
 */

// Since most controllers are going to share a lot of common code,
// create a base class template. The big difference between controllers
// will be the mode the Talon is run in. This is specificed by the type
// of talon interface, so make this the templated parameter.
// The function used to update the Talon is also a template parameter.
// This would be e.g. setControlOutput, setControlPosition, etc depending
// on the mode selected for the particular controller in question.
template <typename TALON_IF, auto UPDATE_FN>
class TalonFXProController:
	public controller_interface::Controller<hardware_interface::talonfxpro::TalonFXProCommandInterface>
{
	public:
		TalonFXProController() =default;
        TalonFXProController(const TalonFXProController &) = delete;
        TalonFXProController(TalonFXProController &&) noexcept = delete;
        ~TalonFXProController() override = default;

        TalonFXProController &operator=(const TalonFXProController &other) = delete;
        TalonFXProController &operator=(TalonFXProController &&other) noexcept = delete;

		bool init(hardware_interface::talonfxpro::TalonFXProCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw, nullptr, n))
				return false;

			// Might want to make message type a template parameter as well?
			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &TalonFXProController::commandCB, this, ros::TransportHints().tcpNoDelay());
			service_command_ = n.advertiseService("command", &TalonFXProController::commandService, this);
			return true;
		}

		void starting(const ros::Time & /*time*/) override
		{
			// Start controller with motor stopped for great safety
			command_buffer_.writeFromNonRT(0.0);
		}
		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Take the most recent value stored in the command
			// buffer (the most recent value read from the "command"
			// topic) and set the Talon to that commanded value
			(talon_if_.*UPDATE_FN)(*command_buffer_.readFromRT());
		}

	protected:
		TALON_IF talon_if_;

	private:
		ros::Subscriber sub_command_;
		ros::ServiceServer service_command_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.  This buffer is read in each call to update()
		// to get the command to send to the Talon
		realtime_tools::RealtimeBuffer<double> command_buffer_;

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

		// Same as above, but this time the command is sent via a service
		bool commandService(talon_controller_msgs::Command::Request &req, talon_controller_msgs::Command::Response &res)
		{
			command_buffer_.writeFromNonRT(req.command);
			res.success = true;
			return true;
		}
};

using TalonFXProDutyCycleOutController = TalonFXProController<TalonFXProDutyCycleOutControllerInterface, &TalonFXProDutyCycleOutControllerInterface::setControlOutput>;
using TalonFXProTorqueCurrentFOCController = TalonFXProController<TalonFXProTorqueCurrentFOCControllerInterface, &TalonFXProTorqueCurrentFOCControllerInterface::setControlOutput>;
using TalonFXProVoltageOutputController = TalonFXProController<TalonFXProVoltageOutControllerInterface, &TalonFXProVoltageOutControllerInterface::setControlOutput>;
using TalonFXProMusicToneController = TalonFXProController<TalonFXProMusicToneControllerInterface, &TalonFXProMusicToneControllerInterface::setControlOutput>;

// Add a service to set PIDF config slot for all close-loop controllers
template <typename TALON_IF, auto UPDATE_FN>
class TalonFXProCloseLoopController :
	public TalonFXProController<TALON_IF, UPDATE_FN>
{
	public:
		TalonFXProCloseLoopController() =default;
        TalonFXProCloseLoopController(const TalonFXProCloseLoopController &) = default;
        TalonFXProCloseLoopController(TalonFXProCloseLoopController &&) noexcept = default;
        virtual ~TalonFXProCloseLoopController() = default;

        TalonFXProCloseLoopController &operator=(const TalonFXProCloseLoopController &other) = default;
        TalonFXProCloseLoopController &operator=(TalonFXProCloseLoopController &&other) noexcept = default;

		bool init(hardware_interface::talonfxpro::TalonFXProCommandInterface *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!TalonFXProController<TALON_IF, UPDATE_FN>::init(hw,n))
				return false;

			pidf_service_ = n.advertiseService("pidf_slot", &TalonFXProCloseLoopController::pidf_slot_service, this);
			return true;
		}

	private:
		ros::ServiceServer pidf_service_;
		bool pidf_slot_service(talon_controller_msgs::PidfSlot::Request  &req,
		                       talon_controller_msgs::PidfSlot::Response &/*res*/)
		{
			this->talon_if_.setControlSlot(req.pidf_slot);
			return true;
		}
};

using TalonFXProPositionDutyCycleController = TalonFXProCloseLoopController<TalonFXProPositionDutyCycleControllerInterface, &TalonFXProPositionDutyCycleControllerInterface::setControlPosition>;
using TalonFXProPositionVoltageController = TalonFXProCloseLoopController<TalonFXProPositionVoltageControllerInterface, &TalonFXProPositionVoltageControllerInterface::setControlPosition>;
using TalonFXProPositionTorqueCurrentFOCController = TalonFXProCloseLoopController<TalonFXProPositionTorqueCurrentFOCControllerInterface, &TalonFXProPositionTorqueCurrentFOCControllerInterface::setControlPosition>;
using TalonFXProVelocityDutyCycleController = TalonFXProCloseLoopController<TalonFXProVelocityDutyCycleControllerInterface, &TalonFXProVelocityDutyCycleControllerInterface::setControlVelocity>;
using TalonFXProVelocityVoltageController = TalonFXProCloseLoopController<TalonFXProVelocityVoltageControllerInterface, &TalonFXProVelocityVoltageControllerInterface::setControlVelocity>;
using TalonFXProVelocityTorqueCurrentFOCController = TalonFXProCloseLoopController<TalonFXProVelocityTorqueCurrentFOCControllerInterface, &TalonFXProVelocityTorqueCurrentFOCControllerInterface::setControlVelocity>;
using TalonFXProMotionMagicDutyCycleController = TalonFXProCloseLoopController<TalonFXProMotionMagicDutyCycleControllerInterface, &TalonFXProMotionMagicDutyCycleControllerInterface::setControlPosition>;
using TalonFXProMotionMagicVoltageController = TalonFXProCloseLoopController<TalonFXProMotionMagicVoltageControllerInterface, &TalonFXProMotionMagicVoltageControllerInterface::setControlPosition>;
using TalonFXProMotionMagicTorqueCurrentFOCController = TalonFXProCloseLoopController<TalonFXProMotionMagicTorqueCurrentFOCControllerInterface, &TalonFXProMotionMagicTorqueCurrentFOCControllerInterface::setControlPosition>;
using TalonFXProMotionMagicExpoDutyCycleController = TalonFXProCloseLoopController<TalonFXProMotionMagicExpoDutyCycleControllerInterface, &TalonFXProMotionMagicExpoDutyCycleControllerInterface::setControlPosition>;
using TalonFXProMotionMagicExpoVoltageController = TalonFXProCloseLoopController<TalonFXProMotionMagicExpoVoltageControllerInterface, &TalonFXProMotionMagicExpoVoltageControllerInterface::setControlPosition>;
using TalonFXProMotionMagicExpoTorqueCurrentFOCController = TalonFXProCloseLoopController<TalonFXProMotionMagicExpoTorqueCurrentFOCControllerInterface, &TalonFXProMotionMagicExpoTorqueCurrentFOCControllerInterface::setControlPosition>;
using TalonFXProMotionMagicVelocityDutyCycleController = TalonFXProCloseLoopController<TalonFXProMotionMagicVelocityDutyCycleControllerInterface, &TalonFXProMotionMagicVelocityDutyCycleControllerInterface::setControlPosition>;
using TalonFXProMotionMagicVelocityVoltageController = TalonFXProCloseLoopController<TalonFXProMotionMagicVelocityVoltageControllerInterface, &TalonFXProMotionMagicVelocityVoltageControllerInterface::setControlPosition>;
using TalonFXProMotionMagicVelocityTorqueCurrentFOCController = TalonFXProCloseLoopController<TalonFXProMotionMagicVelocityTorqueCurrentFOCControllerInterface, &TalonFXProMotionMagicVelocityTorqueCurrentFOCControllerInterface::setControlPosition>;
using TalonFXProDynamicMotionMagicDutyCycleController = TalonFXProCloseLoopController<TalonFXProDynamicMotionMagicDutyCycleControllerInterface, &TalonFXProDynamicMotionMagicDutyCycleControllerInterface::setControlPosition>;
using TalonFXProDynamicMotionMagicVoltageController = TalonFXProCloseLoopController<TalonFXProDynamicMotionMagicVoltageControllerInterface, &TalonFXProDynamicMotionMagicVoltageControllerInterface::setControlPosition>;
using TalonFXProDynamicMotionMagicTorqueCurrentFOCController = TalonFXProCloseLoopController<TalonFXProDynamicMotionMagicTorqueCurrentFOCControllerInterface, &TalonFXProDynamicMotionMagicTorqueCurrentFOCControllerInterface::setControlPosition>;
// Follower controller sets up a Talon to mirror the actions
// of another talon. This talon is defined by joint name in
// params/yaml config.
template <class TALON_IF>
class TalonFXProFollowerControllerBase:
	public controller_interface::MultiInterfaceController<hardware_interface::talonfxpro::TalonFXProCommandInterface,
														  hardware_interface::talonfxpro::TalonFXProStateInterface>
{
	public:
		TalonFXProFollowerControllerBase() =default;
        TalonFXProFollowerControllerBase(const TalonFXProFollowerControllerBase &) = default;
        TalonFXProFollowerControllerBase(TalonFXProFollowerControllerBase &&) noexcept = default;
        ~TalonFXProFollowerControllerBase() override = default;

        TalonFXProFollowerControllerBase &operator=(const TalonFXProFollowerControllerBase &other) = default;
        TalonFXProFollowerControllerBase &operator=(TalonFXProFollowerControllerBase &&other) noexcept = default;

		bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &n) override
		{
			// Read params from command line / config file
			if (!talon_if_.initWithNode(hw->get<hardware_interface::talonfxpro::TalonFXProCommandInterface>(),
										hw->get<hardware_interface::talonfxpro::TalonFXProStateInterface>(), n))
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
		TALON_IF talon_if_;
};

using TalonFXProFollowerController = TalonFXProFollowerControllerBase<TalonFXProFollowerControllerInterface>;
using TalonFXProStrictFollowerController = TalonFXProFollowerControllerBase<TalonFXProStrictFollowerControllerInterface>;

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProDutyCycleOutController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProTorqueCurrentFOCController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProVoltageOutputController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProPositionDutyCycleController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProPositionVoltageController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProPositionTorqueCurrentFOCController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProVelocityDutyCycleController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProVelocityVoltageController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProVelocityTorqueCurrentFOCController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicDutyCycleController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicVoltageController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicTorqueCurrentFOCController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicExpoDutyCycleController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicExpoVoltageController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicExpoTorqueCurrentFOCController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicVelocityDutyCycleController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicVelocityVoltageController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMotionMagicVelocityTorqueCurrentFOCController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProDynamicMotionMagicDutyCycleController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProDynamicMotionMagicVoltageController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProDynamicMotionMagicTorqueCurrentFOCController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProFollowerController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProStrictFollowerController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(talonfxpro_controllers::TalonFXProMusicToneController,
					   controller_interface::ControllerBase)