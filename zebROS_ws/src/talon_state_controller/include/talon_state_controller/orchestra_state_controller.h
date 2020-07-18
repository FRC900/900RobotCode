#pragma once

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <talon_interface/orchestra_state_interface.h>
#include <talon_state_msgs/OrchestraState.h>
#include <talon_state_msgs/InstrumentList.h>

namespace orchestra_state_controller
{

/**
 * \brief Controller that publishes the state of all orchestras on a robot. (There should really only be one.)
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::OrchestraStateInterface to a
 * topic of type \c talon_state_msgs/OrchestraState. The following is a basic configuration of the controller.
 *
 * \code
 * orchestra_state_controller:
 *   type: talon_state_controller/OrchestraStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class OrchestraStateController: public controller_interface::Controller<hardware_interface::OrchestraStateInterface>
{
	public:
		OrchestraStateController() : publish_rate_(0.0) {}

		virtual bool init(hardware_interface::OrchestraStateInterface *hw,
						  ros::NodeHandle                         &root_nh,
						  ros::NodeHandle                         &controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		virtual void stopping(const ros::Time & /*time*/) override;

	private:
		std::vector<hardware_interface::OrchestraStateHandle> orchestra_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::OrchestraState> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;
		size_t num_hw_joints_; ///< Number of joints present in the OrchestraStateInterface
};
} // namespace
