#pragma once

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/pcm_state_interface.h>
#include <frc_msgs/PCMState.h>

namespace pcm_state_controller
{

/**
 * \brief Controller that publishes the state of all PCMs in a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::PCMStateInterface to a
 * topic of type \c pcm_state_controller/PCMState. The following is a basic configuration of the controller.
 *
 * \code
 * pcm_state_controller:
 *   type: pcm_state_controller/PCMStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class PCMStateController: public controller_interface::Controller<hardware_interface::PCMStateInterface>
{
	public:
		PCMStateController() : publish_rate_(0.0) {}

		bool init(hardware_interface::PCMStateInterface *hw,
				  ros::NodeHandle                       &root_nh,
				  ros::NodeHandle                       &controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		void stopping(const ros::Time & /*time*/) override;

	private:
		std::vector<hardware_interface::PCMStateHandle> pcm_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::PCMState> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;
		size_t num_pcms_;
};

} // namespace pcm_state_controller
