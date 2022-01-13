#ifndef PH_STATE_CONTROLLER_INC_
#define PH_STATE_CONTROLLER_INC_

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/ph_state_interface.h>
#include <frc_msgs/PHState.h>

namespace ph_state_controller
{

/**
 * \brief Controller that publishes the state of all PHs in a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::PHStateInterface to a
 * topic of type \c ph_state_controller/PHState. The following is a basic configuration of the controller.
 *
 * \code
 * ph_state_controller:
 *   type: ph_state_controller/PHStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class PHStateController: public controller_interface::Controller<hardware_interface::PHStateInterface>
{
	public:
		bool init(hardware_interface::PHStateInterface *hw,
				  ros::NodeHandle                       &root_nh,
				  ros::NodeHandle                       &controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		void stopping(const ros::Time & /*time*/) override;

	private:
		std::vector<hardware_interface::PHStateHandle> ph_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<frc_msgs::PHState>> realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_{20};
		size_t num_phs_{0};
};

} // namespace ph_state_controller
#endif
