#pragma once

#include <controller_interface/controller.h>
#include "canifier_controller/canifier_controller_interface.h"

namespace canifier_controller
{
class CANifierController: public controller_interface::Controller<hardware_interface::canifier::CANifierCommandInterface>
{
	public:
		CANifierController(void) {}

		virtual bool init(hardware_interface::canifier::CANifierCommandInterface *hw,
						  ros::NodeHandle                                        &root_nh,
						  ros::NodeHandle                                        &controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		virtual void stopping(const ros::Time & /*time*/) override;
	private:
		std::unique_ptr<canifier_controller_interface::CANifierControllerInterface> interface_;
};
} // namespace canifier_controller

