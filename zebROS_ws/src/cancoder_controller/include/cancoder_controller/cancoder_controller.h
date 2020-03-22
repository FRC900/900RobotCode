#pragma once

#include <controller_interface/controller.h>
#include "cancoder_controller/cancoder_controller_interface.h"

namespace cancoder_controller
{
class CANCoderController: public controller_interface::Controller<hardware_interface::cancoder::CANCoderCommandInterface>
{
	public:
		CANCoderController(void) {}

		virtual bool init(hardware_interface::cancoder::CANCoderCommandInterface *hw,
						  ros::NodeHandle                                        &root_nh,
						  ros::NodeHandle                                        &controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		virtual void stopping(const ros::Time & /*time*/) override;
	private:
		std::unique_ptr<cancoder_controller_interface::CANCoderControllerInterface> interface_;
};
} // namespace cancoder_controller

