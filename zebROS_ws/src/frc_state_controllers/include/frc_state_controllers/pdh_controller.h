#ifndef PDH_STATE_CONTROLLER_INC_
#define PDH_STATE_CONTROLLER_INC_

#include <atomic>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <frc_interfaces/pdh_command_interface.h>

namespace pdh_controller
{
class PDHController: public controller_interface::Controller<hardware_interface::PDHCommandInterface>
{
	public:
		bool init(hardware_interface::PDHCommandInterface *hw,
				  ros::NodeHandle                          &root_nh,
				  ros::NodeHandle                          &controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration &) override;
		void stopping(const ros::Time &time) override;

	private:
		hardware_interface::PDHCommandHandle pdh_command_;
		std::atomic<bool> command_switchable_channel_enable_{false};
		std::atomic<bool> trigger_clear_faults_{false};
		std::atomic<bool> trigger_identify_pdh_{false};

		ros::ServiceServer switchable_channel_enable_service_;
		ros::ServiceServer clear_faults_service_;
		ros::ServiceServer identify_pdh_service_;

		bool switchableChannelEnableService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
		bool clearFaultsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool identifyPDHService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

}; //class

} // namespace hardware_interface

#endif
