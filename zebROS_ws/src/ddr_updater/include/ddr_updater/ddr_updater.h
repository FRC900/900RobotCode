#pragma once
#include <atomic>
#include <thread>

#include "ros/ros.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

namespace ddr_updater
{
class DDRUpdater
{
	public:
		DDRUpdater(ros::NodeHandle nh);
		~DDRUpdater();

	protected:
		void                                      triggerDDRUpdate(void);
		ddynamic_reconfigure::DDynamicReconfigure ddr_;

	private:
		std::atomic_flag                          ddr_update_thread_flag_;
		std::atomic<bool>                         ddr_update_thread_active_;
		std::thread                               ddr_update_thread_;
		void                                      DDRUpdateThread(void);
};
}
