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
		DDRUpdater(ros::NodeHandle n);
		DDRUpdater(const DDRUpdater &) = delete;
		DDRUpdater(DDRUpdater &&) = delete;
		virtual ~DDRUpdater();

		DDRUpdater &operator=(const DDRUpdater &) = delete;
		DDRUpdater &operator=(DDRUpdater &&) = delete;

		void                                      triggerDDRUpdate(void);
		void                                      shutdownDDRUpdater(void);
		ddynamic_reconfigure::DDynamicReconfigure ddr_;

	private:
		std::atomic_flag                          ddr_update_thread_flag_;
		std::atomic<bool>                         ddr_update_thread_active_;
		std::thread                               ddr_update_thread_;
		void                                      DDRUpdateThread(void);
};
}
