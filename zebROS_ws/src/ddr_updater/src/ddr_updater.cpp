#ifdef __linux__
#include <sched.h>
#include <pthread.h>
#endif

#include "ddr_updater/ddr_updater.h"

namespace ddr_updater
{

DDRUpdater::DDRUpdater(ros::NodeHandle n)
	: ddr_(n)
	, ddr_update_thread_(std::bind(&DDRUpdater::DDRUpdateThread, this))
{
}

DDRUpdater::~DDRUpdater()
{
	if (ddr_update_thread_.joinable())
	{
		ddr_update_thread_active_ = false;
		ddr_update_thread_.join();
	}
}

// Trigger a write of the current CIParams to the DDR server. This needs
// to happen if the values have been updated via the interface code.
// This sets a flag to force a write by that thread.
// This code can be called from the control loop update function, which
// needs to be quick.  Atomic flags are lock-free so this function should
// take almost no time to run - the time consuming work of sending an
// update to the DDR server is pushed off into a background thread.
void DDRUpdater::triggerDDRUpdate(void)
{
	ddr_update_thread_flag_.clear();
}

// Loop forever, periodically checking for requests from the main thread
// to update values from this class to the DDR server.
void DDRUpdater::DDRUpdateThread(void)
{
#ifdef __linux__
	struct sched_param sp{};
	sched_setscheduler(0, SCHED_IDLE, &sp);
	pthread_setname_np(pthread_self(), "ddr_update");
#endif

	ddr_update_thread_flag_.test_and_set();
	ddr_update_thread_active_ = true;
	ros::Rate r(10);
	while (ddr_update_thread_active_)
	{
		// Loop forever, periodically checking for the flag to be cleared
		// Test and set returns the previous value of the variable and sets
		// it, all in one atomic operation.  The set will reset the flag
		// so after running updatePublishedInformation() the code will loop back and wait
		// here for the next time the flag is cleared by triggerDDRUpdate.
		if (!ddr_update_thread_flag_.test_and_set())
		{
			ddr_.updatePublishedInformation();
		}
		r.sleep();
	}
}

} // namespace ddr_updater
