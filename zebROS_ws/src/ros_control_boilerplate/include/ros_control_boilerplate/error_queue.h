// Class to report errors to the driver station via a service call to
// the RIO hardware interface.
#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>


class ErrorQueue
{
	public:
		ErrorQueue(void);
		~ErrorQueue();
		void enqueue(int32_t errorCode, const std::string &details);
	private:
		void error_queue_thread_fn(void);
		std::thread error_queue_thread_;

		std::queue<std::pair<int32_t, std::string>> error_queue_;
		std::mutex error_queue_mutex_;
		std::condition_variable error_queue_condition_variable_;
};

extern std::unique_ptr<ErrorQueue> errorQueue;
