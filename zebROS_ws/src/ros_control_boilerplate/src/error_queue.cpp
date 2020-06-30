#include <ros/ros.h>
#include "ros_control_boilerplate/DSError.h"
// This will be called by non-Rio code which wants to report an error
// It will make a service call to the rio ds_error_service with the
// error data. The rio is then responsible for making an call to
// the actual DS-connected HAL_SendError function to display it on
// the DS.
// The errors are added to a queue so HAL_SendError can return immediately
// The queue is read by a separate thread - error_queue_thread_fn.
// This thread reads from the queue in a loop and makes a service call to
// the rio hardware interface to send each message to the DS in order
//
#include <ros_control_boilerplate/error_queue.h>
ErrorQueue::ErrorQueue(void)
	: error_queue_thread_(std::bind(&ErrorQueue::error_queue_thread_fn, this))
{
}

void ErrorQueue::error_queue_thread_fn(void)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "error_queue");
#endif

	ros::NodeHandle nh;
	auto ds_error_client_ = nh.serviceClient<ros_control_boilerplate::DSError>("/frcrobot_rio/ds_error_service");
	if (!ds_error_client_.waitForExistence(ros::Duration(60)))
	{
		ROS_ERROR("Timeout waiting for /frcrobot_rio/ds_error_service");
		return;
	}

	std::pair<int32_t, std::string> msg_pair;
	while (true)
	{
		// Scope makes sure the lock is released
		// after getting data from the queue and
		// not held during the service call itself
		{
			std::unique_lock<std::mutex> l(error_queue_mutex_);
			while (error_queue_.empty())
				error_queue_condition_variable_.wait(l);
			msg_pair = error_queue_.front();
			error_queue_.pop();
		}
		ros_control_boilerplate::DSError msg;
		// Hack to get the thread to exit when quitting the hwi
		if (msg_pair.first == -900)
			return;
		msg.request.error_code = msg_pair.first;
		msg.request.details    = msg_pair.second;
		if (!ds_error_client_.call(msg.request, msg.response))
		{
			ROS_ERROR("ds_error_client call failed");
		}
	}
}

void ErrorQueue::enqueue(int32_t errorCode, const std::string &details)
{
	std::scoped_lock l(error_queue_mutex_);
    error_queue_.push(std::make_pair(errorCode, std::string(details)));
	error_queue_condition_variable_.notify_one();
}

ErrorQueue::~ErrorQueue(void)
{
	if (error_queue_thread_.joinable())
	{
		enqueue(-900, "");
		error_queue_thread_.join();
	}
}


std::unique_ptr<ErrorQueue> errorQueue;

