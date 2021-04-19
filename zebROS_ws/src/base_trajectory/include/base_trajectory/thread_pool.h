#ifndef THREAD_POOL_INC_
#define THREAD_POOL_INC_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>


//Heavily inspiried by https://www.codeproject.com/Tips/1279911/Advanced-Thread-Pool
// The idea is that in our particular case, thread work tasks are fairly short
// Using a typical shared queue for all threads which only one can access at a time
// leads to contention on that lock.  That contention takes about as much time (or
// more time) than the work itself, leading to the thread function launches be so
// spread out that it is effectively serialized.
class ThreadPoolTask
{
	public:
		ThreadPoolTask()
			: valid_{false}
		{
		}
		// Try to lock the task and get it.
		// If able to both get the lock and there's a valid task
		// waiting, return that task
		// Otherwise, return nullopt to indicate that the task
		// wasn't get'd successfully
		std::optional<std::function<void()>> get()
		{
			std::unique_lock<std::mutex> l(mutex_, std::defer_lock);
			if (l.try_lock() && valid_)
			{
				valid_ = false;
				return std::move(task_);
			}
			return std::nullopt;
		}
		// Try to set this task with a function to run
		// Return true if able to get the tasks' lock and there's
		// not something already stored in the task
		// Return false if the task can't be set
		bool set(std::function<void()> &&task)
		{
			std::unique_lock<std::mutex> l(mutex_, std::defer_lock);
			if (l.try_lock() && !valid_)
			{
				task_ = std::move(task);
				valid_ = true;
				return true;
			}
			return false;
		}
		void setUnconditional(std::function<void()> &&task)
		{
			{
				std::unique_lock<std::mutex> l(mutex_);
				task_ = std::move(task);
				valid_ = true;
			}
		}
	private:
		std::function<void()> task_;
		bool valid_;
		std::mutex mutex_;
};

//thread pool
class ThreadPool
{
public:
    ThreadPool(size_t threadCount = std::thread::hardware_concurrency())
    : tasks_(threadCount * 2 + 1)
	, tasksSize_(tasks_.size())
	{
		for (size_t i = 0; i < threadCount; ++i)
			workers_.emplace_back(std::bind(&ThreadPool::thread_proc, this));
	}
    template<class F> void enqueue(F&& f)
	{
		// Loop through the tasks, looking for one which isn't
		//  - locked by another call to enqueue
		//  - already holding a task
		// When a task entry is found which satisfies this condition,
		// set the entry to that function and increment the jobsToFinish counter
		// Signal worker threads to wake up and grab this function
		// Skipping over locked task entries should be quicker than
		// waiting for a locked one to become unlocked
		unsigned int i = taskIndex_++;
		for (unsigned int n = 0; n < tasksSize_ * K; ++n)
		{
			if (tasks_[(i + n) % tasksSize_].set(f))
			{
				++jobsToFinish_;
				cvTask_.notify_one();
				return;
			}
		}
		// If none of the task entries were unlocked, wait for
		// one of them to unlock. This should never happen (TM)
		tasks_[taskIndex_].setUnconditional(f);
	}
	void waitFinished()
	{
		while (jobsToFinish_ > 0)
		{
			// Busy loop - chews up a cpu waiting, but given how quickly tasks
			// run, this is the better alternative to adding a small sleep value
			// Seems the smallest sleep value is way longer than the amount of time
			// normally waited in this loop.
		}
	}

	void sleep(void)
	{
		sleep_ = true;
	}
	void wakeup(void)
	{
		sleep_ = false;
		cvTask_.notify_all();
	}

	~ThreadPool() // set stop_-condition
	{
		stop_ = true;
		cvTask_.notify_all();

		// all threads terminate, then we're done.
		for (auto& t : workers_)
			t.join();
	}

	size_t getThreadCount() const { return workers_.size(); }

private:
	std::vector<std::thread> workers_;
	std::vector<ThreadPoolTask> tasks_;
	size_t tasksSize_;
	std::atomic_uint taskIndex_{0U};
	std::mutex queueMutex_;
	std::condition_variable cvTask_;
	std::atomic_uint jobsToFinish_{0U};
	std::atomic_bool stop_{false};
	std::atomic_bool sleep_{true};
	static constexpr size_t K{3};

	void thread_proc()
	{
		std::optional<std::function<void()>> fn;
		while (!stop_)
		{
			{
			std::unique_lock<std::mutex> lock(queueMutex_);
			cvTask_.wait(lock, [this]() { return !sleep_ || jobsToFinish_ || stop_; });
			}
			// Mutex and lock above is really just a quick hack to
			// get threads to sleep while no work is pending - there's
			// no critical section they're guarding ... the individual
			// work elements all have their own mutexes
			//lock.unlock();
			do
			{
				if (!stop_ && jobsToFinish_)
				{
					// Search through tasks, try to find one that
					// isn't locked by another worker thread and
					// also holds a valid task to run
					for (size_t n = 0; n < tasksSize_; n++)
					{
						fn = tasks_[n].get();
						if (fn.has_value())
						{
							// Run the function
							(*fn)();

							// When finished, there's one less job to complete
							--jobsToFinish_;
						}
					}
				}
			}
			while (!stop_ && !sleep_);
		}
	}
};

#endif

