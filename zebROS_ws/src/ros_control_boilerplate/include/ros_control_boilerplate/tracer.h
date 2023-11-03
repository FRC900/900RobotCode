// Simple class to track cumulative / average execution
// time of a block of code

#ifndef TRACER_INC__
#define TRACER_INC__

#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>

#include <ros/console.h>

class TracerEntry
{
	public:
		TracerEntry() = default;
		TracerEntry(const TracerEntry &) = delete;
		TracerEntry(TracerEntry &&) = default;
		virtual ~TracerEntry() = default;
		TracerEntry operator=(const TracerEntry &) = delete;
		TracerEntry &operator=(TracerEntry &&) = delete;

		void reset()
		{
			count_ = 0;
			total_time_ = std::chrono::duration<double>{0.0};
		}

		void start(const std::string &label)
		{
			if (started_)
			{
				ROS_WARN_STREAM("Tracer::start : start called on already started label " << label);
				return;
			}
			start_time_ = std::chrono::high_resolution_clock::now();
			started_ = true;
		}

		void stop(const std::string &label, bool report_unstarted)
		{
			if (!started_)
			{
				if (report_unstarted)
				{
					ROS_WARN_STREAM("Tracer::stop : label " << label << " not started");
				}
				return;
			}
			total_time_ += std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_);
			count_ += 1;
			started_ = false;
		}

		friend std::ostream& operator<<(std::ostream &os, const TracerEntry &tracer_entry)
		{
			const double avg_time = tracer_entry.total_time_ / std::chrono::duration<double>(tracer_entry.count_);
			os << avg_time << ", count = " << tracer_entry.count_;
			return os;
		}

	private:
		size_t count_{0};
		std::chrono::duration<double> total_time_{0.0};
		std::chrono::high_resolution_clock::time_point start_time_;
		bool started_{false};
};

class Tracer
{
	public:
		explicit Tracer(const std::string &name)
			: name_(name)
		{
		}
		Tracer(const Tracer &) = delete;
		Tracer(Tracer &&) = default;
		virtual ~Tracer() = default;
		Tracer operator=(const Tracer &) = delete;
		Tracer &operator=(Tracer &&) = delete;

		// Mark the start of an event to time.  If the named event exists,
		// use it. Otherwise create a new event entry in the map of events
		// tracked by this Tracer object
		void start(const std::string &label)
		{
			// try_emplace returns an iterator, bool pair.
			// iterator points either to the existing entry with
			// this label or to the new entry if one didn't exist
			auto [entry, placed] = map_.try_emplace(label);

			if (entry == map_.end())
			{
				ROS_ERROR_STREAM("Tracer::start : failed to insert label " << label);
				return;
			}

			entry->second.start(label);
		}

		// Stop all previously started timers,
		// then start the requested one
		void start_unique(const std::string &label)
		{
			stop();
			start(label);
		}

		void stop(const std::string &label)
		{
			auto entry = map_.find(label);

			// If not found, create a new entry for this label
			if (entry == map_.end())
			{
				ROS_ERROR_STREAM("Tracer::stop : couldn't find label " << label);
				return;
			}

			entry->second.stop(label, true);
		}

		// Stop all previously started timers
		void stop(void)
		{
			for (auto &[label, tracer_entry] : map_)
			{
				tracer_entry.stop(label, false);
			}
		}

		void report(const double timeout, const bool auto_stop = true)
		{
			if (auto_stop)
			{
				stop();
			}
			const auto now = ros::Time::now();
			if (((now - last_report_time_).toSec() >= timeout) ||
				(now < last_report_time_))
			{
				std::stringstream s;
				s << name_ << ":" << std::endl;
				for (auto &[name, tracer_entry] : map_)
				{
					s << "\t" << name << " = " << tracer_entry << std::endl;
					tracer_entry.reset();
				}
				ROS_INFO_STREAM(s.str());
				last_report_time_ = now;
			}
		}

	private:
		std::string name_;
		std::unordered_map<std::string, TracerEntry> map_;
		ros::Time last_report_time_{ros::Time::now()};
};

#endif