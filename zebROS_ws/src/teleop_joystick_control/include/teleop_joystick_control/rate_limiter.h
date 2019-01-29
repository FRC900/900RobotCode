#pragma once

#include <ros/ros.h>

// Class used to limit the rate of change of a value.
namespace rate_limiter
{

class RateLimiter
{
	public:
		RateLimiter(double min_val, double max_val, double rise_time_in_msec)
			: max_change_per_msec_((rise_time_in_msec > 0) ?
					((max_val - min_val) / rise_time_in_msec) : std::numeric_limits<double>::max())
		    , last_value_(0)
			, last_update_time_(0)
		{
		}

		double applyLimit(double value)
		{
			const ros::Time now = ros::Time::now();
			const double delta_msec = (now - last_update_time_).toSec() * 1000.;
			const double delta_value = delta_msec * max_change_per_msec_;
#if 0
			ROS_INFO_STREAM("applyLimit value:" << value
					<< " now:" << now
					<< " delta_msec:" << delta_msec
					<< " delta_value:" << delta_value
					<< " last_value_:" << last_value_
					<< " last_update_time_:" << last_update_time_);
#endif
			if (value > last_value_)
			{
				// Increasing value from previous setting?  If so, look for the
				// smaller of the requested value and the previous value + allowed change
				last_value_ = std::min(value, last_value_ + delta_value);
			}
			else
			{
				// Decreasing value from previous setting?  If so, look for the
				// larger of the requested value and the previous value - allowed change
				last_value_ = std::max(value, last_value_ - delta_value);
			}

			last_update_time_ = now;
			return last_value_;
		}

	private:
			double max_change_per_msec_;
			double last_value_;
			ros::Time last_update_time_;
};

}
