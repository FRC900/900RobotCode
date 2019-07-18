#pragma once

#include <ros/ros.h>

// Class used to limit the rate of change of a value.
// In this case, the value being limited is the commanded
// robot velocity.  The goal is to prevent the motors from
// being accelerated faster than they possibly can.
namespace rate_limiter
{

class RateLimiter
{
	public:
		RateLimiter(double min_val, double max_val, double rise_time_in_msec)
			: rise_time_in_msec_(rise_time_in_msec)
		    , last_value_(0)
			, last_update_time_(ros::Time::now())
		{
			updateMinMax(min_val, max_val);
		}

		double applyLimit(double value, const ros::Time &now)
		{
			const double delta_msec = (now - last_update_time_).toSec() * 1000.;
			const double delta_value = delta_msec * max_change_per_msec_;
#if 0
			ROS_INFO_STREAM(__LINE__ << " applyLimit value:" << value
					<< " now:" << now
					<< " delta_msec:" << delta_msec
					<< " delta_value:" << delta_value
					<< " last_value_:" << last_value_
					<< " last_update_time_:" << last_update_time_);
#endif
			// Can happen if replaying rosbag files over and over
			// without restarting teleop node :/
			if (delta_msec < 0)
			{
				last_value_ = 0;
				last_update_time_ = now;
				return 0;
			}
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
#if 0
			ROS_INFO_STREAM(__LINE__ << " applyLimit value:" << value
					<< " now:" << now
					<< " delta_msec:" << delta_msec
					<< " delta_value:" << delta_value
					<< " last_value_:" << last_value_
					<< " last_update_time_:" << last_update_time_);
#endif
			return last_value_;
		}

		void updateMinMax(double min_val, double max_val)
		{
			if (min_val > max_val)
			{
				ROS_ERROR_STREAM("min_val greater than max value in rate limiter : "
						<< min_val << max_val);
			}
			if (rise_time_in_msec_ < 0)
				max_change_per_msec_ = std::numeric_limits<double>::max();
			else
				max_change_per_msec_ = fabs(max_val - min_val) / rise_time_in_msec_;
		}

	private:
			double    rise_time_in_msec_;
			double    max_change_per_msec_;
			double    last_value_;
			ros::Time last_update_time_;
};

}
