#pragma once

#include "ros/ros.h"

// Simple class to turn debugging output on and off
class MessageFilter : public ros::console::FilterBase
{
	public:
		MessageFilter(bool enabled)
		{
			enabled_ = enabled;
		}
		void enable(void)
		{
			enabled_ = true;
		}
		void disable(void)
		{
			enabled_ = false;
		}
		bool isEnabled(void) override
		{
			return enabled_;
		}
		bool isEnabled(ros::console::FilterParams &) override
		{
			return isEnabled();
		}
	private:
		bool enabled_;
};

extern MessageFilter messageFilter;
